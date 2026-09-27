/* pipeline.c —— 端到端链路演示：图像 → 编码 → CSI-2 打包 → GVSP 分块传输（带丢包重传）
 *                → 收端重组 → 解码 → 校验，并输出 CSV 指标。
 *
 * 用法：
 *   clink_pipeline [--w 128] [--h 64] [--quality 75] [--loss 30] [--seed 20260927] [--out out]
 */
#include "clink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PKTS  512

static uint32_t rng_state = 1u;
static uint32_t rnd(void)
{
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x ? x : 0x9E3779B9u;
    return rng_state;
}

/* 造一张「有表面缺陷的检测样片」灰度图：基底织构 + 划痕 + 暗角 + 噪声 */
static void gen_image(uint8_t *img, uint32_t w, uint32_t h, uint32_t seed)
{
    rng_state = seed ? seed : 1u;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            double v = 158.0;
            v += 26.0 * ((double)(rnd() % 1000) / 1000.0 - 0.5);
            double cx = (double)x - w * 0.5, cy = (double)y - h * 0.5;
            double r2 = (cx * cx + cy * cy) / (2.0 * (w * 0.6) * (w * 0.6));
            v *= (1.0 - 0.18 * r2);
            img[(size_t)y * w + x] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
    }
    /* 一条斜向划痕 */
    for (uint32_t t = 0; t < w; ++t) {
        double yy = h * 0.30 + 0.35 * (double)t;
        for (int d = -1; d <= 1; ++d) {
            int y = (int)yy + d;
            if (t < w && y >= 0 && y < (int)h) {
                size_t idx = (size_t)y * w + t;
                img[idx] = (uint8_t)(img[idx] > 46 ? img[idx] - 46 : 0);
            }
        }
    }
}

/* ---- CSI-2：把一帧打成 FS + 每行(LS + 长包 + LE) + FE ---- */
static int csi2_send_frame(const uint8_t *img, uint32_t w, uint32_t h, uint8_t vc,
                           csi2_rx_t *rx, uint32_t *pkts_out, uint32_t *bytes_out,
                           uint32_t *ecc_err_out, uint32_t *crc_err_out)
{
    uint8_t buf[8 * 1024];
    size_t len = 0;
    uint32_t pkts = 0, bytes = 0;
    const uint32_t chunk = 256;      /* 每行长包最多带 256 字节 */

    if (csi2_pack_short(vc, CSI2_DT_FRAME_START, 0, buf, sizeof(buf), &len) != CLINK_OK) {
        return -1;
    }
    bytes += (uint32_t)len; pkts++;

    csi2_packet_t pkt;
    size_t consumed = 0;
    if (csi2_unpack(buf, len, &pkt, &consumed) != CLINK_OK) return -1;
    csi2_rx_feed(rx, &pkt);

    for (uint32_t y = 0; y < h; ++y) {
        csi2_pack_short(vc, CSI2_DT_LINE_START, (uint16_t)y, buf, sizeof(buf), &len);
        bytes += (uint32_t)len; pkts++;
        if (csi2_unpack(buf, len, &pkt, &consumed) != CLINK_OK) return -1;
        csi2_rx_feed(rx, &pkt);

        for (uint32_t off = 0; off < w; off += chunk) {
            uint32_t n = (w - off < chunk) ? (w - off) : chunk;
            csi2_pack_long(vc, CSI2_DT_RAW8, img + (size_t)y * w + off, (uint16_t)n,
                           buf, sizeof(buf), &len);
            bytes += (uint32_t)len; pkts++;
            if (csi2_unpack(buf, len, &pkt, &consumed) != CLINK_OK) return -1;
            csi2_rx_feed(rx, &pkt);
        }

        csi2_pack_short(vc, CSI2_DT_LINE_END, (uint16_t)y, buf, sizeof(buf), &len);
        bytes += (uint32_t)len; pkts++;
        if (csi2_unpack(buf, len, &pkt, &consumed) != CLINK_OK) return -1;
        csi2_rx_feed(rx, &pkt);
    }

    csi2_pack_short(vc, CSI2_DT_FRAME_END, 0, buf, sizeof(buf), &len);
    bytes += (uint32_t)len; pkts++;
    if (csi2_unpack(buf, len, &pkt, &consumed) != CLINK_OK) return -1;
    int done = csi2_rx_feed(rx, &pkt);

    if (pkts_out)   *pkts_out = pkts;
    if (bytes_out)  *bytes_out = bytes;
    if (ecc_err_out) *ecc_err_out = rx->ecc_errors;
    if (crc_err_out) *crc_err_out = rx->crc_errors;
    return done == 1 ? 0 : -1;
}

/* ---- GVSP：一块数据经丢包信道发送 + 重传，直到收齐 ---- */
typedef struct {
    uint32_t sent_pkts;
    uint32_t dropped;
    uint32_t duplicated;
    uint32_t reordered;
    uint32_t resends;
    uint32_t rounds;
} gvsp_stat_t;

static int gvsp_send_block(uint16_t block_id, const uint8_t *data, size_t len,
                           gvsp_channel_t *ch, gvsp_rx_t *rx, gvsp_stat_t *st)
{
    gvsp_packet_t pkts[MAX_PKTS];
    uint32_t ts = 0;
    int n = gvsp_split(block_id, data, len, pkts, MAX_PKTS, &ts);
    if (n < 0) {
        return n;
    }
    uint8_t wire[GVSP_MAX_PAYLOAD + 64];
    size_t wl = 0;
    int done = 0;
    for (int round = 0; round < (int)GVSP_MAX_RESENDS + 2 && !done; ++round) {
        st->rounds++;
        if (round == 0) {
            for (int i = 0; i < n; ++i) {
                st->sent_pkts++;
                if (gvsp_channel_drop(ch)) { st->dropped++; continue; }
                gvsp_serialize(&pkts[i], wire, sizeof(wire), &wl);
                gvsp_packet_t got;
                size_t c = 0;
                gvsp_deserialize(wire, wl, &got, &c);
                int r = gvsp_rx_feed(rx, &got);
                if (r == 1) { done = 1; break; }
                if (gvsp_channel_dup(ch)) {
                    st->duplicated++;
                    gvsp_rx_feed(rx, &got);
                }
                if (gvsp_channel_reorder(ch)) { st->reordered++; }
            }
        } else {
            /* 选择性重传：leader / 缺口 payload / trailer 三类都可能需要补。
             * trailer 每轮都补一次，用它触发完整性判定；
             * leader 用 rx->in_block 判断是否需要重来。 */
            st->resends++;
            for (int i = 0; i < n; ++i) {
                int need;
                if (i == 0) {
                    need = !rx->in_block;                       /* leader 丢了 */
                } else if (i == n - 1) {
                    need = 1;                                   /* trailer 每轮补 */
                } else {
                    need = !gvsp_rx_has_packet(rx, pkts[i].packet_id);
                }
                if (!need) {
                    continue;
                }
                st->sent_pkts++;
                if (gvsp_channel_drop(ch)) { st->dropped++; continue; }
                gvsp_serialize(&pkts[i], wire, sizeof(wire), &wl);
                gvsp_packet_t got;
                size_t c = 0;
                gvsp_deserialize(wire, wl, &got, &c);
                if (gvsp_rx_feed(rx, &got) == 1) { done = 1; }
            }
        }
    }
    return done ? CLINK_OK : CLINK_ERR_TIMEOUT;
}

int main(int argc, char **argv)
{
    uint32_t w = 128, h = 64, seed = 20260927u;
    int quality = 75;
    uint32_t loss = 30;
    const char *outdir = "out";

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--w") && i + 1 < argc)       w = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc)  h = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--quality") && i + 1 < argc) quality = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--loss") && i + 1 < argc)    loss = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc)    seed = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--out") && i + 1 < argc)     outdir = argv[++i];
    }
    if (w % 8 || h % 8) {
        fprintf(stderr, "宽高必须是 8 的整数倍\n");
        return 2;
    }

    printf("=== camera-link-lab 端到端链路仿真 ===\n");
    printf("图像 %ux%u  质量 %d  丢包率 %.1f%%  seed %u\n\n",
           w, h, quality, (double)loss / 10.0, seed);

    size_t npix = (size_t)w * h;
    uint8_t *img = (uint8_t *)malloc(npix);
    uint8_t *dec = (uint8_t *)malloc(npix);
    uint8_t *enc = (uint8_t *)malloc(npix * 4 + 4096);
    uint8_t *blob = (uint8_t *)malloc(npix * 4 + 4096);
    if (!img || !dec || !enc || !blob) {
        fprintf(stderr, "内存不足\n");
        return 2;
    }
    gen_image(img, w, h, seed);

    /* ---------------- 1) 编码 ---------------- */
    codec_stat_t st_rle, st_jpg;
    if (codec_encode(CODEC_RLE_DIFF, 100, img, w, h, enc, npix * 4 + 4096, &st_rle) != CLINK_OK) {
        fprintf(stderr, "RLE 编码失败\n"); return 2;
    }
    codec_decode(CODEC_RLE_DIFF, enc, st_rle.out_len, dec, w, h);
    int rle_identical = (memcmp(img, dec, npix) == 0);
    st_rle.psnr = codec_psnr(img, dec, npix);

    if (codec_encode(CODEC_JPEGLIKE, quality, img, w, h, enc, npix * 4 + 4096, &st_jpg) != CLINK_OK) {
        fprintf(stderr, "JPEG 风格编码失败\n"); return 2;
    }
    codec_decode(CODEC_JPEGLIKE, enc, st_jpg.out_len, dec, w, h);
    st_jpg.psnr = codec_psnr(img, dec, npix);

    printf("[1] 编码\n");
    printf("    无损差分 RLE    %6zu 字节  压缩比 %5.2fx  可逆=%s\n",
           st_rle.out_len, (double)npix / (double)st_rle.out_len, rle_identical ? "是" : "否");
    printf("    JPEG 风格(q=%d)  %6zu 字节  压缩比 %5.2fx  PSNR %.2f dB\n\n",
           quality, st_jpg.out_len, (double)npix / (double)st_jpg.out_len, st_jpg.psnr);

    /* ---------------- 2) CSI-2 帧链路 ---------------- */
    csi2_rx_t rx;
    if (csi2_rx_init(&rx, w, h, 0) != CLINK_OK) {
        fprintf(stderr, "CSI-2 接收器初始化失败\n"); return 2;
    }
    uint32_t pkts = 0, bytes = 0, ecc = 0, crc = 0;
    if (csi2_send_frame(img, w, h, 0, &rx, &pkts, &bytes, &ecc, &crc) != 0) {
        fprintf(stderr, "CSI-2 帧重组失败\n"); return 2;
    }
    int csi_ok = (memcmp(img, rx.frame, npix) == 0);
    printf("[2] MIPI CSI-2 包层\n");
    printf("    包数 %u（短包 %u / 长包 %u）  线字节 %u  重组一致=%s  行异常 %u\n\n",
           pkts, rx.short_packets, rx.long_packets, bytes, csi_ok ? "是" : "否",
           rx.line_overflows);
    csi2_rx_free(&rx);

    /* ---------------- 3) GVSP 分块传输 + 丢包重传 ---------------- */
    size_t blob_len = st_jpg.out_len;
    /* 直接复用刚编好的这一份。**不要重编** —— 重编会把 st_jpg.psnr 冲回 0，
     * 于是 metrics.csv 里的 jpeg_psnr_db 就成了 0（这个 bug 就是这样被发现的）。 */
    memcpy(blob, enc, blob_len);

    gvsp_channel_t ch;
    gvsp_channel_init(&ch, seed, loss, loss / 4, loss / 4);
    gvsp_rx_t grx;
    if (gvsp_rx_init(&grx, blob_len + GVSP_MAX_PAYLOAD) != CLINK_OK) {
        fprintf(stderr, "GVSP 接收器初始化失败\n"); return 2;
    }
    gvsp_stat_t gs;
    memset(&gs, 0, sizeof(gs));
    int gv_rc = gvsp_send_block(7, blob, blob_len, &ch, &grx, &gs);

    int blob_ok = (gv_rc == CLINK_OK) && (grx.got_len == blob_len) &&
                  (memcmp(grx.buf, blob, blob_len) == 0);
    printf("[3] GVSP 分块传输（丢包率 %.1f%%）\n", (double)loss / 10.0);
    printf("    包数 %u  丢失 %u  重复 %u  乱序 %u  重传请求 %u 轮次 %u\n",
           gs.sent_pkts, gs.dropped, gs.duplicated, gs.reordered, gs.resends, gs.rounds);
    printf("    收端：包 OK %u / 重复 %u / 乱序 %u  块完成 %u  重组一致=%s\n\n",
           grx.packets_ok, grx.packets_dup, grx.packets_out_of_order, grx.blocks_done,
           blob_ok ? "是" : "否");

    /* ---------------- 4) 解码还原 ---------------- */
    uint8_t *final_img = (uint8_t *)malloc(npix);
    int dec_rc = codec_decode(CODEC_JPEGLIKE, grx.buf, blob_len, final_img, w, h);
    double psnr_e2e = (dec_rc == CLINK_OK) ? codec_psnr(img, final_img, npix) : 0.0;
    printf("[4] 解码还原\n");
    printf("    端到端 PSNR %.2f dB（编码 → 传输 → 重组 → 解码）\n\n", psnr_e2e);

    /* ---------------- 5) 指标落盘 ---------------- */
    char path[512];
    snprintf(path, sizeof(path), "%s/metrics.csv", outdir);
    FILE *fp = fopen(path, "w");
    if (fp) {
        fprintf(fp, "metric,value\n");
        fprintf(fp, "width,%u\nheight,%u\n", w, h);
        fprintf(fp, "pixels,%zu\n", npix);
        fprintf(fp, "rle_bytes,%zu\nrle_ratio,%.4f\nrle_lossless,%d\n",
                st_rle.out_len, (double)npix / (double)st_rle.out_len, rle_identical);
        fprintf(fp, "jpeg_bytes,%zu\njpeg_ratio,%.4f\njpeg_quality,%d\njpeg_psnr_db,%.4f\n",
                st_jpg.out_len, (double)npix / (double)st_jpg.out_len, quality, st_jpg.psnr);
        fprintf(fp, "csi_packets,%u\ncsi_short,%u\ncsi_long,%u\ncsi_bytes,%u\ncsi_reassembled,%d\n",
                pkts, rx.short_packets, rx.long_packets, bytes, csi_ok);
        fprintf(fp, "gvsp_packets,%u\ngvsp_dropped,%u\ngvsp_duplicated,%u\ngvsp_reordered,%u\n",
                gs.sent_pkts, gs.dropped, gs.duplicated, gs.reordered);
        fprintf(fp, "gvsp_resend_requests,%u\ngvsp_rounds,%u\ngvsp_blocks_done,%u\ngvsp_reassembled,%d\n",
                gs.resends, gs.rounds, grx.blocks_done, blob_ok);
        fprintf(fp, "e2e_psnr_db,%.4f\n", psnr_e2e);
        fclose(fp);
        printf("[5] 指标已写入 %s\n", path);
    }

    int ok = rle_identical && csi_ok && blob_ok && (dec_rc == CLINK_OK);
    printf("\n结论：%s\n", ok ? "全链路通过（无损可逆 / CSI-2 重组一致 / GVSP 丢包重传补齐）"
                             : "★ 有环节未通过，见上方输出");

    free(img); free(dec); free(enc); free(blob); free(final_img);
    gvsp_rx_free(&grx);
    return ok ? 0 : 1;
}
