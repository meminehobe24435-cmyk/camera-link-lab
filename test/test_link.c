/* test_link.c —— camera-link-lab 单元测试（零第三方依赖，自带断言宏） */
#include "clink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;
static const char *g_case = "";

#define CHECK(cond, msg) do {                                              \
    if (cond) { g_pass++; }                                                \
    else { g_fail++; printf("  FAIL [%s] %s  (%s:%d)\n",                   \
                            g_case, msg, __FILE__, __LINE__); }            \
} while (0)

#define CASE(name) do { g_case = name; } while (0)

/* ------------------------------------------------------------------ ECC */
static void test_ecc(void)
{
    CASE("ecc/clean");
    uint32_t h = csi2_header24(2, CSI2_DT_RAW8, 1234);
    uint8_t e = csi2_ecc6(h);
    uint32_t fix = 0;
    CHECK(csi2_ecc_check(h, e, &fix) == CLINK_OK, "无错包头应通过");
    CHECK(fix == h, "无错时应原样返回");

    CASE("ecc/single-bit-correct");
    int corrected = 0;
    for (int b = 0; b < 24; ++b) {
        uint32_t bad = h ^ (1u << b);
        uint32_t out = 0;
        if (csi2_ecc_check(bad, e, &out) == CLINK_OK && out == h) {
            corrected++;
        }
    }
    CHECK(corrected == 24, "24 位里任意 1 位翻转都应被纠正回来");

    CASE("ecc/two-bit-detect");
    uint32_t bad2 = h ^ (1u << 3) ^ (1u << 11);
    CHECK(csi2_ecc_check(bad2, e, NULL) == CLINK_ERR_ECC, "2 位错应报错而不是误纠");
}

/* ------------------------------------------------------------------ CRC */
static void test_crc(void)
{
    CASE("crc/known-vector");
    /* CRC-16/CCITT-FALSE("123456789") = 0x29B1 */
    const uint8_t v[] = "123456789";
    CHECK(csi2_crc16(v, 9) == 0x29B1u, "CRC-16/CCITT 标准测试向量应为 0x29B1");

    CASE("crc/single-bit-sensitive");
    uint8_t a[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t b[8];
    memcpy(b, a, 8);
    b[4] ^= 0x01;
    CHECK(csi2_crc16(a, 8) != csi2_crc16(b, 8), "1 位变化应改变 CRC");
}

/* ----------------------------------------------------------------- 组包 */
static void test_pack(void)
{
    CASE("pack/short-roundtrip");
    uint8_t buf[4096];
    size_t len = 0;
    CHECK(csi2_pack_short(1, CSI2_DT_LINE_START, 37, buf, sizeof(buf), &len) == CLINK_OK,
          "短包打包成功");
    CHECK(len == CSI2_SHORT_PACKET_LEN, "短包应为 4 字节");
    csi2_packet_t pkt;
    size_t c = 0;
    CHECK(csi2_unpack(buf, len, &pkt, &c) == CLINK_OK, "短包解包成功");
    CHECK(pkt.is_long == 0, "短包标志正确");
    CHECK(pkt.di.vc == 1 && pkt.di.dt == CSI2_DT_LINE_START && pkt.di.wc == 37,
          "短包字段往返一致");

    CASE("pack/long-roundtrip");
    uint8_t pl[300];
    for (int i = 0; i < 300; ++i) {
        pl[i] = (uint8_t)(i * 7 + 3);
    }
    CHECK(csi2_pack_long(0, CSI2_DT_RAW8, pl, 300, buf, sizeof(buf), &len) == CLINK_OK,
          "长包打包成功");
    CHECK(len == 4 + 300 + 2, "长包长度 = 头 4 + 载荷 + CRC 2");
    CHECK(csi2_unpack(buf, len, &pkt, &c) == CLINK_OK, "长包解包成功");
    CHECK(pkt.is_long == 1 && pkt.payload_len == 300, "长包字段正确");
    CHECK(memcmp(pkt.payload, pl, 300) == 0, "长包载荷往返一致");

    CASE("pack/long-crc-corrupt");
    buf[10] ^= 0x40;
    CHECK(csi2_unpack(buf, len, &pkt, &c) == CLINK_ERR_CRC, "载荷被改应报 CRC 错");

    CASE("pack/capacity");
    CHECK(csi2_pack_long(0, CSI2_DT_RAW8, pl, 300, buf, 16, &len) == CLINK_ERR_CAPACITY,
          "缓冲区不够应报错");
    CHECK(csi2_pack_long(0, CSI2_DT_RAW8, pl, 0, buf, sizeof(buf), &len) == CLINK_ERR_PARAM,
          "零长载荷应报参数错");
}

/* --------------------------------------------------------------- 帧重组 */
static void test_frame_reassembly(void)
{
    CASE("frame/reassemble");
    const uint32_t w = 32, h = 8;
    uint8_t img[32 * 8];
    for (uint32_t i = 0; i < w * h; ++i) {
        img[i] = (uint8_t)(i * 3);
    }
    csi2_rx_t rx;
    CHECK(csi2_rx_init(&rx, w, h, 0) == CLINK_OK, "接收器初始化");

    uint8_t buf[512];
    size_t len = 0;
    csi2_packet_t pkt;
    size_t c = 0;
    csi2_pack_short(0, CSI2_DT_FRAME_START, 0, buf, sizeof(buf), &len);
    csi2_unpack(buf, len, &pkt, &c);
    csi2_rx_feed(&rx, &pkt);

    int done = 0;
    for (uint32_t y = 0; y < h; ++y) {
        csi2_pack_short(0, CSI2_DT_LINE_START, (uint16_t)y, buf, sizeof(buf), &len);
        csi2_unpack(buf, len, &pkt, &c);
        csi2_rx_feed(&rx, &pkt);

        csi2_pack_long(0, CSI2_DT_RAW8, img + y * w, (uint16_t)w, buf, sizeof(buf), &len);
        csi2_unpack(buf, len, &pkt, &c);
        csi2_rx_feed(&rx, &pkt);

        csi2_pack_short(0, CSI2_DT_LINE_END, (uint16_t)y, buf, sizeof(buf), &len);
        csi2_unpack(buf, len, &pkt, &c);
        csi2_rx_feed(&rx, &pkt);
    }
    csi2_pack_short(0, CSI2_DT_FRAME_END, 0, buf, sizeof(buf), &len);
    csi2_unpack(buf, len, &pkt, &c);
    done = csi2_rx_feed(&rx, &pkt);

    CHECK(done == 1, "帧结束应返回 1");
    CHECK(memcmp(rx.frame, img, w * h) == 0, "重组后的帧应与原图逐字节一致");
    CHECK(rx.lines_done == h, "行计数应为 8");
    CHECK(rx.line_overflows == 0, "不应出现行异常");

    CASE("frame/vc-filter");
    csi2_rx_t rx2;
    csi2_rx_init(&rx2, w, h, 3);
    csi2_pack_short(0, CSI2_DT_FRAME_START, 0, buf, sizeof(buf), &len);
    csi2_unpack(buf, len, &pkt, &c);
    csi2_rx_feed(&rx2, &pkt);
    CHECK(rx2.in_frame == 0, "别的虚拟通道的包应被忽略");

    csi2_rx_free(&rx);
    csi2_rx_free(&rx2);
}

/* ------------------------------------------------------------ GVSP 拆分 */
static void test_gvsp_split(void)
{
    CASE("gvsp/split-count");
    uint8_t data[5000];
    for (int i = 0; i < 5000; ++i) {
        data[i] = (uint8_t)(i & 0xFF);
    }
    gvsp_packet_t pkts[16];
    uint32_t ts = 0;
    int n = gvsp_split(3, data, 5000, pkts, 16, &ts);
    CHECK(n == 2 + 4, "5000 字节 / 1440 → 4 个载荷包 + leader + trailer = 6");
    CHECK(pkts[0].kind == GVSP_LEADER && pkts[0].packet_id == 0, "首包应为 leader 且 id=0");
    CHECK(pkts[n - 1].kind == GVSP_TRAILER, "末包应为 trailer");
    CHECK(pkts[0].payload_len == 5000, "leader 应声明整块长度");
    CHECK(pkts[1].packet_id == 1 && pkts[n - 1].packet_id == 5, "包序号应连续");
    CHECK(pkts[1].data_len == GVSP_MAX_PAYLOAD, "中间载荷包应满");
    CHECK(pkts[n - 2].data_len == 5000 - 3 * GVSP_MAX_PAYLOAD, "末个载荷包为余数");

    CASE("gvsp/serialize-roundtrip");
    uint8_t wire[GVSP_MAX_PAYLOAD + 64];
    size_t wl = 0;
    CHECK(gvsp_serialize(&pkts[1], wire, sizeof(wire), &wl) == CLINK_OK, "序列化成功");
    CHECK(wl == (size_t)17 + (size_t)pkts[1].data_len, "线格式长度正确");
    gvsp_packet_t got;
    size_t c = 0;
    CHECK(gvsp_deserialize(wire, wl, &got, &c) == CLINK_OK, "反序列化成功");
    CHECK(got.kind == pkts[1].kind && got.block_id == pkts[1].block_id &&
          got.packet_id == pkts[1].packet_id && got.data_len == pkts[1].data_len,
          "字段往返一致");
    CHECK(memcmp(got.data, pkts[1].data, got.data_len) == 0, "载荷往返一致");

    CASE("gvsp/split-capacity");
    CHECK(gvsp_split(1, data, 5000, pkts, 3, &ts) == CLINK_ERR_CAPACITY, "包数不够应报错");
}

/* ------------------------------------------------- GVSP 无丢包 / 有丢包 */
static int drive_block(const uint8_t *data, size_t len, uint32_t seed,
                       uint32_t loss, int use_resend, gvsp_rx_t *rx,
                       uint32_t *resends, uint32_t *dropped)
{
    gvsp_packet_t pkts[64];
    uint32_t ts = 0;
    int n = gvsp_split(11, data, len, pkts, 64, &ts);
    if (n < 0) {
        return n;
    }
    gvsp_channel_t ch;
    gvsp_channel_init(&ch, seed, loss, 0, 0);
    uint8_t wire[GVSP_MAX_PAYLOAD + 64];
    size_t wl = 0;
    int done = 0;

    for (int round = 0; round < (int)GVSP_MAX_RESENDS + 2; ++round) {
        if (round == 0) {
            for (int i = 0; i < n; ++i) {
                if (gvsp_channel_drop(&ch)) {
                    if (dropped) { (*dropped)++; }
                    continue;
                }
                gvsp_serialize(&pkts[i], wire, sizeof(wire), &wl);
                gvsp_packet_t got; size_t c = 0;
                gvsp_deserialize(wire, wl, &got, &c);
                int r = gvsp_rx_feed(rx, &got);
                if (r == 1) { done = 1; break; }
            }
        } else {
            if (!use_resend) {
                break;
            }
            if (resends) { (*resends)++; }
            for (int i = 0; i < n; ++i) {
                int need;
                if (i == 0) {
                    need = !rx->in_block;
                } else if (i == n - 1) {
                    need = 1;
                } else {
                    need = !gvsp_rx_has_packet(rx, pkts[i].packet_id);
                }
                if (!need) {
                    continue;
                }
                if (gvsp_channel_drop(&ch)) {
                    if (dropped) { (*dropped)++; }
                    continue;
                }
                gvsp_serialize(&pkts[i], wire, sizeof(wire), &wl);
                gvsp_packet_t got; size_t c = 0;
                gvsp_deserialize(wire, wl, &got, &c);
                if (gvsp_rx_feed(rx, &got) == 1) { done = 1; }
            }
        }
        if (done) {
            break;
        }
    }
    return done ? CLINK_OK : CLINK_ERR_TIMEOUT;
}

static void test_gvsp_transfer(void)
{
    static uint8_t data[9000];
    for (int i = 0; i < 9000; ++i) {
        data[i] = (uint8_t)((i * 31 + 7) & 0xFF);
    }

    CASE("gvsp/clean-channel");
    gvsp_rx_t rx;
    gvsp_rx_init(&rx, sizeof(data) + GVSP_MAX_PAYLOAD);
    uint32_t rs = 0, dr = 0;
    CHECK(drive_block(data, sizeof(data), 1, 0, 1, &rx, &rs, &dr) == CLINK_OK,
          "无丢包时应一次收齐");
    CHECK(rx.got_len == sizeof(data), "长度一致");
    CHECK(memcmp(rx.buf, data, sizeof(data)) == 0, "内容一致");
    CHECK(rs == 0 && dr == 0, "不应有丢包与重传");
    gvsp_rx_free(&rx);

    CASE("gvsp/loss-without-resend");
    gvsp_rx_init(&rx, sizeof(data) + GVSP_MAX_PAYLOAD);
    rs = 0; dr = 0;
    int rc = drive_block(data, sizeof(data), 7, 200, 0, &rx, &rs, &dr);
    CHECK(rc == CLINK_ERR_TIMEOUT, "高丢包且不重传应超时");
    CHECK(dr > 0, "应确实丢了包");
    gvsp_rx_free(&rx);

    CASE("gvsp/loss-with-resend");
    gvsp_rx_init(&rx, sizeof(data) + GVSP_MAX_PAYLOAD);
    rs = 0; dr = 0;
    CHECK(drive_block(data, sizeof(data), 7, 200, 1, &rx, &rs, &dr) == CLINK_OK,
          "有重传应最终收齐");
    CHECK(memcmp(rx.buf, data, sizeof(data)) == 0, "重传补齐后内容必须完全一致");
    CHECK(rs >= 1, "应至少发过一次重传请求");
    gvsp_rx_free(&rx);

    CASE("gvsp/duplicate-is-not-double-written");
    gvsp_rx_init(&rx, sizeof(data) + GVSP_MAX_PAYLOAD);
    gvsp_packet_t pkts[64];
    uint32_t ts = 0;
    int n = gvsp_split(11, data, sizeof(data), pkts, 64, &ts);
    uint8_t wire[GVSP_MAX_PAYLOAD + 64];
    size_t wl = 0;
    for (int i = 0; i < n; ++i) {
        gvsp_serialize(&pkts[i], wire, sizeof(wire), &wl);
        gvsp_packet_t got; size_t c = 0;
        gvsp_deserialize(wire, wl, &got, &c);
        gvsp_rx_feed(&rx, &got);
        if (i == 1) {
            gvsp_rx_feed(&rx, &got);        /* 故意重复一次 */
        }
    }
    CHECK(rx.packets_dup == 1, "重复包应被计数");
    CHECK(memcmp(rx.buf, data, sizeof(data)) == 0, "重复包不应破坏数据");
    gvsp_rx_free(&rx);
}

/* ------------------------------------------------------------------ 编码 */
static void test_codec(void)
{
    const uint32_t w = 64, h = 32;
    size_t n = w * h;
    uint8_t *img = malloc(n), *enc = malloc(n * 4 + 4096), *dec = malloc(n);
    for (size_t i = 0; i < n; ++i) {
        img[i] = (uint8_t)((i % 17) * 13 + 40);
    }

    CASE("codec/rle-lossless");
    codec_stat_t st;
    CHECK(codec_encode(CODEC_RLE_DIFF, 100, img, w, h, enc, n * 4 + 4096, &st) == CLINK_OK,
          "RLE 编码成功");
    CHECK(codec_decode(CODEC_RLE_DIFF, enc, st.out_len, dec, w, h) == CLINK_OK, "RLE 解码成功");
    CHECK(memcmp(img, dec, n) == 0, "RLE 必须无损可逆");
    CHECK(st.lossless == 1, "RLE 应标记为无损");

    CASE("codec/rle-compresses-flat");
    memset(img, 120, n);
    codec_encode(CODEC_RLE_DIFF, 100, img, w, h, enc, n * 4 + 4096, &st);
    CHECK(st.out_len < n / 8, "纯色图 RLE 压缩比应远大于 8x");
    codec_decode(CODEC_RLE_DIFF, enc, st.out_len, dec, w, h);
    CHECK(memcmp(img, dec, n) == 0, "纯色图也要可逆");

    CASE("codec/jpeg-quality-monotonic");
    for (size_t i = 0; i < n; ++i) {
        img[i] = (uint8_t)((i * 5) % 256);
    }
    codec_stat_t lo, hi;
    codec_encode(CODEC_JPEGLIKE, 20, img, w, h, enc, n * 4 + 4096, &lo);
    codec_decode(CODEC_JPEGLIKE, enc, lo.out_len, dec, w, h);
    lo.psnr = codec_psnr(img, dec, n);
    codec_encode(CODEC_JPEGLIKE, 90, img, w, h, enc, n * 4 + 4096, &hi);
    codec_decode(CODEC_JPEGLIKE, enc, hi.out_len, dec, w, h);
    hi.psnr = codec_psnr(img, dec, n);
    CHECK(hi.psnr > lo.psnr, "质量 90 的 PSNR 应高于质量 20");
    CHECK(lo.psnr > 10.0, "即便低质量 PSNR 也不应低到离谱");
    CHECK(hi.out_len > lo.out_len, "高质量应产生更多字节");

    CASE("codec/jpeg-rejects-bad-header");
    CHECK(codec_decode(CODEC_JPEGLIKE, enc, 4, dec, w, h) == CLINK_ERR_FORMAT,
          "头不完整应报格式错");

    CASE("codec/psnr-identical");
    CHECK(codec_psnr(img, img, n) > 1e29, "完全一致时 PSNR 应为无穷大");

    free(img); free(enc); free(dec);
}

/* ------------------------------------------------------------------ 主函数 */
int main(void)
{
    printf("=== camera-link-lab 单元测试 ===\n");
    test_ecc();
    test_crc();
    test_pack();
    test_frame_reassembly();
    test_gvsp_split();
    test_gvsp_transfer();
    test_codec();
    printf("\n==== 结果：%d passed, %d failed ====\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
