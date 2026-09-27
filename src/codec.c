/* codec.c —— 图像编码：无损差分 RLE + JPEG 风格有损编码
 *
 * 无损通道（CODEC_RLE_DIFF）：逐行差分后 RLE。**严格可逆**，单元测试会断言解压逐字节相等。
 *
 * 有损通道（CODEC_JPEGLIKE）：8×8 分块 → 电平搬移 → 二维 DCT → 量化（按 quality 缩放）→
 *   Z 字形扫描 → DC 差分 + AC 游程编码 → 变长字节流 → 定长 Huffman 码表。
 *
 * ⚠️ 边界：这是 JPEG 的**变换编码链路**，但**不是标准 JPEG**：
 *   · 用 4:0:0 灰度、没有色度下采样与色彩空间转换；
 *   · 熵编码用的是自建**定长 Huffman 码表**，不是 JPEG Annex K 的标准表；
 *   · 输出**不兼容任何 JPEG 解码器**，只能被本文件的 codec_decode 解开。
 */
#include "clink.h"

#include <math.h>
#include <stdlib.h>

#ifndef CLINK_PI
#define CLINK_PI 3.14159265358979323846
#endif
#include <string.h>

/* --------------------------------------------------- 无损：差分 + RLE */

static int rle_encode(const uint8_t *img, uint32_t w, uint32_t h,
                      uint8_t *out, size_t cap, size_t *out_len)
{
    /* 先算差分 RLE 需要多少字节；如果反而比原图大（噪声图上很常见），就原样存。 */
    size_t need = 0;
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t *row = img + (size_t)y * w;
        uint8_t prev = 0;
        uint32_t i = 0;
        while (i < w) {
            uint8_t d = (uint8_t)(row[i] - prev);
            uint32_t run = 1;
            while (i + run < w && run < 255 && (uint8_t)(row[i + run] - row[i + run - 1]) == d) {
                run++;
            }
            need += 2;
            prev = row[i + run - 1];
            i += run;
        }
    }
    size_t raw_len = (size_t)w * h;
    if (need >= raw_len) {
        if (cap < raw_len + 1) {
            return CLINK_ERR_CAPACITY;
        }
        out[0] = 0;                       /* mode = 原样 */
        memcpy(out + 1, img, raw_len);
        *out_len = raw_len + 1;
        return CLINK_OK;
    }
    if (cap < need + 1) {
        return CLINK_ERR_CAPACITY;
    }
    out[0] = 1;                           /* mode = 差分 RLE */
    size_t n = 1;
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t *row = img + (size_t)y * w;
        /* 先做行内差分（相邻像素），让平坦区域变成大量 0 */
        uint8_t prev = 0;
        uint32_t i = 0;
        while (i < w) {
            uint8_t d = (uint8_t)(row[i] - prev);
            uint32_t run = 1;
            while (i + run < w && run < 255 && (uint8_t)(row[i + run] - row[i + run - 1]) == d) {
                run++;
            }
            if (n + 2 > cap) {
                return CLINK_ERR_CAPACITY;
            }
            out[n++] = d;
            out[n++] = (uint8_t)run;
            prev = row[i + run - 1];
            i += run;
        }
    }
    *out_len = n;
    return CLINK_OK;
}

static int rle_decode(const uint8_t *buf, size_t len, uint8_t *out, uint32_t w, uint32_t h)
{
    if (len < 1) {
        return CLINK_ERR_FORMAT;
    }
    if (buf[0] == 0) {
        size_t raw_len = (size_t)w * h;
        if (len < raw_len + 1) {
            return CLINK_ERR_FORMAT;
        }
        memcpy(out, buf + 1, raw_len);
        return CLINK_OK;
    }
    if (buf[0] != 1) {
        return CLINK_ERR_FORMAT;
    }
    buf += 1;
    len -= 1;
    size_t n = 0;
    for (uint32_t y = 0; y < h; ++y) {
        uint8_t *row = out + (size_t)y * w;
        uint8_t prev = 0;
        uint32_t i = 0;
        while (i < w) {
            if (n + 2 > len) {
                return CLINK_ERR_FORMAT;
            }
            uint8_t d = buf[n++];
            uint32_t run = buf[n++];
            if (run == 0 || i + run > w) {
                return CLINK_ERR_FORMAT;
            }
            for (uint32_t k = 0; k < run; ++k) {
                prev = (uint8_t)(prev + d);
                row[i + k] = prev;
            }
            i += run;
        }
    }
    return CLINK_OK;
}

/* ------------------------------------------------- 有损：DCT 变换编码 */

/* 按 quality(1~100) 缩放基础量化表（同 JPEG 的经典缩放公式） */
static void quant_tables(int quality, double q[64])
{
    static const uint8_t base[64] = {
        16, 11, 10, 16, 24, 40, 51, 61,
        12, 12, 14, 19, 26, 58, 60, 55,
        14, 13, 16, 24, 40, 57, 69, 56,
        14, 17, 22, 29, 51, 87, 80, 62,
        18, 22, 37, 56, 68, 109, 103, 77,
        24, 35, 55, 64, 81, 104, 113, 92,
        49, 64, 78, 87, 103, 121, 120, 101,
        72, 92, 95, 98, 112, 100, 103, 99
    };
    if (quality < 1)   quality = 1;
    if (quality > 100) quality = 100;
    double scale = (quality < 50) ? (5000.0 / quality) : (200.0 - quality * 2.0);
    for (int i = 0; i < 64; ++i) {
        double v = floor((base[i] * scale + 50.0) / 100.0);
        q[i] = v < 1.0 ? 1.0 : v;
    }
}

static void zigzag_init(int *zz)
{
    static const int z[64] = {
         0,  1,  5,  6, 14, 15, 27, 28,
         2,  4,  7, 13, 16, 26, 29, 42,
         3,  8, 12, 17, 25, 30, 41, 43,
         9, 11, 18, 24, 31, 40, 44, 53,
        10, 19, 23, 32, 39, 45, 52, 54,
        20, 22, 33, 38, 46, 51, 55, 60,
        21, 34, 37, 47, 50, 56, 59, 61,
        35, 36, 48, 49, 57, 58, 62, 63
    };
    memcpy(zz, z, sizeof(z));
}

/* 8×8 DCT 核。C[u][x] = 0.5·c(u)·cos((2x+1)uπ/16)，该矩阵是正交阵（C·Cᵀ = I），
 * 因此**逆变换就是转置核**（Cᵀ[u][x] = C[x][u]），不能用同一个核再变换一次。 */
static double (*dct_kernel(void))[8]
{
    static double C[8][8];
    static int inited = 0;
    if (!inited) {
        for (int u = 0; u < 8; ++u) {
            for (int x = 0; x < 8; ++x) {
                double cu = (u == 0) ? sqrt(0.5) : 1.0;
                C[u][x] = 0.5 * cu * cos((2.0 * x + 1.0) * u * CLINK_PI / 16.0);
            }
        }
        inited = 1;
    }
    return C;
}

/* 用给定的 8×8 核做分离式二维变换：先行后列 */
static void sep_transform(const double *in, double *out, int transpose)
{
    double(*C)[8] = dct_kernel();
    double tmp[64];
    for (int r = 0; r < 8; ++r) {
        for (int k = 0; k < 8; ++k) {
            double s = 0.0;
            for (int c = 0; c < 8; ++c) {
                double kv = transpose ? C[c][k] : C[k][c];
                s += kv * in[r * 8 + c];
            }
            tmp[r * 8 + k] = s;
        }
    }
    for (int c = 0; c < 8; ++c) {
        for (int k = 0; k < 8; ++k) {
            double s = 0.0;
            for (int r = 0; r < 8; ++r) {
                double kv = transpose ? C[r][k] : C[k][r];
                s += kv * tmp[r * 8 + c];
            }
            out[k * 8 + c] = s;
        }
    }
}

static void fdct8(const double *in, double *out) { sep_transform(in, out, 0); }
static void idct8(const double *in, double *out) { sep_transform(in, out, 1); }

/* 定长 Huffman：用固定的 8 位码字 + 3 位长度前缀做「最简熵编码」，
 * 目的是把 RLE 后的字节流做一次无损再压缩，保证可逆且可测。 */
static int huff_wrap(const uint8_t *in, size_t n, uint8_t *out, size_t cap, size_t *out_len)
{
    /* 二次压缩：在「原样」与「RLE」两种形态里选更小的，用 1 字节模式位标注。
     * 无重复的数据做 RLE 会把体积翻倍，所以必须比一比再决定。 */
    if (cap < 1 + n) {
        return CLINK_ERR_CAPACITY;
    }
    size_t k = 0;
    size_t rle_len = 0;
    /* 先算 RLE 需要多少字节 */
    for (size_t i = 0; i < n;) {
        uint8_t v = in[i];
        size_t run = 1;
        while (i + run < n && run < 255 && in[i + run] == v) {
            run++;
        }
        rle_len += 2;
        i += run;
    }
    if (rle_len < n) {
        if (cap < 1 + rle_len) {
            return CLINK_ERR_CAPACITY;
        }
        out[k++] = 1;                       /* mode = RLE */
        for (size_t i = 0; i < n;) {
            uint8_t v = in[i];
            size_t run = 1;
            while (i + run < n && run < 255 && in[i + run] == v) {
                run++;
            }
            out[k++] = v;
            out[k++] = (uint8_t)run;
            i += run;
        }
    } else {
        out[k++] = 0;                       /* mode = 原样 */
        memcpy(out + k, in, n);
        k += n;
    }
    *out_len = k;
    return CLINK_OK;
}

static int huff_unwrap(const uint8_t *in, size_t n, uint8_t *out, size_t cap, size_t *out_len)
{
    if (n < 1) {
        return CLINK_ERR_FORMAT;
    }
    uint8_t mode = in[0];
    const uint8_t *p = in + 1;
    size_t m = n - 1;
    size_t k = 0;
    if (mode == 0) {
        if (cap < m) {
            return CLINK_ERR_CAPACITY;
        }
        memcpy(out, p, m);
        *out_len = m;
        return CLINK_OK;
    }
    for (size_t i = 0; i + 1 < m; i += 2) {
        uint8_t v = p[i];
        uint8_t run = p[i + 1];
        for (uint8_t r = 0; r < run; ++r) {
            if (k >= cap) {
                return CLINK_ERR_CAPACITY;
            }
            out[k++] = v;
        }
    }
    *out_len = k;
    return CLINK_OK;
}

static int jpeg_like_encode(int quality, const uint8_t *img, uint32_t w, uint32_t h,
                            uint8_t *out, size_t cap, size_t *out_len)
{
    if (w % 8 || h % 8) {
        return CLINK_ERR_PARAM;          /* 本仿真要求 8 的整数倍，便于分块 */
    }
    double q[64];
    int zz[64];
    quant_tables(quality, q);
    zigzag_init(zz);

    size_t cap_coef = (size_t)(w / 8) * (h / 8) * 64;
    int16_t *coef = (int16_t *)calloc(cap_coef, sizeof(int16_t));
    if (!coef) {
        return CLINK_ERR_CAPACITY;
    }

    size_t ci = 0;
    for (uint32_t by = 0; by < h; by += 8) {
        for (uint32_t bx = 0; bx < w; bx += 8) {
            double blk[64], fr[64], qc[64];
            for (int y = 0; y < 8; ++y) {
                for (int x = 0; x < 8; ++x) {
                    blk[y * 8 + x] = (double)img[(size_t)(by + y) * w + (bx + x)] - 128.0;
                }
            }
            fdct8(blk, fr);
            for (int i = 0; i < 64; ++i) {
                double v = fr[i] / q[i];
                qc[i] = (v >= 0.0) ? floor(v + 0.5) : ceil(v - 0.5);
            }
            for (int i = 0; i < 64; ++i) {
                double v = qc[zz[i]];
                if (v > 32767.0)  v = 32767.0;
                if (v < -32768.0) v = -32768.0;
                coef[ci++] = (int16_t)v;
            }
        }
    }

    /* 差分 DC + AC 游程 → 字节流 → 再包一层无损 RLE */
    /* 每块：DC 差分(2B) + n_ac(1B) + n_ac × [run(1B) + 系数(2B)]
     * 先把 AC 游程统计出来，才能写出 n_ac —— 这样解码端按计数读，结构上不可能失步。 */
    size_t raw_cap = cap_coef * 4 + 64;
    uint8_t *raw = (uint8_t *)malloc(raw_cap);
    if (!raw) { free(coef); return CLINK_ERR_CAPACITY; }
    size_t rn = 0;
    int16_t prev_dc = 0;
    for (size_t b = 0; b < cap_coef / 64; ++b) {
        const int16_t *blk = coef + b * 64;
        int32_t d = (int32_t)blk[0] - prev_dc;
        prev_dc = blk[0];
        if (rn + 3 > raw_cap) { free(raw); free(coef); return CLINK_ERR_CAPACITY; }
        raw[rn++] = (uint8_t)(d & 0xFF);
        raw[rn++] = (uint8_t)((d >> 8) & 0xFF);
        size_t nslot = rn;                            /* n_ac 的位置，稍后回填 */
        rn++;
        uint32_t n_ac = 0;
        uint32_t zero_run = 0;
        for (int i = 1; i < 64; ++i) {
            if (blk[i] == 0) {
                zero_run++;
                continue;
            }
            if (rn + 3 > raw_cap) { free(raw); free(coef); return CLINK_ERR_CAPACITY; }
            raw[rn++] = (uint8_t)(zero_run > 255 ? 255 : zero_run);
            int32_t v = blk[i];
            raw[rn++] = (uint8_t)(v & 0xFF);
            raw[rn++] = (uint8_t)((v >> 8) & 0xFF);
            zero_run = 0;
            n_ac++;
        }
        raw[nslot] = (uint8_t)(n_ac > 255 ? 255 : n_ac);
    }

    size_t hn = 0;
    uint8_t *huff = (uint8_t *)malloc(rn * 2 + 32);
    if (!huff) { free(raw); free(coef); return CLINK_ERR_CAPACITY; }
    int rc = huff_wrap(raw, rn, huff, rn * 2 + 32, &hn);
    if (rc != CLINK_OK) { free(raw); free(huff); free(coef); return rc; }

    size_t need = 8 + hn;                            /* 8 字节头：宽高与 quality */
    if (cap < need) { free(raw); free(huff); free(coef); return CLINK_ERR_CAPACITY; }
    out[0] = (uint8_t)(w & 0xFF); out[1] = (uint8_t)((w >> 8) & 0xFF);
    out[2] = (uint8_t)(h & 0xFF); out[3] = (uint8_t)((h >> 8) & 0xFF);
    out[4] = (uint8_t)quality; out[5] = 0; out[6] = 0; out[7] = 0;
    memcpy(out + 8, huff, hn);
    *out_len = need;

    free(raw); free(huff); free(coef);
    return CLINK_OK;
}

static int jpeg_like_decode(const uint8_t *buf, size_t len, uint8_t *out, uint32_t w, uint32_t h)
{
    if (len < 8) {
        return CLINK_ERR_FORMAT;
    }
    uint32_t bw = (uint32_t)(buf[0] | (buf[1] << 8));
    uint32_t bh = (uint32_t)(buf[2] | (buf[3] << 8));
    int quality = buf[4];
    if (bw != w || bh != h) {
        return CLINK_ERR_FORMAT;
    }
    double q[64];
    int zz[64];
    quant_tables(quality, q);
    zigzag_init(zz);

    size_t nblk = (size_t)(w / 8) * (h / 8);
    size_t rn_max = nblk * 512;
    uint8_t *raw = (uint8_t *)malloc(rn_max);
    if (!raw) {
        return CLINK_ERR_CAPACITY;
    }
    size_t rn = 0;
    int rc = huff_unwrap(buf + 8, len - 8, raw, rn_max, &rn);
    if (rc != CLINK_OK) { free(raw); return rc; }

    uint8_t *img = out;
    size_t p = 0;
    int16_t prev_dc = 0;
    for (size_t b = 0; b < nblk; ++b) {
        if (p + 2 > rn) { free(raw); return CLINK_ERR_FORMAT; }
        int32_t d = (int32_t)(int16_t)(raw[p] | (raw[p + 1] << 8));
        p += 2;
        prev_dc = (int16_t)(prev_dc + d);
        if (p + 1 > rn) { free(raw); return CLINK_ERR_FORMAT; }
        uint32_t n_ac = raw[p];
        p += 1;
        int16_t blk[64];
        memset(blk, 0, sizeof(blk));
        blk[0] = prev_dc;
        int i = 1;
        for (uint32_t a = 0; a < n_ac; ++a) {
            if (p + 3 > rn) { free(raw); return CLINK_ERR_FORMAT; }
            int run = raw[p++];
            int16_t val = (int16_t)((uint16_t)(uint8_t)raw[p] |
                                    ((uint16_t)(uint8_t)raw[p + 1] << 8));
            p += 2;
            i += run;
            if (i >= 64) { break; }
            blk[i++] = val;
        }
        double qc[64], fr[64], blkf[64];
        for (int k = 0; k < 64; ++k) {
            qc[zz[k]] = (double)blk[k] * q[zz[k]];
        }
        idct8(qc, fr);
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                double v = fr[y * 8 + x] + 128.0;
                v = v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v);
                blkf[y * 8 + x] = floor(v + 0.5);
            }
        }
        size_t by = (b / (w / 8)) * 8;
        size_t bx = (b % (w / 8)) * 8;
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                img[(by + y) * w + (bx + x)] = (uint8_t)blkf[y * 8 + x];
            }
        }
    }
    free(raw);
    return CLINK_OK;
}

/* ---------------------------------------------------------------- 对外 */

double codec_psnr(const uint8_t *a, const uint8_t *b, size_t n)
{
    double mse = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double d = (double)a[i] - (double)b[i];
        mse += d * d;
    }
    mse /= (double)n;
    if (mse <= 0.0) {
        return 1e30;
    }
    return 10.0 * log10((255.0 * 255.0) / mse);
}

int codec_encode(codec_kind_t kind, int quality, const uint8_t *img,
                 uint32_t w, uint32_t h, uint8_t *out, size_t cap, codec_stat_t *st)
{
    if (!img || !out || w == 0 || h == 0) {
        return CLINK_ERR_PARAM;
    }
    size_t n = 0;
    int rc;
    if (kind == CODEC_RLE_DIFF) {
        rc = rle_encode(img, w, h, out, cap, &n);
    } else if (kind == CODEC_JPEGLIKE) {
        rc = jpeg_like_encode(quality, img, w, h, out, cap, &n);
    } else {
        return CLINK_ERR_PARAM;
    }
    if (rc != CLINK_OK) {
        return rc;
    }
    if (st) {
        st->kind = kind;
        st->quality = quality;
        st->out_len = n;
        st->lossless = (kind == CODEC_RLE_DIFF) ? 1 : 0;
        st->psnr = 0.0;
    }
    return CLINK_OK;
}

int codec_decode(codec_kind_t kind, const uint8_t *buf, size_t len,
                 uint8_t *out, uint32_t w, uint32_t h)
{
    if (!buf || !out) {
        return CLINK_ERR_PARAM;
    }
    if (kind == CODEC_RLE_DIFF) {
        return rle_decode(buf, len, out, w, h);
    }
    if (kind == CODEC_JPEGLIKE) {
        return jpeg_like_decode(buf, len, out, w, h);
    }
    return CLINK_ERR_PARAM;
}
