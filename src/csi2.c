/* csi2.c —— MIPI CSI-2 包层：包头 ECC、长包 CRC、组包/解包、帧与行重组
 *
 * 规范要点（按 CSI-2 的公开描述实现）：
 *   · 包头 24 位 = DI(8) | WC(16)，其中 DI = VC(2) | DT(6)
 *   · 包头后跟 1 字节 ECC（6 位有效），可纠正包头中的 1 位错误、检出 2 位错误
 *   · 长包 = 包头(4B) + payload(WC 字节) + CRC(2B)，CRC 多项式 0x1021、初值 0xFFFF
 *   · 短包 = 包头(4B)，WC 字段直接承载数据（帧起始/行起始等同步码）
 *
 * ⚠️ 本文件的 ECC 是「按 CSI-2 思路实现的 6 位 Hamming 码」：覆盖 24 位、能纠 1 位错。
 *    它**没有**与真实 sensor 对拉验证过，不声称逐位等同于某颗芯片的输出。
 */
#include "clink.h"

#include <stdlib.h>
#include <string.h>

const char *clink_strerror(int code)
{
    switch (code) {
    case CLINK_OK:            return "ok";
    case CLINK_ERR_PARAM:     return "invalid argument";
    case CLINK_ERR_ECC:       return "packet header ECC error";
    case CLINK_ERR_CRC:       return "packet payload CRC error";
    case CLINK_ERR_SEQ:       return "sequence error";
    case CLINK_ERR_CAPACITY:  return "buffer too small";
    case CLINK_ERR_FORMAT:    return "bad packet format";
    case CLINK_ERR_TIMEOUT:   return "timeout";
    default:                  return "unknown";
    }
}

/* ------------------------------------------------------------ 包头与 ECC */

uint32_t csi2_header24(uint8_t vc, uint8_t dt, uint16_t wc)
{
    uint32_t di = (uint32_t)(((vc & 0x3u) << 6) | (dt & 0x3Fu));
    return (di << 16) | (uint32_t)wc;
}

uint8_t csi2_ecc6(uint32_t header24)
{
    /* SECDED：低 5 位是 24 个数据位的伴随式（每位分配唯一非零值 1..24），
     * 第 6 位（bit5）是**整体偶校验**。
     *   · 伴随式非零 + 整体校验错 → 单比特错，可纠正；
     *   · 伴随式非零 + 整体校验对 → 双比特错，只报错不纠（避免误纠成第三位）；
     *   · 伴随式为零 + 整体校验错 → 错在 ECC 自身，报错。
     * 上一版只用 5 位伴随式、没有整体校验位，于是「双比特错」会被误纠成另一个单比特错 ——
     * 这条同样是被 test_ecc 抓出来的。 */
    uint8_t ecc = 0;
    for (int p = 0; p < 5; ++p) {
        int ones = 0;
        for (int b = 0; b < 24; ++b) {
            int syndrome = b + 1;                    /* 1..24，非零且唯一，占 5 位 */
            if ((syndrome >> p) & 1) {
                ones += (int)((header24 >> (23 - b)) & 1u);
            }
        }
        if (ones & 1) {
            ecc |= (uint8_t)(1u << p);
        }
    }
    int total = 0;
    for (int b = 0; b < 24; ++b) {
        total += (int)((header24 >> (23 - b)) & 1u);
    }
    for (int p = 0; p < 5; ++p) {
        total += (ecc >> p) & 1;
    }
    if (total & 1) {
        ecc |= 0x20u;                                /* 整体偶校验位 */
    }
    return (uint8_t)(ecc & 0x3Fu);
}

int csi2_ecc_check(uint32_t header24, uint8_t ecc, uint32_t *corrected)
{
    uint8_t got = (uint8_t)(ecc & 0x3Fu);
    uint8_t c_got = (uint8_t)(got & 0x1Fu);              /* 接收到的 5 位校验位 */
    uint8_t c_want = (uint8_t)(csi2_ecc6(header24) & 0x1Fu);  /* 由接收数据重算的校验位 */
    uint8_t s = (uint8_t)(c_want ^ c_got);               /* 伴随式 */

    /* 真正的整体校验：把**接收到的**数据位 + 校验位 + 整体校验位一起数奇偶。
     * ⚠️ 这里不能用「重算值与原值的第 6 位之差」——那个量等于 1 ^ parity(伴随式列)，
     *    会让一半的单比特错被误判成"双比特错"（实测只能纠正 12/24）。 */
    int total = 0;
    for (int b = 0; b < 24; ++b) {
        total += (int)((header24 >> b) & 1u);
    }
    for (int p = 0; p < 5; ++p) {
        total += (c_got >> p) & 1;
    }
    total += (got >> 5) & 1u;
    int parity_err = total & 1;

    if (s == 0) {
        if (parity_err == 0) {
            if (corrected) {
                *corrected = header24;
            }
            return CLINK_OK;                             /* 无错 */
        }
        return CLINK_ERR_ECC;                            /* 错在整体校验位自身 */
    }
    if (!parity_err) {
        return CLINK_ERR_ECC;                            /* 双比特错：只检出，不纠 */
    }
    if (s <= 24u) {
        int b = (int)s - 1;                              /* 伴随式直接指出出错位（MSB 起算） */
        uint32_t fixed = header24 ^ (1u << (23 - b));
        if (corrected) {
            *corrected = fixed;
        }
        return CLINK_OK;                                 /* 单比特错已纠正 */
    }
    return CLINK_ERR_ECC;
}

uint16_t csi2_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---------------------------------------------------------------- 组包 */

int csi2_pack_short(uint8_t vc, uint8_t dt, uint16_t data,
                    uint8_t *out, size_t cap, size_t *out_len)
{
    if (!out || cap < CSI2_SHORT_PACKET_LEN) {
        return CLINK_ERR_CAPACITY;
    }
    uint32_t h = csi2_header24(vc, dt, data);
    out[0] = (uint8_t)((h >> 16) & 0xFFu);
    out[1] = (uint8_t)((h >> 8) & 0xFFu);
    out[2] = (uint8_t)(h & 0xFFu);
    out[3] = csi2_ecc6(h);
    if (out_len) {
        *out_len = CSI2_SHORT_PACKET_LEN;
    }
    return CLINK_OK;
}

int csi2_pack_long(uint8_t vc, uint8_t dt, const uint8_t *payload, uint16_t len,
                   uint8_t *out, size_t cap, size_t *out_len)
{
    if (!out || !payload || len == 0 || len > CSI2_MAX_PAYLOAD) {
        return CLINK_ERR_PARAM;
    }
    size_t need = CSI2_SHORT_PACKET_LEN + (size_t)len + 2u;
    if (cap < need) {
        return CLINK_ERR_CAPACITY;
    }
    uint32_t h = csi2_header24(vc, dt, len);
    out[0] = (uint8_t)((h >> 16) & 0xFFu);
    out[1] = (uint8_t)((h >> 8) & 0xFFu);
    out[2] = (uint8_t)(h & 0xFFu);
    out[3] = csi2_ecc6(h);
    memcpy(out + 4, payload, len);
    uint16_t crc = csi2_crc16(payload, len);
    out[4 + len] = (uint8_t)((crc >> 8) & 0xFFu);
    out[5 + len] = (uint8_t)(crc & 0xFFu);
    if (out_len) {
        *out_len = need;
    }
    return CLINK_OK;
}

int csi2_unpack(const uint8_t *buf, size_t len, csi2_packet_t *pkt, size_t *consumed)
{
    if (!buf || !pkt || len < CSI2_SHORT_PACKET_LEN) {
        return CLINK_ERR_PARAM;
    }
    uint32_t h = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    uint8_t  ecc = buf[3];
    uint32_t fixed = h;
    int rc = csi2_ecc_check(h, ecc, &fixed);
    if (rc != CLINK_OK) {
        return CLINK_ERR_ECC;
    }
    memset(pkt, 0, sizeof(*pkt));
    pkt->ecc = (uint8_t)(ecc & 0x3Fu);
    pkt->di.vc = (uint8_t)((fixed >> 22) & 0x3u);
    pkt->di.dt = (uint8_t)((fixed >> 16) & 0x3Fu);
    pkt->di.wc = (uint16_t)(fixed & 0xFFFFu);

    /* 长包判定：本仿真里 0x2B/0x2C/0x24/0x12/0x30 走长包格式 */
    switch (pkt->di.dt) {
    case CSI2_DT_RAW8:
    case CSI2_DT_RAW10:
    case CSI2_DT_RGB888:
    case CSI2_DT_EMBEDDED:
    case CSI2_DT_USER_DEF:
        pkt->is_long = 1;
        break;
    default:
        pkt->is_long = 0;
        break;
    }

    if (!pkt->is_long) {
        if (consumed) {
            *consumed = CSI2_SHORT_PACKET_LEN;
        }
        return CLINK_OK;
    }

    uint16_t wc = pkt->di.wc;
    if (wc == 0 || wc > CSI2_MAX_PAYLOAD) {
        return CLINK_ERR_FORMAT;
    }
    if (len < CSI2_SHORT_PACKET_LEN + (size_t)wc + 2u) {
        return CLINK_ERR_PARAM;      /* 包还没收全 */
    }
    const uint8_t *pl = buf + CSI2_SHORT_PACKET_LEN;
    uint16_t got = (uint16_t)(((uint16_t)pl[wc] << 8) | pl[wc + 1]);
    uint16_t want = csi2_crc16(pl, wc);
    if (got != want) {
        return CLINK_ERR_CRC;
    }
    memcpy(pkt->payload, pl, wc);
    pkt->payload_len = wc;
    pkt->crc = got;
    if (consumed) {
        *consumed = CSI2_SHORT_PACKET_LEN + (size_t)wc + 2u;
    }
    return CLINK_OK;
}

/* ------------------------------------------------------------ 帧重组器 */

int csi2_rx_init(csi2_rx_t *rx, uint32_t width, uint32_t height, uint32_t vc)
{
    if (!rx || width == 0 || height == 0 ||
        width > CSI2_MAX_WIDTH || height > CSI2_MAX_HEIGHT) {
        return CLINK_ERR_PARAM;
    }
    memset(rx, 0, sizeof(*rx));
    rx->width = width;
    rx->height = height;
    rx->vc = vc;
    rx->frame = (uint8_t *)calloc((size_t)width * height, 1);
    if (!rx->frame) {
        return CLINK_ERR_CAPACITY;
    }
    return CLINK_OK;
}

void csi2_rx_free(csi2_rx_t *rx)
{
    if (rx && rx->frame) {
        free(rx->frame);
        rx->frame = NULL;
    }
}

int csi2_rx_feed(csi2_rx_t *rx, const csi2_packet_t *pkt)
{
    if (!rx || !pkt) {
        return CLINK_ERR_PARAM;
    }
    if (pkt->di.vc != rx->vc) {
        return 0;                       /* 不是本虚拟通道，忽略 */
    }
    if (!pkt->is_long) {
        rx->short_packets++;
        switch (pkt->di.dt) {
        case CSI2_DT_FRAME_START:
            rx->in_frame = 1;
            rx->lines_done = 0;
            rx->cur_line = 0;
            rx->cur_line_bytes = 0;
            return 0;
        case CSI2_DT_FRAME_END:
            rx->in_frame = 0;
            return 1;                   /* 一帧结束 */
        case CSI2_DT_LINE_START:
            if (!rx->in_frame) {
                return 0;
            }
            if (pkt->di.wc >= rx->height) {
                return CLINK_ERR_FORMAT;
            }
            rx->cur_line = pkt->di.wc;
            rx->cur_line_bytes = 0;
            return 0;
        case CSI2_DT_LINE_END:
            if (!rx->in_frame) {
                return 0;
            }
            if (rx->cur_line_bytes != rx->width) {
                rx->line_overflows++;   /* 行内字节数不足 → 记一次异常 */
            }
            rx->lines_done++;
            return 0;
        default:
            return 0;
        }
    }

    rx->long_packets++;
    if (!rx->in_frame) {
        return 0;
    }
    if (rx->cur_line >= rx->height) {
        rx->line_overflows++;
        return 0;
    }
    size_t off = (size_t)rx->cur_line * rx->width + rx->cur_line_bytes;
    size_t room = (size_t)rx->width - rx->cur_line_bytes;
    size_t n = pkt->payload_len < room ? pkt->payload_len : room;
    if (n > 0) {
        memcpy(rx->frame + off, pkt->payload, n);
        rx->cur_line_bytes += (uint32_t)n;
    }
    if (pkt->payload_len > room) {
        rx->line_overflows++;
    }
    return 0;
}
