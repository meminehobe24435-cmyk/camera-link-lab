/* gvsp.c —— GVSP 风格的分块传输：leader / payload / trailer、包序号、
 *           丢包检测与重传请求，外加一个**确定性**信道模拟器（丢包 / 重复 / 乱序）。
 *
 * 为什么用确定性信道而不是真实 UDP：
 *   真实网络的丢包随机且不可复现，单元测试没法断言。这里把丢包/乱序/重复做成由 seed 驱动的
 *   固定序列，**同一个 seed 必然得到同一组结果**，于是「丢 3 个包 → 发 1 次重传请求 → 补齐」
 *   这种时序可以被精确断言。
 *
 * ⚠️ 边界：只实现分块传输本身，**未实现 GenICam 的 XML 描述文件与寄存器读写（GVCP）**。
 */
#include "clink.h"

#include <stdlib.h>
#include <string.h>

/* 线格式（小端）：
 *   [0]      kind
 *   [1..2]   block_id
 *   [3..6]   packet_id
 *   [7..10]  payload_len
 *   [11..14] timestamp_lo
 *   [15..16] data_len
 *   [17..]   data
 */
#define GVSP_HDR 17u

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu); p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int gvsp_split(uint16_t block_id, const uint8_t *data, size_t len,
               gvsp_packet_t *out, int max_pkts, uint32_t *ts_out)
{
    if (!data || !out || len == 0 || max_pkts < 2) {
        return CLINK_ERR_PARAM;
    }
    size_t n_payload = (len + GVSP_MAX_PAYLOAD - 1) / GVSP_MAX_PAYLOAD;
    if ((int)(n_payload + 2) > max_pkts) {
        return CLINK_ERR_CAPACITY;
    }
    int k = 0;
    memset(&out[k], 0, sizeof(out[k]));
    out[k].kind = GVSP_LEADER;
    out[k].block_id = block_id;
    out[k].packet_id = 0;
    out[k].payload_len = (uint32_t)len;
    out[k].timestamp_lo = (int32_t)(0x5A5A0000u + block_id);
    k++;

    for (size_t i = 0; i < n_payload; ++i) {
        size_t off = i * GVSP_MAX_PAYLOAD;
        size_t n = len - off;
        if (n > GVSP_MAX_PAYLOAD) {
            n = GVSP_MAX_PAYLOAD;
        }
        memset(&out[k], 0, sizeof(out[k]));
        out[k].kind = GVSP_PAYLOAD;
        out[k].block_id = block_id;
        out[k].packet_id = (uint32_t)(i + 1);
        out[k].data_len = (uint16_t)n;
        memcpy(out[k].data, data + off, n);
        k++;
    }

    memset(&out[k], 0, sizeof(out[k]));
    out[k].kind = GVSP_TRAILER;
    out[k].block_id = block_id;
    out[k].packet_id = (uint32_t)(n_payload + 1);
    out[k].payload_len = (uint32_t)len;
    k++;

    if (ts_out) {
        *ts_out = 0x5A5A0000u + block_id;
    }
    return k;
}

int gvsp_serialize(const gvsp_packet_t *pkt, uint8_t *out, size_t cap, size_t *out_len)
{
    if (!pkt || !out) {
        return CLINK_ERR_PARAM;
    }
    size_t need = GVSP_HDR + pkt->data_len;
    if (cap < need) {
        return CLINK_ERR_CAPACITY;
    }
    out[0] = (uint8_t)pkt->kind;
    put_u16(out + 1, pkt->block_id);
    put_u32(out + 3, pkt->packet_id);
    put_u32(out + 7, pkt->payload_len);
    put_u32(out + 11, (uint32_t)pkt->timestamp_lo);
    put_u16(out + 15, pkt->data_len);
    if (pkt->data_len) {
        memcpy(out + GVSP_HDR, pkt->data, pkt->data_len);
    }
    if (out_len) {
        *out_len = need;
    }
    return CLINK_OK;
}

int gvsp_deserialize(const uint8_t *buf, size_t len, gvsp_packet_t *pkt, size_t *consumed)
{
    if (!buf || !pkt || len < GVSP_HDR) {
        return CLINK_ERR_PARAM;
    }
    memset(pkt, 0, sizeof(*pkt));
    pkt->kind = (gvsp_kind_t)buf[0];
    pkt->block_id = get_u16(buf + 1);
    pkt->packet_id = get_u32(buf + 3);
    pkt->payload_len = get_u32(buf + 7);
    pkt->timestamp_lo = (int32_t)get_u32(buf + 11);
    pkt->data_len = get_u16(buf + 15);
    if (pkt->data_len > GVSP_MAX_PAYLOAD) {
        return CLINK_ERR_FORMAT;
    }
    if (len < GVSP_HDR + pkt->data_len) {
        return CLINK_ERR_PARAM;
    }
    if (pkt->data_len) {
        memcpy(pkt->data, buf + GVSP_HDR, pkt->data_len);
    }
    if (consumed) {
        *consumed = GVSP_HDR + pkt->data_len;
    }
    return CLINK_OK;
}

/* ------------------------------------------------------ 确定性信道模拟 */

static uint32_t xorshift(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x ? x : 0x12345678u;
    return *s;
}

static int hit(uint32_t *s, uint32_t permille)
{
    if (permille == 0) {
        return 0;
    }
    return (xorshift(s) % 1000u) < permille;
}

void gvsp_channel_init(gvsp_channel_t *ch, uint32_t seed,
                       uint32_t loss_permille, uint32_t dup_permille, uint32_t reorder_permille)
{
    if (!ch) {
        return;
    }
    memset(ch, 0, sizeof(*ch));
    ch->seed = seed ? seed : 0xDEADBEEFu;
    ch->loss_permille = loss_permille;
    ch->dup_permille = dup_permille;
    ch->reorder_permille = reorder_permille;
}

int gvsp_channel_drop(gvsp_channel_t *ch)
{
    if (!ch) {
        return 0;
    }
    ch->sent++;
    if (hit(&ch->seed, ch->loss_permille)) {
        ch->dropped++;
        return 1;
    }
    return 0;
}

int gvsp_channel_dup(gvsp_channel_t *ch)
{
    if (!ch) {
        return 0;
    }
    if (hit(&ch->seed, ch->dup_permille)) {
        ch->duplicated++;
        return 1;
    }
    return 0;
}

int gvsp_channel_reorder(gvsp_channel_t *ch)
{
    if (!ch) {
        return 0;
    }
    if (hit(&ch->seed, ch->reorder_permille)) {
        ch->reordered++;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------ GVSP 接收 */

static void seen_set(gvsp_rx_t *rx, uint32_t id)
{
    if (id < rx->seen_cap) {
        rx->seen[id] = 1;
    }
}

static int seen_get(const gvsp_rx_t *rx, uint32_t id)
{
    return id < rx->seen_cap ? rx->seen[id] : 0;
}

int gvsp_rx_init(gvsp_rx_t *rx, size_t cap)
{
    if (!rx || cap == 0) {
        return CLINK_ERR_PARAM;
    }
    memset(rx, 0, sizeof(*rx));
    rx->buf = (uint8_t *)calloc(cap, 1);
    rx->seen_cap = cap / 64u + 8u;
    rx->seen = (uint8_t *)calloc(rx->seen_cap, 1);
    rx->cap = cap;
    if (!rx->buf || !rx->seen) {
        gvsp_rx_free(rx);
        return CLINK_ERR_CAPACITY;
    }
    return CLINK_OK;
}

void gvsp_rx_free(gvsp_rx_t *rx)
{
    if (!rx) {
        return;
    }
    free(rx->buf);  rx->buf = NULL;
    free(rx->seen); rx->seen = NULL;
}

int gvsp_rx_feed(gvsp_rx_t *rx, const gvsp_packet_t *pkt)
{
    if (!rx || !pkt) {
        return CLINK_ERR_PARAM;
    }
    switch (pkt->kind) {
    case GVSP_LEADER:
        if (pkt->payload_len > rx->cap) {
            return CLINK_ERR_CAPACITY;
        }
        rx->block_id = pkt->block_id;
        rx->expect_len = pkt->payload_len;
        rx->got_len = 0;
        rx->next_packet_id = 1;
        rx->in_block = 1;
        memset(rx->seen, 0, rx->seen_cap);
        seen_set(rx, 0);
        return 0;

    case GVSP_PAYLOAD: {
        if (!rx->in_block || pkt->block_id != rx->block_id) {
            return 0;                       /* 上一块已收完或块号不符，忽略 */
        }
        if (seen_get(rx, pkt->packet_id)) {
            rx->packets_dup++;              /* 重复包：不重复写入，只计数 */
            return 0;
        }
        if (pkt->packet_id != rx->next_packet_id) {
            /* 乱序到达（不是紧接着的那一包），按偏移写入并记录 */
            rx->packets_out_of_order++;
        }
        size_t off = (size_t)(pkt->packet_id - 1) * GVSP_MAX_PAYLOAD;
        if (off + pkt->data_len > rx->cap) {
            return CLINK_ERR_CAPACITY;
        }
        memcpy(rx->buf + off, pkt->data, pkt->data_len);
        seen_set(rx, pkt->packet_id);
        rx->packets_ok++;
        while (seen_get(rx, rx->next_packet_id)) {
            rx->next_packet_id++;
        }
        return 0;
    }

    case GVSP_TRAILER: {
        if (!rx->in_block || pkt->block_id != rx->block_id) {
            return 0;
        }
        size_t n_payload = (rx->expect_len + GVSP_MAX_PAYLOAD - 1) / GVSP_MAX_PAYLOAD;
        /* 只有前 n_payload 个 payload 包都到齐，才算整块收齐 */
        rx->got_len = 0;
        uint32_t miss = 0;
        for (uint32_t i = 1; i <= n_payload; ++i) {
            if (seen_get(rx, i)) {
                size_t off = (size_t)(i - 1) * GVSP_MAX_PAYLOAD;
                size_t left = rx->expect_len - off;
                rx->got_len += left > GVSP_MAX_PAYLOAD ? GVSP_MAX_PAYLOAD : left;
            } else {
                miss++;
            }
        }
        rx->missing = miss;
        if (miss == 0) {
            rx->in_block = 0;
            rx->blocks_done++;
            return 1;
        }
        return 0;                            /* 有缺口 → 等重传 */
    }

    default:
        return CLINK_ERR_FORMAT;
    }
}

int gvsp_rx_has_packet(const gvsp_rx_t *rx, uint32_t packet_id)
{
    return rx ? seen_get(rx, packet_id) : 0;
}

uint32_t gvsp_rx_missing_count(const gvsp_rx_t *rx)
{
    if (!rx || !rx->in_block || rx->expect_len == 0) {
        return 0;
    }
    size_t n_payload = (rx->expect_len + GVSP_MAX_PAYLOAD - 1) / GVSP_MAX_PAYLOAD;
    uint32_t miss = 0;
    for (uint32_t i = 1; i <= n_payload; ++i) {
        if (!seen_get(rx, i)) {
            miss++;
        }
    }
    return miss;
}

int gvsp_rx_build_resend(gvsp_rx_t *rx, gvsp_packet_t *out)
{
    if (!rx || !out || !rx->in_block) {
        return CLINK_ERR_PARAM;
    }
    /* 缺口要**实时**算：不能依赖 trailer 是否收到 —— trailer 自己也可能被丢掉，
     * 那样 missing 会一直停在 0，发送端就永远不知道该补什么。 */
    size_t n_payload = (rx->expect_len + GVSP_MAX_PAYLOAD - 1) / GVSP_MAX_PAYLOAD;
    uint32_t start = 0, count = 0;
    for (uint32_t i = 1; i <= n_payload; ++i) {
        if (!seen_get(rx, i)) {
            if (start == 0) {
                start = i;
            }
            count++;
        } else if (start != 0) {
            break;                            /* 只请求第一段连续缺口 */
        }
    }
    rx->missing = gvsp_rx_missing_count(rx);
    if (count == 0) {
        return CLINK_ERR_PARAM;               /* 没有缺口 */
    }
    memset(out, 0, sizeof(*out));
    out->kind = GVSP_PAYLOAD;                 /* 复用 payload 包承载重传请求 */
    out->block_id = rx->block_id;
    out->packet_id = start;
    out->payload_len = count;                 /* payload_len 借用来表示请求的包数 */
    out->data_len = 0;
    rx->resend_reqs++;
    return CLINK_OK;
}
