/* clink.h —— camera-link-lab 公共接口
 *
 * 智能相机后端链路的**纯软件仿真**：把「图像 → 编码 → MIPI CSI-2 打包 → GVSP 分块传输
 * （带丢包与重传）→ 收端重组 → 解码 → 推理」整条链路在 PC 上跑通并可单元测试。
 *
 * ⚠️ 边界（README 同步声明）：
 *   · 这是**协议逻辑与时序仿真**，不是真实 sensor / PHY / 网卡的驱动实现；
 *   · MIPI CSI-2 的 ECC 按规范思路实现 6 位 Hamming（可纠 1 位错），但**未与真实 sensor 对拉验证**；
 *   · GVSP 只实现分块传输（leader / payload / trailer + 重传），**未实现 GenICam XML 描述与寄存器访问**；
 *   · 图像编码实现了无损差分 RLE 与 JPEG 风格有损编码（DCT + 量化 + zigzag + 简化 Huffman），
 *     **不是**任何芯片厂商的硬件编码器。
 */
#ifndef CLINK_H
#define CLINK_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------ 基础 */
#define CLINK_OK              0
#define CLINK_ERR_PARAM      -1
#define CLINK_ERR_ECC        -2
#define CLINK_ERR_CRC        -3
#define CLINK_ERR_SEQ        -4
#define CLINK_ERR_CAPACITY   -5
#define CLINK_ERR_FORMAT     -6
#define CLINK_ERR_TIMEOUT    -7

const char *clink_strerror(int code);

/* ------------------------------------------------------- MIPI CSI-2 包层 */
/* 数据类型的常用取值（取自 CSI-2 规范的数据类型表） */
#define CSI2_DT_FRAME_START   0x00
#define CSI2_DT_FRAME_END     0x01
#define CSI2_DT_LINE_START    0x02
#define CSI2_DT_LINE_END      0x03
#define CSI2_DT_RAW8          0x2B
#define CSI2_DT_RAW10         0x2C
#define CSI2_DT_RGB888        0x24
#define CSI2_DT_EMBEDDED      0x12
#define CSI2_DT_USER_DEF      0x30

#define CSI2_SHORT_PACKET_LEN 4u     /* DI(1) + WC(2) + ECC(1) */
#define CSI2_MAX_PAYLOAD      4096u  /* 单个长包允许的最大 payload（本仿真约束） */
#define CSI2_MAX_VC           4u

typedef struct {
    uint8_t  vc;        /* 虚拟通道 0~3 */
    uint8_t  dt;        /* 数据类型 */
    uint16_t wc;        /* 字长：长包为 payload 字节数；短包为数据本身 */
} csi2_di_t;

typedef struct {
    int      is_long;                 /* 1 = 长包（带 payload 与 CRC），0 = 短包 */
    csi2_di_t di;
    uint8_t  ecc;                     /* 包头 ECC（6 位有效） */
    uint8_t  payload[CSI2_MAX_PAYLOAD];
    uint16_t payload_len;
    uint16_t crc;                     /* 长包的 16 位 CRC */
} csi2_packet_t;

/* 把 DI + 字长打包成 24 位包头，并算出 ECC（返回 6 位，右对齐） */
uint32_t csi2_header24(uint8_t vc, uint8_t dt, uint16_t wc);
uint8_t  csi2_ecc6(uint32_t header24);
/* 用 ECC 校验还原包头；可纠正 1 位错。返回 CLINK_OK 或 CLINK_ERR_ECC。
 * corrected 非空时写回纠正后的 24 位包头。 */
int      csi2_ecc_check(uint32_t header24, uint8_t ecc, uint32_t *corrected);

/* 16 位 CRC（多项式 x^16+x^12+x^5+1 = 0x1021，初值 0xFFFF），CSI-2 长包尾用 */
uint16_t csi2_crc16(const uint8_t *data, size_t len);

/* 组包 / 解包 */
int csi2_pack_short(uint8_t vc, uint8_t dt, uint16_t data, uint8_t *out, size_t cap, size_t *out_len);
int csi2_pack_long(uint8_t vc, uint8_t dt, const uint8_t *payload, uint16_t len,
                   uint8_t *out, size_t cap, size_t *out_len);
/* 从字节流里解一个包；*consumed 写回本次消耗的字节数 */
int csi2_unpack(const uint8_t *buf, size_t len, csi2_packet_t *pkt, size_t *consumed);

/* --------------------------------------------------- CSI-2 帧重组器 */
#define CSI2_MAX_WIDTH  4096u
#define CSI2_MAX_HEIGHT 4096u

typedef struct {
    uint8_t  *frame;         /* 宽 × 高 的灰度帧（本仿真用 RAW8） */
    uint32_t  width;
    uint32_t  height;
    uint32_t  vc;
    int       in_frame;
    uint32_t  cur_line;
    uint32_t  cur_line_bytes;
    uint32_t  lines_done;
    uint32_t  long_packets;
    uint32_t  short_packets;
    uint32_t  ecc_errors;
    uint32_t  crc_errors;
    uint32_t  line_overflows;
} csi2_rx_t;

int  csi2_rx_init(csi2_rx_t *rx, uint32_t width, uint32_t height, uint32_t vc);
void csi2_rx_free(csi2_rx_t *rx);
/* 喂一个已解出的包；收满一帧返回 1，否则 0，出错返回负值 */
int  csi2_rx_feed(csi2_rx_t *rx, const csi2_packet_t *pkt);

/* --------------------------------------------------- GVSP 风格分块传输 */
#define GVSP_MAX_PAYLOAD   1440u
#define GVSP_MAX_RESENDS   4u

typedef enum {
    GVSP_LEADER  = 1,
    GVSP_PAYLOAD = 2,
    GVSP_TRAILER = 3
} gvsp_kind_t;

typedef struct {
    gvsp_kind_t kind;
    uint16_t    block_id;
    uint32_t    packet_id;      /* 同一 block 内从 0 递增；resend 包的 id 指向被请求的包 */
    uint32_t    payload_len;    /* leader/trailer 里记录整块字节数 */
    int32_t     timestamp_lo;
    uint16_t    data_len;
    uint8_t     data[GVSP_MAX_PAYLOAD];
} gvsp_packet_t;

/* 把一块数据拆成 leader + N×payload + trailer，返回写出的包数（失败返回负值） */
int gvsp_split(uint16_t block_id, const uint8_t *data, size_t len,
               gvsp_packet_t *out, int max_pkts, uint32_t *ts_out);
/* 把包序列化成线格式（同一套序列化给发送端与重传用） */
int gvsp_serialize(const gvsp_packet_t *pkt, uint8_t *out, size_t cap, size_t *out_len);
int gvsp_deserialize(const uint8_t *buf, size_t len, gvsp_packet_t *pkt, size_t *consumed);

/* ------------------------------------------- 确定性信道模拟（丢包/乱序/重复） */
typedef struct {
    uint32_t seed;
    uint32_t sent;
    uint32_t dropped;
    uint32_t duplicated;
    uint32_t reordered;
    uint32_t loss_permille;      /* 丢包率（千分比） */
    uint32_t dup_permille;       /* 重复率 */
    uint32_t reorder_permille;   /* 乱序率（与下一包交换） */
} gvsp_channel_t;

void gvsp_channel_init(gvsp_channel_t *ch, uint32_t seed,
                       uint32_t loss_permille, uint32_t dup_permille, uint32_t reorder_permille);
/* 判定第 n 个包是否被丢；返回 1 = 丢弃 */
int  gvsp_channel_drop(gvsp_channel_t *ch);
int  gvsp_channel_dup(gvsp_channel_t *ch);
int  gvsp_channel_reorder(gvsp_channel_t *ch);

/* -------------------------------------------------------- GVSP 接收端 */
typedef struct {
    uint8_t  *buf;              /* 重组缓冲区 */
    size_t    cap;
    size_t    expect_len;       /* leader 里声明的块长度 */
    size_t    got_len;
    uint16_t  block_id;
    uint32_t  next_packet_id;
    int       in_block;
    uint8_t  *seen;             /* 每包是否已收到（位图） */
    size_t    seen_cap;
    uint32_t  missing;          /* 当前缺口数 */
    uint32_t  resend_reqs;      /* 发出的重传请求次数 */
    uint32_t  packets_ok;
    uint32_t  packets_dup;
    uint32_t  packets_out_of_order;
    uint32_t  blocks_done;
} gvsp_rx_t;

int  gvsp_rx_init(gvsp_rx_t *rx, size_t cap);
void gvsp_rx_free(gvsp_rx_t *rx);
/* 收一个包：返回 1 = 整块收齐，0 = 继续收，负值 = 出错 */
int  gvsp_rx_feed(gvsp_rx_t *rx, const gvsp_packet_t *pkt);
/* 生成重传请求（把当前缺口区间写进 out，packet_id 填缺口起点，payload_len 填缺口长度） */
int  gvsp_rx_build_resend(gvsp_rx_t *rx, gvsp_packet_t *out);
/* 查询某个包是否已收到（发送端做选择性重传时要问这个） */
int  gvsp_rx_has_packet(const gvsp_rx_t *rx, uint32_t packet_id);
/* 当前缺口包数（按 leader 声明的块长实时算，不依赖 trailer 是否收到） */
uint32_t gvsp_rx_missing_count(const gvsp_rx_t *rx);

/* ------------------------------------------------------------- 图像编码 */
typedef enum { CODEC_RLE_DIFF = 0, CODEC_JPEGLIKE = 1 } codec_kind_t;

typedef struct {
    codec_kind_t kind;
    int          quality;        /* 1~100，仅 JPEG 风格使用 */
    size_t       out_len;        /* 编码后字节数 */
    double       psnr;           /* 解码还原质量（无损时为 inf） */
    int          lossless;
} codec_stat_t;

/* 编码到 out（cap 为 out 容量）；返回 CLINK_OK 或负值 */
int codec_encode(codec_kind_t kind, int quality, const uint8_t *img,
                 uint32_t w, uint32_t h, uint8_t *out, size_t cap, codec_stat_t *st);
/* 解码到 out（w×h 字节） */
int codec_decode(codec_kind_t kind, const uint8_t *buf, size_t len,
                 uint8_t *out, uint32_t w, uint32_t h);
/* PSNR（dB）；完全一致返回 1e30 */
double codec_psnr(const uint8_t *a, const uint8_t *b, size_t n);

#endif /* CLINK_H */
