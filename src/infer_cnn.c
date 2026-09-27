/* infer_cnn.c —— 嵌入式侧的前向推理（纯 C99、零第三方依赖）
 *
 * 把 ONNX / PyTorch 训练出来的小 CNN 用 C 手写一遍前向：
 *   Conv2d(1→4,3×3,pad=1) → ReLU → MaxPool2d(2)
 *   Conv2d(4→8,3×3,pad=1) → ReLU → MaxPool2d(2)
 *   Flatten → Linear(128→1) → Sigmoid
 *
 * 用途：验证「模型部署到没有 Python 的目标平台」时，数值能不能和训练框架对上。
 * 权重来自 weights/cnn_weights.h（由 tools/export_onnx.py 生成），测试输入也一并内嵌，
 * 因此本程序可以在任意只有 C 编译器的机器上复现 PyTorch 的结果。
 *
 * ⚠️ 边界：这是一个**数值一致性验证程序**，不是高性能推理引擎 ——
 *    没有做算子融合、量化、SIMD 或内存复用；也不代表任何厂商推理框架的实测性能。
 */
#include "cnn_weights.h"

#include <math.h>
#include <stdio.h>

#define C1 CNN_C1
#define C2 CNN_C2
#define W  CNN_IN_W
#define H  CNN_IN_H
#define P1 (W / 2)          /* 第一次池化后 8×8 */
#define P2 (W / 4)          /* 第二次池化后 4×4 */

static float relu(float v) { return v > 0.0f ? v : 0.0f; }

/* 3×3、stride 1、padding 1 的卷积。in: (cin, H, W)，w: (cout, cin, 3, 3) */
static void conv3x3(const float *in, int cin, int h, int w,
                    const float *weight, const float *bias, int cout, float *out)
{
    for (int co = 0; co < cout; ++co) {
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float acc = bias[co];
                for (int ci = 0; ci < cin; ++ci) {
                    const float *src = in + (size_t)ci * h * w;
                    const float *k = weight + ((size_t)co * cin + ci) * 9;
                    for (int dy = -1; dy <= 1; ++dy) {
                        int sy = y + dy;
                        if (sy < 0 || sy >= h) {
                            continue;
                        }
                        for (int dx = -1; dx <= 1; ++dx) {
                            int sx = x + dx;
                            if (sx < 0 || sx >= w) {
                                continue;
                            }
                            acc += src[(size_t)sy * w + sx] * k[(dy + 1) * 3 + (dx + 1)];
                        }
                    }
                }
                out[((size_t)co * h + y) * w + x] = relu(acc);
            }
        }
    }
}

/* 2×2、stride 2 的最大池化 */
static void maxpool2(const float *in, int c, int h, int w, float *out)
{
    int oh = h / 2, ow = w / 2;
    for (int ch = 0; ch < c; ++ch) {
        const float *src = in + (size_t)ch * h * w;
        float *dst = out + (size_t)ch * oh * ow;
        for (int y = 0; y < oh; ++y) {
            for (int x = 0; x < ow; ++x) {
                float m = src[(size_t)(2 * y) * w + 2 * x];
                float v;
                v = src[(size_t)(2 * y) * w + 2 * x + 1];       if (v > m) m = v;
                v = src[(size_t)(2 * y + 1) * w + 2 * x];       if (v > m) m = v;
                v = src[(size_t)(2 * y + 1) * w + 2 * x + 1];   if (v > m) m = v;
                dst[(size_t)y * ow + x] = m;
            }
        }
    }
}

int main(void)
{
    static float a1[C1 * H * W];
    static float p1[C1 * P1 * P1];
    static float a2[C2 * P1 * P1];
    static float p2[C2 * P2 * P2];

    conv3x3(cnn_test_input, 1, H, W, cnn_conv1_w, cnn_conv1_b, C1, a1);
    maxpool2(a1, C1, H, W, p1);
    conv3x3(p1, C1, P1, P1, cnn_conv2_w, cnn_conv2_b, C2, a2);
    maxpool2(a2, C2, P1, P1, p2);

    int n = C2 * P2 * P2;
    float logit = cnn_fc_b[0];
    for (int i = 0; i < n; ++i) {
        logit += cnn_fc_w[i] * p2[i];
    }
    float prob = 1.0f / (1.0f + expf(-logit));

    printf("CNN_OUT %.8f\n", (double)prob);
    printf("CNN_REF %.8f\n", (double)CNN_REF_OUTPUT);
    double diff = fabs((double)prob - (double)CNN_REF_OUTPUT);
    printf("CNN_DIFF %.3e\n", diff);
    return diff < 1e-4 ? 0 : 1;
}
