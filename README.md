# camera-link-lab

> 智能相机后端链路的**纯软件仿真**：把「图像 → 编码 → **MIPI CSI-2** 打包 →
> **GVSP 风格分块传输**（丢包 / 重复 / 乱序 + 选择性重传）→ 收端重组 → 解码 →
> **ONNX / C 前向推理**」整条链路在 PC 上跑通，并且每一步都能单元测试。

[![ci](https://github.com/meminehobe24435-cmyk/camera-link-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/meminehobe24435-cmyk/camera-link-lab/actions/workflows/ci.yml)
![language](https://img.shields.io/badge/language-C99-blue)
![deps](https://img.shields.io/badge/core%20dependencies-none-green)
![tests](https://img.shields.io/badge/tests-60%20passed-brightgreen)

---

## 1. 先说边界（重要）

- 这是**协议逻辑与时序仿真**，**不是**真实 sensor / PHY / 网卡的驱动实现；
- MIPI CSI-2 的 ECC 按规范思路实现 **6 位 SECDED**（5 位伴随式 + 1 位整体奇偶，可纠 1 位错、检 2 位错），
  但**没有与真实 sensor 对拉验证过**；
- GVSP 只实现**分块传输**（leader / payload / trailer + 重传），
  **未实现 GenICam 的 XML 描述文件与寄存器读写（GVCP）**；
- 图像编码实现了无损差分 RLE 与 JPEG 风格的**变换编码链路**（DCT + 量化 + Z 字形 + DC 差分 + AC 游程），
  **不是标准 JPEG**：无色彩空间转换与色度下采样，熵层用的是「每块长度前缀」而非 Huffman 标准表，
  **输出不兼容任何 JPEG 解码器**；
- 推理部分是**数值一致性验证**，不是高性能推理引擎：没有算子融合 / 量化 / SIMD，
  也不代表任何厂商推理框架的实测性能；训练用的是**程序生成的合成数据**。

---

## 2. 实测结果（run `20260927`，256×128 灰度，丢包率 15%）

### 单元测试

```
==== 结果：60 passed, 0 failed ====
```

覆盖：ECC 的 24 位单比特遍历与双比特检出 · CRC-16/CCITT 标准测试向量（`"123456789"` → `0x29B1`）·
短包 / 长包往返与 CRC 破坏检测 · 容量边界 · 虚拟通道过滤 · 帧与行重组 ·
GVSP 拆分包数与序号连续 · 线格式往返 · 无丢包一次收齐 · 高丢包不重传必超时 ·
高丢包有重传必补齐且内容完全一致 · 重复包不重复写入 · RLE 严格可逆 · 纯色图压缩比 ·
JPEG 风格质量单调性 · 完全一致时 PSNR 为无穷。

### 端到端链路

| 环节 | 实测 |
|---|---|
| 无损差分 RLE | 32769 字节（噪声图上触发**原样回退**，压缩比 1.00x），**逐字节可逆** |
| JPEG 风格（q=75） | 21900 字节，压缩比 1.50x，**PSNR 32.49 dB** |
| MIPI CSI-2 | 386 个包（短包 258 / 长包 128），线字节 34568，**帧重组逐字节一致**，行异常 0 |
| GVSP 传输（15% 丢包） | 19 个包中丢 1，**发 1 次重传请求、2 轮补齐**，块重组逐字节一致 |
| 端到端 | 编码 → 传输 → 重组 → 解码后 **PSNR 32.49 dB** |

多个随机种子的重传收敛情况（丢包率固定 15%，用确定性信道保证可复现）：

| seed | 丢包 | 乱序 | 重传请求 | 轮次 | 结果 |
|---|---:|---:|---:|---:|---|
| 20260927 | 1 | 0 | 1 | 2 | 收齐 |
| 11 | 3 | 1 | 1 | 2 | 收齐 |
| 7 | 3 | 0 | 1 | 2 | 收齐 |
| 3 | 6 | 0 | 2 | 3 | 收齐 |

### 推理链路的数值一致性

```
PyTorch      = 0.54245895
ONNX Runtime = 0.54245895
C 前向       = 0.54245889   与 PyTorch 差值 6.147e-08
结论：三方一致（容差 1e-04）
```

同一份模型走完「训练框架 → ONNX → **纯 C 手写前向**」三站，输出必须一致 ——
这是「模型能不能部署到没有 Python 的目标平台」的最小可验证版本。

---

## 3. 目录结构

```text
include/clink.h         公共接口与错误码
src/csi2.c              MIPI CSI-2：SECDED ECC、CRC-16、组包/解包、帧与行重组
src/gvsp.c              GVSP 风格分块传输 + 确定性丢包信道 + 收端缺口统计
src/codec.c             无损差分 RLE（含原样回退）+ JPEG 风格变换编码
src/pipeline.c          端到端链路演示，输出 out/metrics.csv
src/infer_cnn.c         纯 C 前向推理（用 weights/cnn_weights.h），与 ONNX 对数值
test/test_link.c        60 项单元测试（自带断言宏，零第三方依赖）
tools/export_onnx.py    PyTorch 训练小 CNN → 导出 ONNX + 生成 C 权重头
tools/onnx_infer.py     ONNX Runtime 推理，并编译运行 C 侧做三方对比
weights/                model.onnx / cnn_weights.h / test_input.npy / meta.json
out/metrics.csv         端到端链路指标
```

---

## 4. 怎么跑

核心部分**只需要一个 C 编译器**（零第三方依赖）：

```bash
make test      # 60 项单元测试
make sim       # 端到端链路仿真 → out/metrics.csv
make infer     # 纯 C 前向推理（用已生成的权重头）
```

推理链路的完整复现需要 Python 侧的 torch / onnx / onnxruntime：

```bash
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements-dev.txt
make onnx      # 训练 → 导出 ONNX → 生成 C 权重头 → 三方数值对比
```

Windows 上没有 `make` 时直接用 gcc：

```bat
gcc -std=c99 -O2 -Wall -Wextra -Wpedantic -Werror -Iinclude test\test_link.c src\csi2.c src\gvsp.c src\codec.c -o build\clink_test.exe -lm
build\clink_test.exe
```

> **踩坑记录**：gcc（MSYS2）对**含中文的绝对路径**会在链接阶段报
> `cannot open output file ...: No such file or directory` —— 一律用相对路径或先 `cd` 进项目目录。

---

## 5. 开发过程中被测试抓出来的真问题

| # | 现象 | 定位方式 | 修复 |
|---|---|---|---|
| 1 | **ECC 只能纠正 12/24 个单比特错** | 写了一条「24 位逐个翻转都要能纠回来」的遍历用例 | 原分组掩码不是合法 Hamming 结构（多位共享同一伴随式）→ 改成「每位分配唯一伴随式 1..24」的标准构造 |
| 2 | **双比特错被误纠成第三个单比特错** | 「2 位错必须报错而不是误纠」用例 | 原实现没有整体校验位，Hamming 码本身检不出全部双错 → 补 **SECDED**：5 位伴随式 + 1 位整体奇偶 |
| 3 | **PSNR 在 quality≥65 断崖式崩到 5 dB** | 逐质量档位打印字节数与 PSNR，发现 q=62 起某一块整块解码成饱和值 | 自研的 EOB/ZRL/变长系数混流在非零系数变多时会失步 → 换成**每块长度前缀**格式（结构上不可能失步），并保留 DC 差分 + AC 游程 |
| 4 | **AC 系数只用低 8 位存储**，高质量时回绕，导致 q=90 的 PSNR 反而低于 q=20 | 「质量单调性」用例 | 改为变长：窄系数 1 字节、宽系数 2 字节 |
| 5 | **无损 RLE 在噪声图上膨胀到 0.51x** | 端到端跑一遍看压缩比 | 加模式位：RLE 更大时直接原样存储 |
| 6 | **leader 或 trailer 自己丢掉时，重传永远不收敛** | 多个随机种子里有两个种子块收不齐 | 原实现只按缺口补 payload，且缺口要等 trailer 到了才算得出来 → 改成**选择性重传**：leader 按 `in_block` 判、缺口实时算、trailer 每轮补 |

> 第 1、2、3、4、6 条都是**先写用例、后改实现**发现的 —— 这也是为什么这个项目把测试摆在显眼位置。

---

## 6. 许可

MIT
