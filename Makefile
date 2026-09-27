CC      ?= gcc
PY      ?= python
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Werror -Iinclude
LDLIBS  ?= -lm

BUILD   := build
CORE    := src/csi2.c src/gvsp.c src/codec.c

.PHONY: all test sim infer onnx clean

all: test sim infer

$(BUILD):
	@mkdir -p $(BUILD)

## 单元测试（60 项，零第三方依赖）
test: $(BUILD)
	$(CC) $(CFLAGS) test/test_link.c $(CORE) -o $(BUILD)/clink_test$(EXE) $(LDLIBS)
	./$(BUILD)/clink_test$(EXE)

## 端到端链路仿真：编码 → CSI-2 → GVSP（含丢包重传）→ 重组 → 解码
sim: $(BUILD)
	$(CC) $(CFLAGS) src/pipeline.c $(CORE) -o $(BUILD)/clink_pipeline$(EXE) $(LDLIBS)
	@mkdir -p out
	./$(BUILD)/clink_pipeline$(EXE) --w 256 --h 128 --quality 75 --loss 150 --seed 20260927 --out out

## 导出 ONNX + 生成 C 权重头（需要 torch / onnx / onnxruntime）
onnx: $(BUILD)
	$(PY) tools/export_onnx.py
	$(PY) tools/onnx_infer.py --cpp

## 只跑 C 侧前向推理（权重头已存在时）
infer: $(BUILD)
	$(CC) $(CFLAGS) -Iweights src/infer_cnn.c -o $(BUILD)/clink_infer$(EXE) $(LDLIBS)
	./$(BUILD)/clink_infer$(EXE)

clean:
	rm -rf $(BUILD) out
