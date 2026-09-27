# -*- coding: utf-8 -*-
"""导出小 CNN 到 ONNX + 生成 C 头文件（供 src/infer_cnn.c 做嵌入式侧前向推理）。

模型（灰度 16×16 输入，二分类「表面合格 / 有缺陷」）：
    Conv2d(1→4, 3×3, pad=1) → ReLU → MaxPool2d(2)
    Conv2d(4→8, 3×3, pad=1) → ReLU → MaxPool2d(2)
    Flatten → Linear(128→1) → Sigmoid

为什么自己造一个小模型而不是直接用 inspection-ai-lab 的大模型：
    嵌入式侧要做的是「把模型搬到没有 Python 的环境里跑」，关键验证点是**前向数值能不能对上**，
    所以用一个能在 C 里手写前向、且权重体积可控的小网络最有说明力。
    合成数据是程序生成的（不是真实样本），README 里已声明边界。

用法：
    python tools/export_onnx.py            # 训练 → 导出 ONNX → 生成 C 权重头
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W_DIR = os.path.join(ROOT, 'weights')
SIZE = 16
SEED = 20260927


class TinyDefectNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 4, 3, padding=1)
        self.conv2 = nn.Conv2d(4, 8, 3, padding=1)
        self.fc = nn.Linear(8 * 4 * 4, 1)

    def forward(self, x):
        x = torch.relu(self.conv1(x))
        x = torch.max_pool2d(x, 2)
        x = torch.relu(self.conv2(x))
        x = torch.max_pool2d(x, 2)
        x = torch.flatten(x, 1)
        return torch.sigmoid(self.fc(x))


def make_dataset(n=512, seed=SEED):
    """合成「表面图像」：合格 = 平坦基底 + 噪声；缺陷 = 叠一条窄暗线。"""
    rng = np.random.default_rng(seed)
    x = np.zeros((n, 1, SIZE, SIZE), np.float32)
    y = np.zeros((n, 1), np.float32)
    for i in range(n):
        img = 0.65 + rng.normal(0, 0.04, (SIZE, SIZE))
        yy, xx = np.mgrid[0:SIZE, 0:SIZE].astype(np.float32)
        if rng.random() < 0.5:
            y[i, 0] = 1.0
            ang = rng.uniform(0, np.pi)
            cx, cy = rng.uniform(3, SIZE - 3, 2)
            d = np.abs(np.cos(ang) * (yy - cy) - np.sin(ang) * (xx - cx))
            img -= 0.35 * np.exp(-0.5 * (d / 0.9) ** 2)
        x[i, 0] = img
    return x, y


def export_c_header(model, path, test_input):
    """把权重写成 C 数组头文件，并带上一个确定的测试输入与 PyTorch 的参考输出。"""
    sd = {k: v.detach().numpy() for k, v in model.state_dict().items()}

    def arr(name, a, ctype='float'):
        flat = a.reshape(-1)
        body = ', '.join('%.8ff' % v for v in flat)
        return 'static const %s %s[%d] = {%s};\n' % (ctype, name, flat.size, body)

    with torch.no_grad():
        ref = float(model(torch.from_numpy(test_input)).item())

    lines = ['/* 由 tools/export_onnx.py 自动生成，请勿手改。 */',
             '#ifndef CLINK_CNN_WEIGHTS_H',
             '#define CLINK_CNN_WEIGHTS_H',
             '',
             '#define CNN_IN_H %d' % SIZE,
             '#define CNN_IN_W %d' % SIZE,
             '#define CNN_C1 4',
             '#define CNN_C2 8',
             '',
             arr('cnn_conv1_w', sd['conv1.weight']),
             arr('cnn_conv1_b', sd['conv1.bias']),
             arr('cnn_conv2_w', sd['conv2.weight']),
             arr('cnn_conv2_b', sd['conv2.bias']),
             arr('cnn_fc_w', sd['fc.weight']),
             arr('cnn_fc_b', sd['fc.bias']),
             '',
             '/* 与 ONNX / PyTorch 对数值用的确定输入 */',
             arr('cnn_test_input', test_input.reshape(-1)),
             '',
             '#define CNN_REF_OUTPUT %.8ff' % ref,
             '',
             '#endif /* CLINK_CNN_WEIGHTS_H */',
             '']
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return ref


def main():
    os.makedirs(W_DIR, exist_ok=True)
    torch.manual_seed(SEED)
    x, y = make_dataset()
    xt = torch.from_numpy(x)
    yt = torch.from_numpy(y)

    model = TinyDefectNet()
    opt = torch.optim.Adam(model.parameters(), lr=5e-3)
    lossf = nn.BCELoss()
    for ep in range(60):
        opt.zero_grad()
        out = model(xt)
        loss = lossf(out, yt)
        loss.backward()
        opt.step()
    with torch.no_grad():
        pred = (model(xt) > 0.5).float()
        acc = float((pred == yt).float().mean().item())
    print('训练完成：loss=%.4f  acc=%.4f' % (float(loss.item()), acc))

    # 确定测试输入（取一张有缺陷的样本）
    idx = int(np.argmax(y[:, 0]))
    test_input = x[idx:idx + 1].copy()

    model.eval()
    onnx_path = os.path.join(W_DIR, 'model.onnx')
    torch.onnx.export(
        model, torch.from_numpy(test_input), onnx_path,
        input_names=['image'], output_names=['prob'],
        opset_version=17, dynamo=False,
    )
    print('ONNX 已导出：%s' % onnx_path)

    h_path = os.path.join(W_DIR, 'cnn_weights.h')
    ref = export_c_header(model, h_path, test_input)
    print('C 权重头已生成：%s' % h_path)

    np.save(os.path.join(W_DIR, 'test_input.npy'), test_input)

    meta = {'acc': acc, 'ref_output': ref, 'test_index': idx,
            'input_shape': list(test_input.shape)}
    with open(os.path.join(W_DIR, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print('PyTorch 参考输出 = %.8f' % ref)
    return 0


if __name__ == '__main__':
    sys.exit(main())
