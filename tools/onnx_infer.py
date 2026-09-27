# -*- coding: utf-8 -*-
"""用 ONNX Runtime 跑同一份模型，并和 C 侧前向推理逐值对比。

这条链路就是「训练框架 → ONNX → 目标平台」的最小可验证版本：
    PyTorch 训练（tools/export_onnx.py）
      → model.onnx（ONNX Runtime 跑一遍）
      → cnn_weights.h（src/infer_cnn.c 用纯 C 再跑一遍）
      → 三者对同一个输入的输出必须一致（容差 1e-4）

用法：
    python tools/onnx_infer.py            # 只跑 ONNX Runtime + 与 PyTorch 比
    python tools/onnx_infer.py --cpp      # 额外编译并运行 C 侧，三方一起比
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W_DIR = os.path.join(ROOT, 'weights')
TOL = 1e-4


def run_onnx(x: np.ndarray) -> float:
    import onnxruntime as ort
    sess = ort.InferenceSession(os.path.join(W_DIR, 'model.onnx'),
                                providers=['CPUExecutionProvider'])
    out = sess.run(['prob'], {'image': x})[0]
    return float(np.asarray(out).reshape(-1)[0])


def run_cpp() -> float | None:
    """编译并运行 src/infer_cnn.c，解析它打印的 CNN_OUT。

    注意：gcc（MSYS2）对**含中文的绝对路径**会在链接阶段报
    `cannot open output file ...: No such file or directory`，
    所以这里统一用 cwd=ROOT + 相对路径调用。
    """
    cc = os.environ.get('CC', 'gcc')
    os.makedirs(os.path.join(ROOT, 'build'), exist_ok=True)
    cmd = [cc, '-std=c99', '-O2', '-Wall', '-Wextra',
           '-Iinclude', '-Iweights', 'src/infer_cnn.c', '-o',
           os.path.join('build', 'clink_infer.exe'), '-lm']
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if r.returncode != 0:
        print('C 编译失败：\n%s' % r.stderr)
        return None
    r2 = subprocess.run([os.path.join('build', 'clink_infer.exe')],
                        capture_output=True, text=True, cwd=ROOT)
    print(r2.stdout.strip())
    m = re.search(r'CNN_OUT\s+([0-9.]+)', r2.stdout)
    return float(m.group(1)) if m else None


def main() -> int:
    npy = os.path.join(W_DIR, 'test_input.npy')
    if not os.path.exists(npy):
        print('缺少 %s，请先运行 tools/export_onnx.py' % npy)
        return 1
    x = np.load(npy).astype(np.float32)
    meta = json.load(open(os.path.join(W_DIR, 'meta.json'), encoding='utf-8'))

    pt = float(meta['ref_output'])
    ort_v = run_onnx(x)
    print('PyTorch     = %.8f' % pt)
    print('ONNX Runtime= %.8f' % ort_v)
    d1 = abs(ort_v - pt)
    print('差值        = %.3e' % d1)

    ok = d1 < TOL
    if '--cpp' in sys.argv:
        cpp_v = run_cpp()
        if cpp_v is None:
            return 1
        d2 = abs(cpp_v - pt)
        print('C 前向      = %.8f   与 PyTorch 差值 %.3e' % (cpp_v, d2))
        ok = ok and d2 < TOL

    print('\n结论：%s' % ('三方一致（容差 %.0e）' % TOL if ok else '★ 数值不一致'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
