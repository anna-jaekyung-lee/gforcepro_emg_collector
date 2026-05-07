#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gForcePRO EMG bin → CSV 변환기
사용법: python bin_to_csv.py <파일명.bin> [채널수] [데이터타입]

예시:
  python bin_to_csv.py data.bin          # 기본값: 8채널, uint8
  python bin_to_csv.py data.bin 8 uint8  # 명시적 지정
  python bin_to_csv.py data.bin 8 int16  # 16bit 모드
"""

import sys
import numpy as np
import os

# ── 설정 ──────────────────────────────────────────
DEFAULT_CHANNELS = 8
DEFAULT_DTYPE = "uint8"  # uint8 / int8 / int16 / uint16 / float32

DTYPE_MAP = {
    "uint8":   (np.uint8,   1),
    "int8":    (np.int8,    1),
    "uint16":  (np.uint16,  2),
    "int16":   (np.int16,   2),
    "float32": (np.float32, 4),
}
# ──────────────────────────────────────────────────


def convert(bin_path, n_channels=DEFAULT_CHANNELS, dtype_str=DEFAULT_DTYPE):
    if not os.path.exists(bin_path):
        print(f"[ERROR] 파일을 찾을 수 없어요: {bin_path}")
        sys.exit(1)

    if dtype_str not in DTYPE_MAP:
        print(f"[ERROR] 지원하지 않는 데이터 타입: {dtype_str}")
        print(f"  지원 타입: {list(DTYPE_MAP.keys())}")
        sys.exit(1)

    np_dtype, bytes_per_sample = DTYPE_MAP[dtype_str]
    bytes_per_row = n_channels * bytes_per_sample
    file_size = os.path.getsize(bin_path)

    if file_size % bytes_per_row != 0:
        print(f"[WARNING] 파일 크기({file_size} bytes)가 행 크기({bytes_per_row} bytes)로 나눠떨어지지 않아요.")
        print(f"  나머지 {file_size % bytes_per_row} bytes는 잘려요.")

    n_samples = file_size // bytes_per_row

    print(f"[INFO] 파일: {bin_path}")
    print(f"[INFO] 크기: {file_size:,} bytes")
    print(f"[INFO] 채널: {n_channels}ch / 타입: {dtype_str}")
    print(f"[INFO] 샘플 수: {n_samples:,} samples/ch")

    # 읽기
    raw = np.fromfile(bin_path, dtype=np_dtype)
    data = raw[: n_samples * n_channels].reshape(n_samples, n_channels)

    # CSV 저장
    csv_path = os.path.splitext(bin_path)[0] + ".csv"
    header = ",".join([f"CH{i+1}" for i in range(n_channels)])

    np.savetxt(csv_path, data, delimiter=",", header=header, comments="", fmt="%d" if dtype_str != "float32" else "%.6f")

    print(f"[OK] 저장 완료: {csv_path}")
    print(f"     ({n_samples:,} rows × {n_channels} cols)")


if __name__ == "__main__":
    args = sys.argv[1:]

    if len(args) == 0:
        print(__doc__)
        sys.exit(0)

    bin_path  = args[0]
    n_ch      = int(args[1])    if len(args) > 1 else DEFAULT_CHANNELS
    dtype_str = args[2].lower() if len(args) > 2 else DEFAULT_DTYPE

    convert(bin_path, n_ch, dtype_str)
