#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gForcePRO EMG bin -> CSV Converter
Usage: python bin_to_csv.py <file.bin> [channels] [dtype]

Examples:
  python bin_to_csv.py data.bin            # default: 8ch, uint8
  python bin_to_csv.py data.bin 8 uint8    # explicit
  python bin_to_csv.py data.bin 8 int16    # 16-bit mode
"""

import sys
import numpy as np
import os

# ── Settings ──────────────────────────────────────
DEFAULT_CHANNELS = 8
DEFAULT_DTYPE    = "uint8"   # uint8 / int8 / int16 / uint16 / float32

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
        print(f"[ERROR] File not found: {bin_path}")
        sys.exit(1)

    if dtype_str not in DTYPE_MAP:
        print(f"[ERROR] Unsupported dtype: {dtype_str}")
        print(f"  Supported: {list(DTYPE_MAP.keys())}")
        sys.exit(1)

    np_dtype, bytes_per_sample = DTYPE_MAP[dtype_str]
    bytes_per_row = n_channels * bytes_per_sample
    file_size     = os.path.getsize(bin_path)

    if file_size % bytes_per_row != 0:
        remainder = file_size % bytes_per_row
        print(f"[WARNING] File size ({file_size} bytes) not evenly divisible by row size ({bytes_per_row} bytes).")
        print(f"  Trailing {remainder} bytes will be discarded.")

    n_samples = file_size // bytes_per_row

    print(f"[INFO] File    : {bin_path}")
    print(f"[INFO] Size    : {file_size:,} bytes")
    print(f"[INFO] Channels: {n_channels}ch / dtype: {dtype_str}")
    print(f"[INFO] Samples : {n_samples:,} per channel")

    raw  = np.fromfile(bin_path, dtype=np_dtype)
    data = raw[: n_samples * n_channels].reshape(n_samples, n_channels)

    csv_path = os.path.splitext(bin_path)[0] + ".csv"
    header   = ",".join([f"CH{i+1}" for i in range(n_channels)])
    fmt      = "%d" if dtype_str != "float32" else "%.6f"

    np.savetxt(csv_path, data, delimiter=",", header=header, comments="", fmt=fmt)

    print(f"[OK] Saved: {csv_path}")
    print(f"     ({n_samples:,} rows x {n_channels} cols)")


if __name__ == "__main__":
    args = sys.argv[1:]

    if len(args) == 0:
        print(__doc__)
        sys.exit(0)

    bin_path  = args[0]
    n_ch      = int(args[1])    if len(args) > 1 else DEFAULT_CHANNELS
    dtype_str = args[2].lower() if len(args) > 2 else DEFAULT_DTYPE

    convert(bin_path, n_ch, dtype_str)
