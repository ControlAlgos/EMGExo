#!/usr/bin/env python3
"""
Convert prepared .npz EMG data (from data_prep.py) into flat binary files
for the C++ DatasetStreamer.

For each gesture class, all [3, 205] windows are concatenated into a
continuous 3-channel time-series, resampled from 2048 Hz to 1000 Hz,
and written as interleaved float32: [sample0_ch0, sample0_ch1, sample0_ch2,
                                      sample1_ch0, sample1_ch1, sample1_ch2, ...]

Usage:
    python convert_npz_to_bin.py --npz prepared_data/healthy_train.npz --out emg_bin/
"""

from __future__ import annotations

import argparse
import json
import os
import struct

import numpy as np
from scipy.signal import resample

GESTURE_NAMES = {
    0: "rest",
    1: "wrist_flex",
    2: "wrist_ext",
    3: "mass_flex",
    4: "mass_ext",
    5: "pronation",
    6: "supination",
}

SRC_FS = 2048
DST_FS = 1000


def convert(npz_path: str, out_dir: str, min_windows: int = 10):
    os.makedirs(out_dir, exist_ok=True)

    data = np.load(npz_path)
    X = data["X"]  # (N, 3, 205)
    y = data["y"]  # (N,)

    print(f"Loaded {npz_path}: {X.shape[0]} windows, {X.shape[1]} channels, "
          f"{X.shape[2]} samples/window")

    manifest = {}

    for cls_id, cls_name in GESTURE_NAMES.items():
        mask = y == cls_id
        windows = X[mask]  # (n, 3, 205)

        if len(windows) < min_windows:
            print(f"  [{cls_name}] only {len(windows)} windows — skipping "
                  f"(need >= {min_windows})")
            continue

        # Concatenate windows along time axis: (3, n*205)
        concat = np.concatenate([w for w in windows], axis=1)
        n_src = concat.shape[1]

        # Resample from 2048 Hz to 1000 Hz
        n_dst = int(n_src * DST_FS / SRC_FS)
        resampled = np.zeros((3, n_dst), dtype=np.float32)
        for ch in range(3):
            resampled[ch] = resample(concat[ch], n_dst).astype(np.float32)

        # Interleave: [sample][channel] layout
        interleaved = resampled.T.copy()  # (n_dst, 3), C-contiguous
        assert interleaved.shape == (n_dst, 3)

        bin_path = os.path.join(out_dir, f"{cls_name}.bin")
        interleaved.tofile(bin_path)

        manifest[str(cls_id)] = {
            "file": f"{cls_name}.bin",
            "name": cls_name,
            "num_samples": n_dst,
            "num_channels": 3,
            "sample_rate_hz": DST_FS,
            "source_windows": int(len(windows)),
        }

        size_kb = os.path.getsize(bin_path) / 1024
        duration_s = n_dst / DST_FS
        print(f"  [{cls_name}] {len(windows)} windows -> {n_dst} samples "
              f"({duration_s:.1f}s) -> {bin_path} ({size_kb:.1f} KB)")

    manifest_path = os.path.join(out_dir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nManifest written to {manifest_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert .npz EMG data to flat binary for C++ streamer")
    parser.add_argument("--npz", required=True,
                        help="Path to .npz file from data_prep.py")
    parser.add_argument("--out", default="emg_bin",
                        help="Output directory for .bin files")
    parser.add_argument("--min-windows", type=int, default=10,
                        help="Minimum windows per class to include")
    args = parser.parse_args()

    convert(args.npz, args.out, args.min_windows)


if __name__ == "__main__":
    main()
