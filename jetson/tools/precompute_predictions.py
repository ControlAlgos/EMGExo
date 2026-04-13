#!/usr/bin/env python3
"""
Pre-compute CNN predictions from the ORIGINAL .npz data (native 2048Hz).

For each gesture class in the .npz file, concatenates all windows into a
continuous stream, then slides a 205-sample window every STRIDE samples,
runs the trained CNN, and saves a companion .pred.bin file.

Each .pred.bin entry: [uint8 class_id, float32 confidence] = 5 bytes.
The stride (in 1kHz-equivalent samples) is also saved so the C++
DatasetStreamer can index correctly.

Also generates a per-gesture accuracy report.

Usage:
    python precompute_predictions.py \
        --model ../checkpoints/healthy_model.pth \
        --npz ../prepared_data/healthy_train.npz \
        --bin_dir ../../emg_bin/
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time

import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from model import EMGCNN

WINDOW_SIZE = 205   # must match model training
NUM_CHANNELS = 3
STRIDE_WINDOWS = 1  # predict every window (windows overlap 90% in training)

GESTURE_NAMES = {
    0: "rest",
    1: "wrist_flex",
    2: "wrist_ext",
    3: "mass_flex",
    4: "mass_ext",
    5: "pronation",
    6: "supination",
}

# The .bin files are at 1kHz. Predictions are spaced every PRED_STRIDE
# 1kHz-samples apart so the C++ DatasetStreamer can index them.
# Each original 2048Hz window spans ~100ms = 100 samples at 1kHz.
# The training step size is 20 samples at 2048Hz = ~10ms = 10 samples at 1kHz.
# We generate one prediction per training window, then map to 1kHz time.
PRED_STRIDE_1KHZ = 10  # one prediction every 10ms of 1kHz playback


def precompute(model_path: str, npz_path: str, bin_dir: str):
    device = "cuda" if torch.cuda.is_available() else "cpu"

    model = EMGCNN()
    state = torch.load(model_path, map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    print(f"Loaded model: {model_path} (device: {device})")

    data = np.load(npz_path)
    X = data["X"]  # (N, 3, 205) — original 2048Hz windows
    y = data["y"]  # (N,) — ground truth labels
    print(f"Loaded {npz_path}: {X.shape[0]} windows")

    manifest_path = os.path.join(bin_dir, "manifest.json")
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            manifest = json.load(f)
    else:
        manifest = {}

    batch_size = 512
    total_correct = 0
    total_count = 0

    print(f"\n{'Gesture':<16} {'Windows':>8} {'Accuracy':>10} {'Avg Conf':>10} {'File'}")
    print("-" * 72)

    for cls_id, cls_name in GESTURE_NAMES.items():
        mask = y == cls_id
        cls_windows = X[mask]  # (n, 3, 205)
        n = len(cls_windows)

        if n == 0:
            print(f"  [{cls_name}] no windows — skipping")
            continue

        all_preds = []
        all_confs = []
        correct = 0

        for start in range(0, n, batch_size):
            end = min(start + batch_size, n)
            batch = torch.from_numpy(cls_windows[start:end]).float().to(device)

            with torch.no_grad():
                logits = model(batch)
                probs = F.softmax(logits, dim=1)
                confs, preds = probs.max(dim=1)

            all_preds.extend(preds.cpu().numpy().tolist())
            all_confs.extend(confs.cpu().numpy().tolist())
            correct += (preds.cpu().numpy() == cls_id).sum()

        accuracy = correct / n
        avg_conf = np.mean(all_confs)
        total_correct += correct
        total_count += n

        # Save .pred.bin: [uint8 class_id, float32 confidence] per window
        pred_path = os.path.join(bin_dir, cls_name + ".pred.bin")
        with open(pred_path, "wb") as f:
            for p, c in zip(all_preds, all_confs):
                f.write(struct.pack("<Bf", p, c))

        pred_kb = os.path.getsize(pred_path) / 1024
        print(f"{cls_name:<16} {n:>8} {accuracy:>9.1%} {avg_conf:>10.3f} "
              f"{pred_path} ({pred_kb:.1f} KB)")

        # Update manifest
        cls_str = str(cls_id)
        if cls_str not in manifest:
            manifest[cls_str] = {"name": cls_name}
        manifest[cls_str]["pred_file"] = cls_name + ".pred.bin"
        manifest[cls_str]["pred_count"] = len(all_preds)
        manifest[cls_str]["pred_stride"] = PRED_STRIDE_1KHZ
        manifest[cls_str]["pred_accuracy"] = float(accuracy)

    overall_acc = total_correct / total_count if total_count > 0 else 0
    print("-" * 72)
    print(f"{'OVERALL':<16} {total_count:>8} {overall_acc:>9.1%}")

    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nManifest updated: {manifest_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Pre-compute CNN predictions from original .npz data")
    parser.add_argument("--model", required=True,
                        help="Path to trained .pth model")
    parser.add_argument("--npz", required=True,
                        help="Path to .npz file (from data_prep.py)")
    parser.add_argument("--bin_dir", required=True,
                        help="Directory containing .bin EMG files and manifest.json")
    args = parser.parse_args()

    precompute(args.model, args.npz, args.bin_dir)


if __name__ == "__main__":
    main()
