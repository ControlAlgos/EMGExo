"""
PhysioMiO Data Preparation — loads HD-sEMG parquet files, spatially averages
64 channels into 3 virtual muscle groups, applies bandpass filtering and
Z-score normalization, then generates sliding-window segments for CNN training.

Produces separate .npz datasets for healthy-arm and impaired-arm recordings
with a patient-based train/val/test split (no patient leakage).

Usage:
    python -m data_prep --data_dir ../physiomio/patdata/data --out_dir ./prepared_data
"""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from scipy import signal

# ======================== Constants ============================

FS = 2048  # PhysioMiO sampling rate (Hz)

WINDOW_SAMPLES = 205   # 100 ms at 2048 Hz
STEP_SAMPLES = 20      # 10 ms  at 2048 Hz  (≈90 % overlap)

GESTURE_LABELS = {
    "Rest": 0,
    "Wrist Flexion": 1,
    "Wrist Extension": 2,
    "Mass Flexion": 3,      # all-finger grip (replaces Radial Deviation)
    "Mass Extension": 4,    # all-finger open  (replaces Ulnar Deviation)
    "Pronation": 5,
    "Supination": 6,
}
NUM_CLASSES = len(GESTURE_LABELS)

# HD-sEMG 8×8 grid → 3 virtual muscle groups.  Channel indices are 1-based
# to match the parquet column names (channel_01 … channel_64).
CHANNEL_GROUPS = {
    "flexors":    list(range(1, 22)),   # channels  1–21  (anterior forearm)
    "extensors":  list(range(22, 44)),  # channels 22–43  (posterior forearm)
    "pronators":  list(range(44, 65)),  # channels 44–64  (lateral forearm)
}

# ======================== Helpers =============================


def _col_names(indices: list[int]) -> list[str]:
    return [f"channel_{i:02d}" for i in indices]


def _build_bandpass(low: float = 20.0, high: float = 450.0,
                    fs: float = FS, order: int = 4):
    nyq = fs / 2.0
    b, a = signal.butter(order, [low / nyq, high / nyq], btype="band")
    return b, a


def _normalise_gesture_name(name: str) -> Optional[str]:
    """Map PhysioMiO movement_type strings to our canonical 7-class names."""
    # PhysioMiO actual labels → our canonical names
    physio_map = {
        "Rest":                "Rest",
        "WristVolarFlexion":   "Wrist Flexion",
        "WristDorsiFlexion":   "Wrist Extension",
        "MassFlexion":         "Mass Flexion",
        "MassExtension":       "Mass Extension",
        "ForearmPronation":    "Pronation",
        "ForearmSupination":   "Supination",
    }
    canonical = physio_map.get(name.strip())
    if canonical and canonical in GESTURE_LABELS:
        return canonical
    return None


# ======================== Core pipeline =======================


def load_and_reduce(filepath: str) -> Optional[pd.DataFrame]:
    """Load one parquet, keep only the 7 target gestures, and spatially
    average 64 channels into 3 virtual channels."""
    df = pd.read_parquet(filepath)

    if "movement_type" not in df.columns:
        return None

    # Map gesture names to canonical labels
    df["gesture"] = df["movement_type"].map(_normalise_gesture_name)
    df = df.dropna(subset=["gesture"])
    if df.empty:
        return None

    df["label"] = df["gesture"].map(GESTURE_LABELS).astype(np.int64)

    # Spatial averaging into 3 virtual channels
    out = pd.DataFrame({"label": df["label"].values})
    for group_name, ch_indices in CHANNEL_GROUPS.items():
        cols = [c for c in _col_names(ch_indices) if c in df.columns]
        if not cols:
            return None
        out[group_name] = df[cols].values.mean(axis=1)

    return out


def bandpass_filter(data: np.ndarray, b: np.ndarray, a: np.ndarray) -> np.ndarray:
    """Apply zero-phase bandpass to each channel (rows = channels)."""
    filtered = np.empty_like(data)
    for i in range(data.shape[0]):
        filtered[i] = signal.filtfilt(b, a, data[i])
    return filtered


def zscore_normalise(data: np.ndarray) -> np.ndarray:
    """Z-score per channel (rows = channels) across time axis."""
    mu = data.mean(axis=1, keepdims=True)
    sigma = data.std(axis=1, keepdims=True)
    sigma[sigma < 1e-8] = 1.0
    return (data - mu) / sigma


def sliding_windows(data: np.ndarray, labels: np.ndarray,
                    win: int = WINDOW_SAMPLES, step: int = STEP_SAMPLES):
    """Generate (windows, window_labels) from continuous recording.
    data  : (3, T)
    labels: (T,)
    Each window is assigned the label that covers its majority of samples.
    """
    T = data.shape[1]
    n_windows = max(0, (T - win) // step + 1)
    if n_windows == 0:
        return np.empty((0, 3, win), dtype=np.float32), np.empty((0,), dtype=np.int64)

    X = np.empty((n_windows, 3, win), dtype=np.float32)
    y = np.empty(n_windows, dtype=np.int64)

    for i in range(n_windows):
        s = i * step
        X[i] = data[:, s:s + win]
        # Majority vote for window label
        seg_labels = labels[s:s + win]
        counts = np.bincount(seg_labels, minlength=NUM_CLASSES)
        y[i] = counts.argmax()

    return X, y


def process_recording(filepath: str, bp_b: np.ndarray, bp_a: np.ndarray):
    """Full pipeline for one parquet file → (X_windows, y_labels)."""
    df = load_and_reduce(filepath)
    if df is None:
        return None, None

    channels = np.stack([df["flexors"].values,
                         df["extensors"].values,
                         df["pronators"].values], axis=0).astype(np.float64)

    channels = bandpass_filter(channels, bp_b, bp_a)
    channels = zscore_normalise(channels)

    labels = df["label"].values
    return sliding_windows(channels.astype(np.float32), labels)


# =================== Dataset assembly ========================


def discover_files(data_dir: str):
    """Return dict  {arm_type: [(patient_id, filepath), ...]}."""
    result = {"healthy": [], "impaired": []}
    for fpath in sorted(glob.glob(os.path.join(data_dir, "**", "*.parquet"), recursive=True)):
        parts = Path(fpath).parts
        patient_str = [p for p in parts if p.startswith("patient")]
        arm_str = [p for p in parts if p in ("healthy_arm", "impaired_arm")]
        if not patient_str or not arm_str:
            continue
        pid = int(patient_str[0].replace("patient", ""))
        arm = "healthy" if arm_str[0] == "healthy_arm" else "impaired"
        result[arm].append((pid, fpath))
    return result


def patient_split(patient_ids: np.ndarray, val_frac: float = 0.15,
                  test_frac: float = 0.15, seed: int = 42):
    """Split unique patient IDs into train / val / test sets."""
    rng = np.random.default_rng(seed)
    ids = np.unique(patient_ids)
    rng.shuffle(ids)
    n = len(ids)
    n_test = max(1, int(n * test_frac))
    n_val = max(1, int(n * val_frac))
    test_ids = set(ids[:n_test])
    val_ids = set(ids[n_test:n_test + n_val])
    train_ids = set(ids[n_test + n_val:])
    return train_ids, val_ids, test_ids


def build_dataset(file_list: list[tuple[int, str]], bp_b, bp_a):
    """Process a list of (patient_id, filepath) pairs.
    Returns X (N, 3, 205), y (N,), pids (N,)."""
    all_X, all_y, all_pids = [], [], []
    for pid, fpath in file_list:
        X, y = process_recording(fpath, bp_b, bp_a)
        if X is None or len(X) == 0:
            continue
        all_X.append(X)
        all_y.append(y)
        all_pids.append(np.full(len(y), pid, dtype=np.int32))
    if not all_X:
        return np.empty((0, 3, WINDOW_SAMPLES)), np.empty((0,)), np.empty((0,))
    return (np.concatenate(all_X),
            np.concatenate(all_y),
            np.concatenate(all_pids))


def prepare_arm(arm_files: list[tuple[int, str]], arm_name: str,
                out_dir: str, bp_b, bp_a, seed: int = 42):
    """Build and save train/val/test .npz files for one arm type."""
    print(f"\n{'='*60}")
    print(f"Processing {arm_name} arm  ({len(arm_files)} recordings)")
    print(f"{'='*60}")

    X_all, y_all, pids_all = build_dataset(arm_files, bp_b, bp_a)
    print(f"  Total windows: {len(y_all)}")
    if len(y_all) == 0:
        print("  WARNING: no windows produced — skipping.")
        return

    unique_pids = np.unique(pids_all)
    train_ids, val_ids, test_ids = patient_split(unique_pids, seed=seed)

    for split_name, id_set in [("train", train_ids), ("val", val_ids), ("test", test_ids)]:
        mask = np.isin(pids_all, list(id_set))
        Xs, ys = X_all[mask], y_all[mask]
        out_path = os.path.join(out_dir, f"{arm_name}_{split_name}.npz")
        np.savez_compressed(out_path, X=Xs, y=ys)
        label_dist = {GESTURE_LABELS_INV.get(int(k), k): int(v)
                      for k, v in zip(*np.unique(ys, return_counts=True))}
        print(f"  {split_name:5s}: {len(ys):7d} windows  |  patients {sorted(id_set)}  |  {label_dist}")


GESTURE_LABELS_INV = {v: k for k, v in GESTURE_LABELS.items()}


# ======================== CLI ================================


def main():
    parser = argparse.ArgumentParser(description="Prepare PhysioMiO data for CNN training")
    parser.add_argument("--data_dir", type=str,
                        default=str(Path(__file__).resolve().parent.parent / "physiomio" / "patdata" / "data"),
                        help="Root of the PhysioMiO patient data")
    parser.add_argument("--out_dir", type=str,
                        default=str(Path(__file__).resolve().parent / "prepared_data"),
                        help="Output directory for .npz files")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Discover all parquet files
    files = discover_files(args.data_dir)
    print(f"Found {len(files['healthy'])} healthy recordings, "
          f"{len(files['impaired'])} impaired recordings")

    if not files["healthy"] and not files["impaired"]:
        print("ERROR: no parquet files found. Check --data_dir path.")
        return

    # Print unique gesture names found in a sample file for verification
    sample_path = (files["healthy"] or files["impaired"])[0][1]
    sample_df = pd.read_parquet(sample_path)
    if "movement_type" in sample_df.columns:
        all_gestures = sample_df["movement_type"].unique()
        print(f"\nGesture types in sample file: {sorted(all_gestures)}")
        matched = [g for g in all_gestures if _normalise_gesture_name(g) is not None]
        print(f"Matched to 7-class set: {matched}")

    bp_b, bp_a = _build_bandpass()

    for arm_name, arm_files in files.items():
        if arm_files:
            prepare_arm(arm_files, arm_name, args.out_dir, bp_b, bp_a, seed=args.seed)

    print(f"\nDone. Output saved to {args.out_dir}/")


if __name__ == "__main__":
    main()
