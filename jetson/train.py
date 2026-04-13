"""
Training script for the EMG gesture CNN.

Trains two independent models on the prepared PhysioMiO data:
  - healthy_model.pth   (trained on healthy-arm recordings)
  - stroke_model.pth    (trained on impaired-arm recordings)

Both use the same EMGCNN architecture and 7-class gesture labels.

Usage:
    python -m train --data_dir ./prepared_data --out_dir ./checkpoints
    python -m train --data_dir ./prepared_data --arm healthy   # single arm
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from model import EMGCNN, NUM_CLASSES, count_parameters

# ======================== Defaults ============================

BATCH_SIZE = 256
LR = 1e-3
EPOCHS = 60
WEIGHT_DECAY = 1e-4
PATIENCE = 12  # early-stopping patience (epochs without val improvement)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

GESTURE_NAMES = [
    "Rest", "Wrist Flexion", "Wrist Extension",
    "Mass Flexion", "Mass Extension",
    "Pronation", "Supination",
]

# ======================== Data loading ========================


def load_split(data_dir: str, arm: str, split: str):
    """Load a prepared .npz split.  Returns (X, y) tensors."""
    path = os.path.join(data_dir, f"{arm}_{split}.npz")
    if not os.path.exists(path):
        return None, None
    d = np.load(path)
    X = torch.from_numpy(d["X"]).float()
    y = torch.from_numpy(d["y"]).long()
    return X, y


def make_loader(X, y, batch_size: int, shuffle: bool = True):
    ds = TensorDataset(X, y)
    return DataLoader(ds, batch_size=batch_size, shuffle=shuffle,
                      num_workers=2, pin_memory=(DEVICE == "cuda"))


def compute_class_weights(y: torch.Tensor, num_classes: int = NUM_CLASSES) -> torch.Tensor:
    """Inverse-frequency class weights for balanced CrossEntropy."""
    counts = torch.bincount(y, minlength=num_classes).float()
    counts[counts == 0] = 1.0
    weights = 1.0 / counts
    weights /= weights.sum()
    return weights * num_classes


# ======================== Training loop =======================


def train_one_epoch(model, loader, criterion, optimizer, device):
    model.train()
    total_loss, correct, total = 0.0, 0, 0
    for X_batch, y_batch in loader:
        X_batch, y_batch = X_batch.to(device), y_batch.to(device)
        optimizer.zero_grad()
        logits = model(X_batch)
        loss = criterion(logits, y_batch)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * len(y_batch)
        correct += (logits.argmax(1) == y_batch).sum().item()
        total += len(y_batch)
    return total_loss / total, correct / total


@torch.no_grad()
def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss, correct, total = 0.0, 0, 0
    per_class_correct = torch.zeros(NUM_CLASSES)
    per_class_total = torch.zeros(NUM_CLASSES)

    for X_batch, y_batch in loader:
        X_batch, y_batch = X_batch.to(device), y_batch.to(device)
        logits = model(X_batch)
        loss = criterion(logits, y_batch)
        preds = logits.argmax(1)
        total_loss += loss.item() * len(y_batch)
        correct += (preds == y_batch).sum().item()
        total += len(y_batch)
        for c in range(NUM_CLASSES):
            mask = y_batch == c
            per_class_correct[c] += (preds[mask] == c).sum().item()
            per_class_total[c] += mask.sum().item()

    acc = correct / total
    per_class_total[per_class_total == 0] = 1
    per_class_acc = per_class_correct / per_class_total
    balanced_acc = per_class_acc.mean().item()

    return total_loss / total, acc, balanced_acc, per_class_acc


def train_model(arm_name: str, data_dir: str, out_dir: str,
                epochs: int = EPOCHS, batch_size: int = BATCH_SIZE,
                lr: float = LR):
    """Full training run for one arm type."""
    print(f"\n{'='*60}")
    print(f"  Training {arm_name.upper()} model")
    print(f"{'='*60}")

    X_train, y_train = load_split(data_dir, arm_name, "train")
    X_val, y_val = load_split(data_dir, arm_name, "val")

    if X_train is None:
        print(f"  No training data found for {arm_name}. Skipping.")
        return

    print(f"  Train: {len(y_train)} windows  |  Val: {len(y_val) if y_val is not None else 0} windows")
    print(f"  Train label distribution: {dict(zip(*np.unique(y_train.numpy(), return_counts=True)))}")

    train_loader = make_loader(X_train, y_train, batch_size)
    val_loader = make_loader(X_val, y_val, batch_size, shuffle=False) if y_val is not None else None

    model = EMGCNN().to(DEVICE)
    print(f"  Parameters: {count_parameters(model):,}")

    class_weights = compute_class_weights(y_train).to(DEVICE)
    criterion = nn.CrossEntropyLoss(weight=class_weights)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="max", factor=0.5, patience=5, min_lr=1e-6)

    best_bal_acc = 0.0
    best_epoch = 0
    no_improve = 0
    save_name = "healthy_model.pth" if arm_name == "healthy" else "stroke_model.pth"
    save_path = os.path.join(out_dir, save_name)

    for epoch in range(1, epochs + 1):
        t0 = time.time()
        train_loss, train_acc = train_one_epoch(model, train_loader, criterion, optimizer, DEVICE)

        if val_loader is not None:
            val_loss, val_acc, val_bal_acc, per_class = evaluate(model, val_loader, criterion, DEVICE)
            scheduler.step(val_bal_acc)

            improved = val_bal_acc > best_bal_acc
            if improved:
                best_bal_acc = val_bal_acc
                best_epoch = epoch
                no_improve = 0
                torch.save(model.state_dict(), save_path)
            else:
                no_improve += 1

            dt = time.time() - t0
            flag = " *" if improved else ""
            print(f"  [{epoch:3d}/{epochs}] {dt:.1f}s  "
                  f"train_loss={train_loss:.4f} train_acc={train_acc:.3f}  "
                  f"val_loss={val_loss:.4f} val_acc={val_acc:.3f} val_bal={val_bal_acc:.3f}{flag}")

            if no_improve >= PATIENCE:
                print(f"  Early stopping at epoch {epoch} (best: {best_epoch})")
                break
        else:
            torch.save(model.state_dict(), save_path)
            dt = time.time() - t0
            print(f"  [{epoch:3d}/{epochs}] {dt:.1f}s  "
                  f"train_loss={train_loss:.4f} train_acc={train_acc:.3f}")

    # Final evaluation on test set
    X_test, y_test = load_split(data_dir, arm_name, "test")
    if X_test is not None:
        model.load_state_dict(torch.load(save_path, weights_only=True))
        test_loader = make_loader(X_test, y_test, batch_size, shuffle=False)
        _, test_acc, test_bal_acc, per_class = evaluate(model, test_loader, criterion, DEVICE)

        print(f"\n  TEST RESULTS ({arm_name}):")
        print(f"    Accuracy:          {test_acc:.3f}")
        print(f"    Balanced Accuracy: {test_bal_acc:.3f}")
        print(f"    Per-class accuracy:")
        for i, name in enumerate(GESTURE_NAMES):
            print(f"      {name:20s}: {per_class[i]:.3f}")

    print(f"\n  Saved: {save_path}")
    return save_path


# ======================== CLI =================================


def main():
    parser = argparse.ArgumentParser(description="Train EMG gesture CNN")
    parser.add_argument("--data_dir", type=str,
                        default=str(Path(__file__).resolve().parent / "prepared_data"))
    parser.add_argument("--out_dir", type=str,
                        default=str(Path(__file__).resolve().parent / "checkpoints"))
    parser.add_argument("--arm", type=str, default="both",
                        choices=["healthy", "impaired", "both"],
                        help="Which arm type to train")
    parser.add_argument("--epochs", type=int, default=EPOCHS)
    parser.add_argument("--batch_size", type=int, default=BATCH_SIZE)
    parser.add_argument("--lr", type=float, default=LR)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    print(f"Device: {DEVICE}")

    arms = ["healthy", "impaired"] if args.arm == "both" else [args.arm]
    for arm in arms:
        train_model(arm, args.data_dir, args.out_dir,
                    epochs=args.epochs, batch_size=args.batch_size, lr=args.lr)


if __name__ == "__main__":
    main()
