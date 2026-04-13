"""
Export trained PyTorch models to ONNX and TensorRT for Jetson Orin Nano.

Pipeline:
  1. Load .pth checkpoint → PyTorch model
  2. Export to ONNX (opset 17, dynamic batch)
  3. Convert ONNX → TensorRT engine via trtexec (ships with JetPack)

Usage:
    # Export both models
    python -m export_model --checkpoint_dir ./checkpoints

    # Export a single model
    python -m export_model --pth ./checkpoints/healthy_model.pth

    # ONNX only (no TensorRT — useful on dev machine without GPU)
    python -m export_model --pth ./checkpoints/healthy_model.pth --onnx_only
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

import torch

from model import EMGCNN, IN_CHANNELS, WINDOW_SIZE


def export_onnx(pth_path: str, onnx_path: str,
                in_channels: int = IN_CHANNELS,
                window_size: int = WINDOW_SIZE) -> str:
    """Load a .pth checkpoint and export as ONNX."""
    model = EMGCNN(in_channels=in_channels)
    state = torch.load(pth_path, map_location="cpu", weights_only=True)
    model.load_state_dict(state)
    model.eval()

    dummy = torch.randn(1, in_channels, window_size)

    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        opset_version=17,
        input_names=["emg_input"],
        output_names=["gesture_logits"],
        dynamic_axes={
            "emg_input": {0: "batch"},
            "gesture_logits": {0: "batch"},
        },
    )
    print(f"  ONNX saved: {onnx_path}  ({os.path.getsize(onnx_path) / 1024:.1f} KB)")
    return onnx_path


def export_tensorrt(onnx_path: str, engine_path: str,
                    fp16: bool = True,
                    workspace_mb: int = 256) -> str | None:
    """Convert ONNX to TensorRT engine using trtexec."""
    trtexec = shutil.which("trtexec")
    if trtexec is None:
        # Common JetPack location
        candidate = "/usr/src/tensorrt/bin/trtexec"
        if os.path.isfile(candidate):
            trtexec = candidate
        else:
            print("  WARNING: trtexec not found. Skipping TensorRT conversion.")
            print("  Install TensorRT or run this step on the Jetson Orin Nano.")
            return None

    cmd = [
        trtexec,
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        f"--workspace={workspace_mb}",
        "--minShapes=emg_input:1x3x205",
        "--optShapes=emg_input:1x3x205",
        "--maxShapes=emg_input:16x3x205",
    ]
    if fp16:
        cmd.append("--fp16")

    print(f"  Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"  trtexec FAILED (exit {result.returncode}):")
        print(result.stderr[-2000:] if result.stderr else "(no stderr)")
        return None

    print(f"  TensorRT engine saved: {engine_path}  ({os.path.getsize(engine_path) / 1024:.1f} KB)")
    return engine_path


def convert_checkpoint(pth_path: str, out_dir: str, onnx_only: bool = False):
    """Full conversion pipeline for a single .pth file."""
    stem = Path(pth_path).stem
    onnx_path = os.path.join(out_dir, f"{stem}.onnx")
    engine_path = os.path.join(out_dir, f"{stem}.engine")

    print(f"\nConverting {pth_path}")
    export_onnx(pth_path, onnx_path)

    if not onnx_only:
        export_tensorrt(onnx_path, engine_path)


def main():
    parser = argparse.ArgumentParser(description="Export EMG CNN to ONNX / TensorRT")
    parser.add_argument("--pth", type=str, default=None,
                        help="Path to a single .pth checkpoint")
    parser.add_argument("--checkpoint_dir", type=str,
                        default=str(Path(__file__).resolve().parent / "checkpoints"),
                        help="Directory containing .pth files")
    parser.add_argument("--out_dir", type=str, default=None,
                        help="Output directory (defaults to same as checkpoint_dir)")
    parser.add_argument("--onnx_only", action="store_true",
                        help="Skip TensorRT conversion (for machines without trtexec)")
    args = parser.parse_args()

    out_dir = args.out_dir or args.checkpoint_dir
    os.makedirs(out_dir, exist_ok=True)

    if args.pth:
        convert_checkpoint(args.pth, out_dir, args.onnx_only)
    else:
        pth_files = list(Path(args.checkpoint_dir).glob("*.pth"))
        if not pth_files:
            print(f"No .pth files found in {args.checkpoint_dir}")
            sys.exit(1)
        for p in sorted(pth_files):
            convert_checkpoint(str(p), out_dir, args.onnx_only)


if __name__ == "__main__":
    main()
