"""
Real-time Jetson ↔ ESP32 UART bridge with TensorRT CNN inference.

Reads raw 3-channel EMG samples from the ESP32 over UART at 921600 baud,
maintains a circular buffer (100ms sliding window), runs a TensorRT gesture
classification model every 10ms, and writes the predicted Intent ID back
to the ESP32 over the same UART link.

Usage:
    python jetson_bridge.py --engine checkpoints/healthy_model.engine
    python jetson_bridge.py --engine checkpoints/stroke_model.engine --port /dev/ttyUSB1

    # Dry-run with PyTorch (no TensorRT required — for dev/testing)
    python jetson_bridge.py --pth checkpoints/healthy_model.pth --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import collections
import struct
import sys
import time
from pathlib import Path

import numpy as np

# ======================== Constants ===========================

BAUD_RATE = 921_600
WINDOW_SIZE = 205       # 100 ms at 2048 Hz
INFER_EVERY = 20        # run inference every 20 new samples (~10 ms)
VOTE_WINDOW = 5         # majority vote over last 5 predictions (~50 ms)
DEFAULT_RMS_THRESHOLD = 0.05  # below this → force Rest (calibrate empirically)

NUM_CHANNELS = 3
NUM_CLASSES = 7
CLASS_REST = 0

# UART frame constants
EMG_HEADER = 0xAA       # ESP32 → Jetson
INTENT_HEADER = 0xBB    # Jetson → ESP32
EMG_FRAME_LEN = 8       # [0xAA, ch0_h, ch0_l, ch1_h, ch1_l, ch2_h, ch2_l, XOR]
INTENT_FRAME_LEN = 3    # [0xBB, intent_id, XOR]

ADC_MAX = 4095.0         # ESP32 12-bit ADC

GESTURE_NAMES = [
    "Rest", "Wrist Flexion", "Wrist Extension",
    "Mass Flexion", "Mass Extension",
    "Pronation", "Supination",
]

# ======================== Circular buffer =====================


class CircularBuffer:
    """Fixed-size ring buffer for (NUM_CHANNELS, WINDOW_SIZE) EMG data."""

    def __init__(self, channels: int = NUM_CHANNELS, length: int = WINDOW_SIZE):
        self.buf = np.zeros((channels, length), dtype=np.float32)
        self.write_idx = 0
        self.count = 0
        self.length = length

    def push(self, sample: np.ndarray):
        """Push one (channels,) sample into the buffer."""
        self.buf[:, self.write_idx % self.length] = sample
        self.write_idx += 1
        self.count = min(self.count + 1, self.length)

    @property
    def full(self) -> bool:
        return self.count >= self.length

    def snapshot(self) -> np.ndarray:
        """Return a (1, channels, length) contiguous copy in time order."""
        if self.count < self.length:
            return self.buf[:, :self.count].copy()[np.newaxis]
        idx = self.write_idx % self.length
        ordered = np.concatenate([self.buf[:, idx:], self.buf[:, :idx]], axis=1)
        return ordered[np.newaxis].copy()


# ======================== TensorRT backend ====================


class TRTInferenceEngine:
    """Minimal TensorRT inference wrapper with pre-allocated CUDA buffers."""

    def __init__(self, engine_path: str):
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401 — initialises CUDA context

        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.d_input = cuda.mem_alloc(NUM_CHANNELS * WINDOW_SIZE * 4)
        self.d_output = cuda.mem_alloc(NUM_CLASSES * 4)
        self.h_output = np.empty(NUM_CLASSES, dtype=np.float32)
        self.stream = cuda.Stream()

        self.context.set_input_shape("emg_input", (1, NUM_CHANNELS, WINDOW_SIZE))

    def predict(self, window: np.ndarray) -> int:
        """Run inference on (1, 3, 205) float32 array. Returns class index."""
        import pycuda.driver as cuda

        np.copyto(self.h_output, self.h_output)  # ensure contiguous
        input_data = np.ascontiguousarray(window, dtype=np.float32)

        cuda.memcpy_htod_async(self.d_input, input_data, self.stream)
        self.context.execute_async_v2(
            bindings=[int(self.d_input), int(self.d_output)],
            stream_handle=self.stream.handle,
        )
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()

        return int(self.h_output.argmax())


# ======================== PyTorch fallback =====================


class TorchInferenceEngine:
    """Fallback inference engine using PyTorch (for dev machines without TRT)."""

    def __init__(self, pth_path: str):
        import torch
        from model import EMGCNN

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = EMGCNN().to(self.device)
        state = torch.load(pth_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state)
        self.model.eval()

    def predict(self, window: np.ndarray) -> int:
        import torch
        with torch.no_grad():
            x = torch.from_numpy(window).float().to(self.device)
            logits = self.model(x)
            return int(logits.argmax(1).item())


# ======================== UART I/O ============================


def parse_emg_frame(frame: bytes) -> np.ndarray | None:
    """Parse an 8-byte EMG frame into (3,) float32 normalised values.
    Returns None if checksum fails."""
    if len(frame) != EMG_FRAME_LEN or frame[0] != EMG_HEADER:
        return None
    xor = 0
    for b in frame[:-1]:
        xor ^= b
    if xor != frame[-1]:
        return None
    ch0 = (frame[1] << 8 | frame[2]) / ADC_MAX
    ch1 = (frame[3] << 8 | frame[4]) / ADC_MAX
    ch2 = (frame[5] << 8 | frame[6]) / ADC_MAX
    return np.array([ch0, ch1, ch2], dtype=np.float32)


def build_intent_frame(intent_id: int) -> bytes:
    """Build a 3-byte intent frame: [0xBB, id, XOR]."""
    xor = INTENT_HEADER ^ intent_id
    return bytes([INTENT_HEADER, intent_id & 0xFF, xor & 0xFF])


# ======================== Main loop ===========================


def run(port: str, engine, rms_threshold: float = DEFAULT_RMS_THRESHOLD):
    """Main real-time loop with majority voting and RMS magnitude gate."""
    import serial

    ser = serial.Serial(port, BAUD_RATE, timeout=0.001)
    buf = CircularBuffer()
    vote_buf = collections.deque(maxlen=VOTE_WINDOW)
    samples_since_infer = 0
    last_voted_intent = -1
    infer_count = 0
    gate_count = 0
    start_time = time.perf_counter()

    print(f"Listening on {port} at {BAUD_RATE} baud …  (Ctrl+C to stop)")
    print(f"  RMS gate threshold: {rms_threshold:.4f}")
    print(f"  Majority vote window: {VOTE_WINDOW} predictions")

    raw_buf = bytearray()

    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            raw_buf.extend(chunk)

            while len(raw_buf) >= EMG_FRAME_LEN:
                try:
                    hdr_idx = raw_buf.index(EMG_HEADER)
                except ValueError:
                    raw_buf.clear()
                    break
                if hdr_idx > 0:
                    del raw_buf[:hdr_idx]

                if len(raw_buf) < EMG_FRAME_LEN:
                    break

                frame = bytes(raw_buf[:EMG_FRAME_LEN])
                sample = parse_emg_frame(frame)
                if sample is not None:
                    del raw_buf[:EMG_FRAME_LEN]
                    buf.push(sample)
                    samples_since_infer += 1
                else:
                    del raw_buf[:1]
                    continue

                if buf.full and samples_since_infer >= INFER_EVERY:
                    samples_since_infer = 0
                    window = buf.snapshot()  # (1, 3, 205)

                    # --- Magnitude gate: skip CNN if signal is below threshold ---
                    rms = float(np.sqrt(np.mean(window * window)))
                    if rms < rms_threshold:
                        intent = CLASS_REST
                        gate_count += 1
                    else:
                        intent = engine.predict(window)

                    infer_count += 1

                    # --- Sliding majority vote ---
                    vote_buf.append(intent)
                    counts = collections.Counter(vote_buf)
                    voted_intent = counts.most_common(1)[0][0]

                    if voted_intent != last_voted_intent:
                        last_voted_intent = voted_intent
                        ser.write(build_intent_frame(voted_intent))
                        elapsed = time.perf_counter() - start_time
                        print(f"  [{elapsed:8.2f}s] Intent {voted_intent} = "
                              f"{GESTURE_NAMES[voted_intent]}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        print(f"\nStopped. {infer_count} inferences in {elapsed:.1f}s "
              f"({infer_count / elapsed:.1f} Hz)")
        print(f"  RMS gate bypassed CNN {gate_count} times "
              f"({gate_count / max(1, infer_count) * 100:.1f}%)")
    finally:
        ser.close()


# ======================== CLI =================================


def main():
    parser = argparse.ArgumentParser(description="Jetson real-time EMG inference bridge")
    parser.add_argument("--engine", type=str, default=None,
                        help="Path to TensorRT .engine file")
    parser.add_argument("--pth", type=str, default=None,
                        help="Path to PyTorch .pth file (fallback, no TRT needed)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="UART serial port")
    parser.add_argument("--rms_threshold", type=float, default=DEFAULT_RMS_THRESHOLD,
                        help="RMS magnitude gate — below this, force Rest (skip CNN)")
    args = parser.parse_args()

    if args.engine:
        print(f"Loading TensorRT engine: {args.engine}")
        engine = TRTInferenceEngine(args.engine)
    elif args.pth:
        print(f"Loading PyTorch model (fallback): {args.pth}")
        engine = TorchInferenceEngine(args.pth)
    else:
        print("ERROR: provide --engine (TensorRT) or --pth (PyTorch fallback)")
        sys.exit(1)

    run(args.port, engine, rms_threshold=args.rms_threshold)


if __name__ == "__main__":
    main()
