#!/usr/bin/env python3
"""
Linux Python equivalent of serial_write_threejoints_wrist.cpp

Sends Fourier-series wrist trajectories to ESP32 over serial.
  Motor 1 → Radial / Ulnar Deviation
  Motor 2 → Flexion / Extension
  Motor 3 → Pronation / Supination

Protocol (matches ESP32 firmware):
  write  "id\n"
  write  "theta_int\n"   (theta in rad × 1000, truncated to int)

Usage:
  python serial_write_wrist.py              # defaults to /dev/ttyUSB0
  python serial_write_wrist.py --port /dev/ttyUSB1
  python serial_write_wrist.py --cycles 3   # run 3 full cycles instead of 5
"""

import argparse
import math
import time
import serial

# ── Fourier coefficients (hip trajectory from ARMS lab) ──────────────
a0, a1, b1 = 40.69, 23.22, -8.65
a2, b2 = -4.487, 3.338
a3, b3 = 0.3995, 1.389
a4, b4 = 0.7047, 0.7989
a5, b5 = 1.078, 0.342
a6, b6 = -0.2732, 0.06696
w = 0.65

# Knee trajectory coefficients
ka0, ka1, kb1 = 25.7, -3.83, -19.28
ka2, kb2 = -8.54, 17.93
ka3, kb3 = 1.91, 3.77
ka4, kb4 = 1.09, 1.50
ka5, kb5 = 2.05, 0.58
ka6, kb6 = -0.31, -0.90

# Trajectory shaping
AMP_SHIFT_DEG = 23.0
TRAJ_PERIOD = 2.0 * math.pi / w
PHASE_SHIFT_DEG = 180.0
B_SHIFT = (PHASE_SHIFT_DEG / 360.0) * TRAJ_PERIOD  # phase shift in seconds

MOTOR_IDS = [1, 2, 3]
DT_MS = 10            # command interval (ms)
DT_S = DT_MS * 1e-3
HOMING_DUR_S = 5.0    # homing takes 5 seconds
BAUDRATE = 115200


def hip_fourier(t: float) -> float:
    return (a0
            + a1 * math.cos(t * w)   + b1 * math.sin(t * w)
            + a2 * math.cos(2*t*w)   + b2 * math.sin(2*t*w)
            + a3 * math.cos(3*t*w)   + b3 * math.sin(3*t*w)
            + a4 * math.cos(4*t*w)   + b4 * math.sin(4*t*w)
            + a5 * math.cos(5*t*w)   + b5 * math.sin(5*t*w)
            + a6 * math.cos(6*t*w)   + b6 * math.sin(6*t*w)
            - AMP_SHIFT_DEG)


def hip_fourier_shifted(t: float) -> float:
    ts = t - B_SHIFT
    return (a0
            + a1 * math.cos(ts * w)  + b1 * math.sin(ts * w)
            + a2 * math.cos(2*ts*w)  + b2 * math.sin(2*ts*w)
            + a3 * math.cos(3*ts*w)  + b3 * math.sin(3*ts*w)
            + a4 * math.cos(4*ts*w)  + b4 * math.sin(4*ts*w)
            + a5 * math.cos(5*ts*w)  + b5 * math.sin(5*ts*w)
            + a6 * math.cos(6*ts*w)  + b6 * math.sin(6*ts*w)
            - AMP_SHIFT_DEG)


def knee_fourier(t: float) -> float:
    return (ka0
            + ka1 * math.cos(t * w)  + kb1 * math.sin(t * w)
            + ka2 * math.cos(2*t*w)  + kb2 * math.sin(2*t*w)
            + ka3 * math.cos(3*t*w)  + kb3 * math.sin(3*t*w)
            + ka4 * math.cos(4*t*w)  + kb4 * math.sin(4*t*w)
            + ka5 * math.cos(5*t*w)  + kb5 * math.sin(5*t*w)
            + ka6 * math.cos(6*t*w)  + kb6 * math.sin(6*t*w)
            - AMP_SHIFT_DEG)


# ── Trajectory target at t=0 (used for homing end-point) ────────────
THETA_F_M1 = (math.pi / 180.0) * 1.25 * hip_fourier(0)         # Motor 1
THETA_F_M2 = (-math.pi / 180.0) * 2.0  * hip_fourier_shifted(0) # Motor 2
THETA_F_M3 = (-math.pi / 180.0) * 7.0  * knee_fourier(0)        # Motor 3


def send_cmd(ser: serial.Serial, motor_id: int, theta_rad: float):
    """Send one motor command over serial (matches ESP32 protocol)."""
    ser.write(f"{motor_id}\n".encode())
    theta_int = int(theta_rad * 1000)
    ser.write(f"{theta_int}\n".encode())


def homing_theta(theta_f: float, t: float, homing_w: float) -> float:
    """Sinusoidal interpolation from 0 → theta_f."""
    return theta_f + (0.0 - theta_f) * ((1.0 + math.sin(homing_w * t + math.pi / 2.0)) / 2.0)


def run(port: str, num_cycles: int):
    print(f"Opening {port} @ {BAUDRATE} baud …")
    ser = serial.Serial(port, BAUDRATE, timeout=0.1)
    # Opening the port toggles DTR → ESP32 resets → setup() runs.
    # setup() enters motors 1-4 into control mode with 1s delay each ≈ 4s total.
    print("Waiting 6s for ESP32 to reset and enter all motors into control mode …")
    time.sleep(6)
    ser.reset_input_buffer()  # flush any stale data from setup phase

    # ── Send initial zero-effort for each motor ─────────────────────
    print("Sending zero-effort to all motors …")
    for mid in MOTOR_IDS:
        send_cmd(ser, mid, 0.0)
    time.sleep(1.0)

    # ── Homing sequence ─────────────────────────────────────────────
    period = HOMING_DUR_S * 2.0
    homing_w = 2.0 * math.pi / period
    num_homing_steps = int(HOMING_DUR_S / DT_S)

    print(f"Homing ({HOMING_DUR_S}s) …")
    t0 = time.perf_counter()
    for step in range(num_homing_steps + 1):
        t = step * DT_S
        th1 = homing_theta(THETA_F_M1, t, homing_w)
        th2 = homing_theta(THETA_F_M2, t, homing_w)
        th3 = homing_theta(THETA_F_M3, t, homing_w)

        send_cmd(ser, 1, th1)
        send_cmd(ser, 2, th2)
        send_cmd(ser, 3, th3)

        # Print every 50 steps (~500ms)
        if step % 50 == 0:
            print(f"  homing t={t:.2f}s  M1={th1:+.4f}  M2={th2:+.4f}  M3={th3:+.4f}")

        # Maintain timing
        next_t = t0 + (step + 1) * DT_S
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

    # ── Walking trajectory ──────────────────────────────────────────
    run_time_s = TRAJ_PERIOD * num_cycles
    num_steps = int(run_time_s / DT_S)

    print(f"Running trajectory ({num_cycles} cycles, {run_time_s:.1f}s) …")
    print(f"  Motor 1 = Radial/Ulnar Deviation")
    print(f"  Motor 2 = Flexion/Extension")
    print(f"  Motor 3 = Pronation/Supination")

    t0 = time.perf_counter()
    for step in range(num_steps + 1):
        t = step * DT_S

        th1 = (math.pi / 180.0) * 1.25 * hip_fourier(t)
        th2 = (-math.pi / 180.0) * 2.0  * hip_fourier_shifted(t)
        th3 = (-math.pi / 180.0) * 7.0  * knee_fourier(t)

        send_cmd(ser, 1, th1)
        send_cmd(ser, 2, th2)
        send_cmd(ser, 3, th3)

        if step % 100 == 0:
            print(f"  t={t:6.2f}s  "
                  f"Rad/Uln={th1:+.4f} rad  "
                  f"Flex/Ext={th2:+.4f} rad  "
                  f"Pro/Sup={th3:+.4f} rad")

        next_t = t0 + (step + 1) * DT_S
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

    print("\nCommands sent!")
    ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send wrist trajectories to ESP32")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port for ESP32 (default: /dev/ttyUSB0)")
    parser.add_argument("--cycles", type=int, default=5,
                        help="Number of full trajectory cycles (default: 5)")
    args = parser.parse_args()
    run(args.port, args.cycles)
