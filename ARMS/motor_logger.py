#!/usr/bin/env python3
"""
Read motor feedback from ESP32 debug serial (FTDI /dev/ttyUSB0).
Firmware outputs "%.4f;" per command in round-robin order (M1, M2, M3, M1, ...).
Logs to motor_log.csv. Run BEFORE starting the trajectory.
Press Ctrl+C to stop.
"""
import serial
import time
import csv

PORT = "/dev/ttyUSB0"
BAUD = 115200
NUM_MOTORS = 3

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()

    csv_path = "motor_log.csv"
    print(f"Logging motor feedback from {PORT} -> {csv_path}")
    print(f"Expecting {NUM_MOTORS} motors in round-robin order.")
    print("Press Ctrl+C to stop.\n")

    t0 = time.monotonic()
    count = 0
    motor_idx = 0
    buf = ""

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "motor_id", "position_rad"])

        try:
            while True:
                raw = ser.read(256)
                if not raw:
                    continue
                buf += raw.decode("utf-8", errors="replace")

                while ";" in buf:
                    val_str, buf = buf.split(";", 1)
                    val_str = val_str.strip()
                    if not val_str:
                        continue

                    # Skip startup messages like "1 in control"
                    if "control" in val_str or not val_str.replace(".", "").replace("-", "").isdigit():
                        continue

                    try:
                        pos = float(val_str)
                    except ValueError:
                        continue

                    mid = (motor_idx % NUM_MOTORS) + 1
                    motor_idx += 1
                    t = time.monotonic() - t0

                    writer.writerow([f"{t:.4f}", mid, f"{pos:.4f}"])
                    count += 1

                    if count % 150 == 0:
                        f.flush()
                        print(f"  {count} samples | t={t:.1f}s | M{mid} pos={pos:.4f}")

        except KeyboardInterrupt:
            pass

    per_motor = {}
    print(f"\nSaved {count} samples to {csv_path}")
    for m in range(1, NUM_MOTORS + 1):
        n = count // NUM_MOTORS + (1 if (count % NUM_MOTORS) >= m else 0)
        print(f"  Motor {m}: ~{count // NUM_MOTORS} samples")


if __name__ == "__main__":
    main()
