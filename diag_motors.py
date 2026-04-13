#!/usr/bin/env python3
"""
Diagnostic: reset ESP32, verify control-mode entry, test each motor individually.
Reads debug output from FTDI while sending commands on CP2102.
"""

import serial
import time
import threading
import sys

CMD_PORT   = "/dev/ttyUSB2"   # CP2102 → ESP32 Serial (commands)
DEBUG_PORT = "/dev/ttyUSB0"   # FTDI   → ESP32 mySerial2 (debug feedback)
BAUD       = 115200

stop_flag = False

def debug_reader(debug_ser):
    """Background thread: print everything from the debug serial."""
    while not stop_flag:
        try:
            line = debug_ser.readline()
            if line:
                text = line.decode(errors="replace").rstrip()
                print(f"  [DEBUG] {text}")
        except Exception:
            pass


def send_cmd(ser, motor_id, theta_rad):
    theta_int = int(theta_rad * 1000)
    ser.write(f"{motor_id}\n".encode())
    ser.write(f"{theta_int}\n".encode())


def main():
    global stop_flag

    if len(sys.argv) >= 3:
        cmd_port, dbg_port = sys.argv[1], sys.argv[2]
    else:
        cmd_port, dbg_port = CMD_PORT, DEBUG_PORT

    print(f"Command port : {cmd_port}")
    print(f"Debug port   : {dbg_port}")

    # Open debug port first (won't reset ESP32)
    debug_ser = serial.Serial(dbg_port, BAUD, timeout=0.5)
    debug_ser.reset_input_buffer()

    # Start debug reader thread
    t = threading.Thread(target=debug_reader, args=(debug_ser,), daemon=True)
    t.start()

    # Open command port (DTR toggles → ESP32 resets → setup() runs)
    print("\nOpening command port (ESP32 will reset) …")
    cmd_ser = serial.Serial(cmd_port, BAUD, timeout=0.1)

    print("Waiting 7s for setup() — watch for 'X in control' messages …\n")
    time.sleep(7)
    cmd_ser.reset_input_buffer()

    # ── Test each motor individually ────────────────────────────────
    for mid in [1, 2, 3]:
        print(f"\n{'='*50}")
        print(f"Testing Motor {mid}: sending pos=0.0 (3 times, 100ms apart)")
        print(f"{'='*50}")
        for _ in range(3):
            send_cmd(cmd_ser, mid, 0.0)
            time.sleep(0.1)
        time.sleep(0.5)  # wait for debug feedback

    # ── Send a small motion to each motor ───────────────────────────
    print(f"\n{'='*50}")
    print("Sending small position (0.1 rad) to each motor, one at a time")
    print(f"{'='*50}")
    for mid in [1, 2, 3]:
        print(f"\n  → Motor {mid}: pos = 0.100 rad")
        for _ in range(10):
            send_cmd(cmd_ser, mid, 0.1)
            time.sleep(0.05)
        time.sleep(1.0)  # pause between motors to see which one moved

    # ── Send interleaved commands (like normal operation) ───────────
    print(f"\n{'='*50}")
    print("Sending interleaved: M1, M2, M3 at 10ms intervals (2 seconds)")
    print(f"{'='*50}")
    t0 = time.perf_counter()
    step = 0
    while time.perf_counter() - t0 < 2.0:
        send_cmd(cmd_ser, 1, 0.05)
        send_cmd(cmd_ser, 2, 0.05)
        send_cmd(cmd_ser, 3, 0.05)
        step += 1
        next_t = t0 + step * 0.01
        wait = next_t - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
    print(f"  Sent {step} interleaved batches")

    time.sleep(1.0)

    # ── Return all to zero ──────────────────────────────────────────
    print(f"\n{'='*50}")
    print("Returning all motors to 0.0")
    print(f"{'='*50}")
    for _ in range(20):
        for mid in [1, 2, 3]:
            send_cmd(cmd_ser, mid, 0.0)
        time.sleep(0.01)
    time.sleep(1.0)

    stop_flag = True
    cmd_ser.close()
    debug_ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
