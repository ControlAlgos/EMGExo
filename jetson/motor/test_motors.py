#!/usr/bin/env python3
"""
Standalone motor test — run on Jetson with ESP32 connected via SPI.
No EMG hardware needed.

Sends sinusoidal position commands to each motor and prints feedback.

Usage:
    python -m motor.test_motors
    python -m motor.test_motors --amplitude 0.5 --frequency 0.5 --duration 5

Wiring (ESP32 on SPI0):
    Jetson SPI0_MOSI (pin 19) -> ESP32 GPIO 23
    Jetson SPI0_MISO (pin 21) -> ESP32 GPIO 19
    Jetson SPI0_SCLK (pin 23) -> ESP32 GPIO 18
    Jetson SPI0_CS0  (pin 24) -> ESP32 GPIO 5
    Common GND
"""

import argparse
import math
import sys
import time

sys.path.insert(0, "..")

from motor.motor_controller import MotorController, MotorCommand


def main():
    parser = argparse.ArgumentParser(description="Standalone motor control test")
    parser.add_argument("--spi-bus", type=int, default=0)
    parser.add_argument("--spi-device", type=int, default=0)
    parser.add_argument("--speed-hz", type=int, default=8_000_000)
    parser.add_argument("--amplitude", type=float, default=0.3, help="Sinusoid amplitude (rad)")
    parser.add_argument("--frequency", type=float, default=0.5, help="Sinusoid frequency (Hz)")
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--kd", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=500, help="Command rate (Hz)")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--motors", type=int, nargs="+", default=[1, 2, 3], help="Motor IDs")
    args = parser.parse_args()

    dt = 1.0 / args.rate

    print(f"Motor Test: IDs={args.motors}, amp={args.amplitude}rad, freq={args.frequency}Hz")
    print(f"Kp={args.kp}, Kd={args.kd}, rate={args.rate}Hz, duration={args.duration}s")
    print("-" * 70)

    ctrl = MotorController(
        spi_bus=args.spi_bus,
        spi_device=args.spi_device,
        speed_hz=args.speed_hz,
    )

    print("Entering control mode...")
    ctrl.enter_control_mode()
    time.sleep(0.5)
    print("Control mode active. Starting trajectory.\n")

    step = 0
    start = time.perf_counter()

    try:
        while (time.perf_counter() - start) < args.duration:
            t0 = time.perf_counter()
            t = time.perf_counter() - start

            theta = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

            commands = [
                MotorCommand(
                    motor_id=mid,
                    position=theta,
                    velocity=0.0,
                    kp=args.kp,
                    kd=args.kd,
                    torque=0.0,
                )
                for mid in args.motors
            ]

            feedback = ctrl.send_commands(commands)

            step += 1
            if step % int(args.rate / 5) == 0:  # print 5x/sec
                fb_str = " | ".join(
                    f"M{fb.motor_id}: cmd={theta:+.3f} enc={fb.position:+.3f}"
                    for fb in feedback
                )
                loop_ms = (time.perf_counter() - t0) * 1000
                print(f"[{t:5.2f}s] {fb_str}  ({loop_ms:.1f}ms)")

            elapsed = time.perf_counter() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        print("\nExiting control mode...")
        ctrl.exit_control_mode()
        ctrl.close()

    total = time.perf_counter() - start
    print(f"Done. {step} commands in {total:.2f}s ({step/total:.0f} Hz actual)")


if __name__ == "__main__":
    main()
