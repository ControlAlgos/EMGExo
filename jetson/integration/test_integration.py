#!/usr/bin/env python3
"""
Integration test — full EMG-to-motor pipeline.

Requires all hardware connected: MyoWare + MCP3008 on SPI1,
ESP32 on SPI0, motors on CAN bus.

Usage:
    python -m integration.test_integration
    python -m integration.test_integration --duration 10 --rate 500
"""

import argparse
import sys
import time

sys.path.insert(0, "..")

from emg.emg_sensor import EMGSensor
from emg.emg_processor import EMGProcessor
from motor.motor_controller import MotorController, MotorCommand, MotorFeedback
from integration.emg_exo_controller import EMGExoController, ChannelMapping


def logging_callback(
    envelopes: dict[int, float],
    commands: list[MotorCommand],
    feedback: list[MotorFeedback],
    state: dict,
):
    state["count"] = state.get("count", 0) + 1
    if state["count"] % state.get("print_interval", 100) == 0:
        env_str = " ".join(f"E{k}={v:.3f}" for k, v in sorted(envelopes.items()))
        cmd_str = " ".join(f"M{c.motor_id}={c.position:+.3f}" for c in commands)
        fb_str = " ".join(f"F{f.motor_id}={f.position:+.3f}" for f in feedback)
        elapsed = time.perf_counter() - state["start"]
        print(f"[{elapsed:6.2f}s] {env_str} | {cmd_str} | {fb_str}")


def main():
    parser = argparse.ArgumentParser(description="Full EMG-to-motor integration test")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=500.0, help="Control loop rate (Hz)")
    parser.add_argument("--emg-alpha", type=float, default=0.1)
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--kd", type=float, default=2.0)
    parser.add_argument("--max-pos", type=float, default=1.5, help="Max motor position (rad)")
    args = parser.parse_args()

    print("=" * 70)
    print("EMGExo Integration Test")
    print(f"Rate: {args.rate}Hz | Duration: {args.duration}s")
    print(f"EMG alpha: {args.emg_alpha} | Kp: {args.kp} | Kd: {args.kd}")
    print(f"Max position: {args.max_pos} rad")
    print("=" * 70)

    emg_sensor = EMGSensor(spi_bus=1, spi_device=0, channels=[0, 1, 2])
    emg_proc = EMGProcessor(num_channels=3, alpha=args.emg_alpha, sample_rate=args.rate)
    motor_ctrl = MotorController(spi_bus=0, spi_device=0)

    mappings = [
        ChannelMapping(emg_channel=0, motor_id=1, position_range=(0, args.max_pos), kp=args.kp, kd=args.kd),
        ChannelMapping(emg_channel=1, motor_id=2, position_range=(0, args.max_pos), kp=args.kp, kd=args.kd),
        ChannelMapping(emg_channel=2, motor_id=3, position_range=(0, args.max_pos), kp=args.kp, kd=args.kd),
    ]

    controller = EMGExoController(
        emg_sensor=emg_sensor,
        emg_processor=emg_proc,
        motor_controller=motor_ctrl,
        channel_mappings=mappings,
        loop_rate_hz=args.rate,
    )

    cb_state = {"start": time.perf_counter(), "print_interval": int(args.rate / 5)}
    controller.set_loop_callback(lambda e, c, f: logging_callback(e, c, f, cb_state))

    print("\nStarting control loop... (Ctrl+C to stop)\n")
    try:
        controller.run(duration=args.duration)
    except KeyboardInterrupt:
        controller.stop()
        print("\nStopped by user.")

    print(f"\nFinal stats: {controller.stats}")


if __name__ == "__main__":
    main()
