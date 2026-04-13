#!/usr/bin/env python3
"""
Standalone EMG test — run on Jetson with MyoWare + MCP3008 connected.
No motors or ESP32 needed.

Reads EMG signals from 3 channels, processes them through the envelope
detector, and prints real-time activation levels.

Usage:
    python -m emg.test_emg
    python -m emg.test_emg --channels 0 1 --rate 500 --duration 10

Wiring (MCP3008 on SPI1):
    Jetson SPI1_MOSI (pin 37) -> MCP3008 DIN
    Jetson SPI1_MISO (pin 22) -> MCP3008 DOUT
    Jetson SPI1_SCLK (pin 13) -> MCP3008 CLK
    Jetson SPI1_CS0  (pin 18) -> MCP3008 CS
    MCP3008 VDD, VREF -> 3.3V
    MCP3008 AGND, DGND -> GND
    MyoWare SIG -> MCP3008 CH0 (and CH1, CH2 for additional sensors)
"""

import argparse
import sys
import time

sys.path.insert(0, "..")

from emg.emg_sensor import EMGSensor
from emg.emg_processor import EMGProcessor


def main():
    parser = argparse.ArgumentParser(description="Standalone EMG sensor test")
    parser.add_argument("--spi-bus", type=int, default=1, help="SPI bus for MCP3008")
    parser.add_argument("--spi-device", type=int, default=0, help="SPI device (CS)")
    parser.add_argument("--channels", type=int, nargs="+", default=[0, 1, 2], help="ADC channels")
    parser.add_argument("--rate", type=float, default=500, help="Sample rate (Hz)")
    parser.add_argument("--duration", type=float, default=5.0, help="Test duration (seconds)")
    parser.add_argument("--alpha", type=float, default=0.1, help="EMA smoothing factor")
    args = parser.parse_args()

    dt = 1.0 / args.rate

    print(f"EMG Test: channels={args.channels}, rate={args.rate}Hz, duration={args.duration}s")
    print(f"EMA alpha={args.alpha}, effective latency ~{1.0/(args.alpha * args.rate)*1000:.1f}ms")
    print("-" * 60)

    sensor = EMGSensor(
        spi_bus=args.spi_bus,
        spi_device=args.spi_device,
        channels=args.channels,
    )
    processor = EMGProcessor(
        num_channels=len(args.channels),
        alpha=args.alpha,
        sample_rate=args.rate,
    )

    sample_count = 0
    start = time.perf_counter()

    try:
        while (time.perf_counter() - start) < args.duration:
            t0 = time.perf_counter()

            raw = sensor.read_all()
            envelopes = processor.update(raw)

            sample_count += 1
            if sample_count % int(args.rate / 4) == 0:  # print 4x/sec
                bar = " | ".join(
                    f"CH{ch}: {envelopes.get(ch, 0):.3f} {'#' * int(envelopes.get(ch, 0) * 30)}"
                    for ch in args.channels
                )
                elapsed = time.perf_counter() - start
                print(f"[{elapsed:6.2f}s] {bar}")

            elapsed = time.perf_counter() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sensor.close()

    total = time.perf_counter() - start
    print(f"\n{sample_count} samples in {total:.2f}s ({sample_count/total:.0f} Hz actual)")


if __name__ == "__main__":
    main()
