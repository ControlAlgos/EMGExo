"""
EMG Sensor — reads analog MyoWare signals via MCP3008 SPI ADC.

The MCP3008 is a 10-bit, 8-channel SPI ADC. Each read returns a
value 0–1023 corresponding to 0–3.3V.

This module can be tested independently without any motors connected.

Usage:
    from emg.emg_sensor import EMGSensor

    sensor = EMGSensor(spi_bus=1, spi_device=0, channels=[0, 1, 2])
    while True:
        readings = sensor.read_all()
        # readings = {0: 512, 1: 340, 2: 780}
"""

from __future__ import annotations

from typing import Optional

try:
    import spidev
except ImportError:
    spidev = None


class EMGSensor:
    """Reads analog EMG signals from MyoWare sensors via MCP3008 ADC."""

    MCP3008_VREF = 3.3  # reference voltage
    MCP3008_RESOLUTION = 1024  # 10-bit ADC

    def __init__(
        self,
        spi_bus: int = 1,
        spi_device: int = 0,
        speed_hz: int = 1_000_000,
        channels: Optional[list[int]] = None,
        vref: float = 3.3,
    ):
        """
        Args:
            spi_bus: SPI bus number (use 1 to keep bus 0 for ESP32).
            spi_device: SPI chip-select device number.
            speed_hz: SPI clock speed. MCP3008 supports up to 3.6 MHz @ 5V,
                      1.35 MHz @ 2.7V. 1 MHz is safe for 3.3V operation.
            channels: List of ADC channels to read (0–7). Defaults to [0].
            vref: ADC reference voltage in volts.
        """
        if spidev is None:
            raise RuntimeError(
                "spidev not available. Install with: pip install spidev\n"
                "This module requires a Linux system with SPI enabled."
            )
        self._spi = spidev.SpiDev()
        self._spi.open(spi_bus, spi_device)
        self._spi.max_speed_hz = speed_hz
        self._spi.mode = 0
        self._channels = channels or [0]
        self._vref = vref

    def close(self):
        if self._spi:
            self._spi.close()
            self._spi = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def read_raw(self, channel: int) -> int:
        """Read a single MCP3008 channel. Returns raw 10-bit value (0–1023)."""
        if channel < 0 or channel > 7:
            raise ValueError(f"MCP3008 channel must be 0–7, got {channel}")
        # MCP3008 SPI protocol: send start bit, single-ended mode, channel bits
        cmd = [0x01, (0x80 | (channel << 4)), 0x00]
        result = self._spi.xfer2(cmd)
        return ((result[1] & 0x03) << 8) | result[2]

    def read_voltage(self, channel: int) -> float:
        """Read a single channel as voltage."""
        raw = self.read_raw(channel)
        return raw * self._vref / self.MCP3008_RESOLUTION

    def read_all(self) -> dict[int, int]:
        """Read all configured channels. Returns {channel: raw_value}."""
        return {ch: self.read_raw(ch) for ch in self._channels}

    def read_all_voltage(self) -> dict[int, float]:
        """Read all configured channels as voltages."""
        return {ch: self.read_voltage(ch) for ch in self._channels}
