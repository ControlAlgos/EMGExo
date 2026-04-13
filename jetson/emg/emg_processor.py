"""
EMG Processor — real-time envelope detection using exponential moving average.

Converts raw EMG signals into smooth activation envelopes suitable for
proportional motor control. Uses a causal EMA filter for minimal latency
(~1-3ms effective delay depending on alpha).

This module is pure computation with no hardware dependencies, so it
can be tested on any machine with sample data.

Usage:
    from emg.emg_processor import EMGProcessor

    proc = EMGProcessor(num_channels=3, alpha=0.1, sample_rate=1000)

    # Feed samples one at a time (real-time loop)
    envelope = proc.update({0: 512, 1: 340, 2: 780})
    # envelope = {0: 0.23, 1: 0.15, 2: 0.42}  (normalized 0–1)
"""

from __future__ import annotations

import math
from typing import Optional


class EMGProcessor:
    """
    Real-time EMG envelope detection with configurable smoothing.

    The processing pipeline per sample:
      1. Normalize raw ADC value to 0–1 range
      2. Remove DC offset (high-pass via running mean subtraction)
      3. Full-wave rectify (abs)
      4. Exponential moving average (low-pass envelope)
      5. Optional: scale to 0–1 output range with configurable gain
    """

    def __init__(
        self,
        num_channels: int = 1,
        alpha: float = 0.1,
        sample_rate: float = 1000.0,
        adc_max: int = 1023,
        dc_alpha: float = 0.001,
        gain: float = 2.0,
    ):
        """
        Args:
            num_channels: Number of EMG channels.
            alpha: EMA smoothing factor for envelope (0–1). Higher = faster
                   response but noisier. Effective time constant ~= 1/(alpha * sample_rate).
                   At alpha=0.1, sample_rate=1000: ~1ms time constant.
            sample_rate: Expected sample rate in Hz (for time constant reference only).
            adc_max: Maximum raw ADC value (1023 for 10-bit MCP3008).
            dc_alpha: EMA factor for DC offset tracking (very slow, ~0.001).
            gain: Output gain multiplier. Scales the envelope so typical
                  activations reach ~1.0. Adjust based on your MyoWare gain setting.
        """
        self._alpha = alpha
        self._dc_alpha = dc_alpha
        self._adc_max = adc_max
        self._gain = gain
        self._sample_rate = sample_rate

        self._envelope: dict[int, float] = {}
        self._dc_offset: dict[int, float] = {}
        self._channels = list(range(num_channels))

    @classmethod
    def from_time_constant(
        cls,
        num_channels: int = 1,
        time_constant_ms: float = 3.0,
        sample_rate: float = 1000.0,
        **kwargs,
    ) -> "EMGProcessor":
        """
        Create a processor with a specific time constant in milliseconds.

        This is more intuitive than setting alpha directly.
        A 3ms time constant gives ~3ms effective latency.
        """
        dt = 1.0 / sample_rate
        tc = time_constant_ms / 1000.0
        alpha = 1.0 - math.exp(-dt / tc)
        return cls(num_channels=num_channels, alpha=alpha, sample_rate=sample_rate, **kwargs)

    def reset(self):
        """Reset all internal state."""
        self._envelope.clear()
        self._dc_offset.clear()

    @property
    def effective_latency_ms(self) -> float:
        """Approximate effective latency of the envelope filter."""
        return 1.0 / (self._alpha * self._sample_rate) * 1000.0

    def update(self, raw_readings: dict[int, int]) -> dict[int, float]:
        """
        Process one set of raw ADC readings and return envelopes.

        Args:
            raw_readings: {channel: raw_adc_value} from EMGSensor.read_all()

        Returns:
            {channel: envelope_value} normalized roughly to 0–1.
        """
        result = {}
        for ch, raw in raw_readings.items():
            normalized = raw / self._adc_max

            # Track and subtract DC offset
            if ch not in self._dc_offset:
                self._dc_offset[ch] = normalized
            self._dc_offset[ch] += self._dc_alpha * (normalized - self._dc_offset[ch])
            centered = normalized - self._dc_offset[ch]

            rectified = abs(centered)

            # EMA envelope
            if ch not in self._envelope:
                self._envelope[ch] = rectified
            self._envelope[ch] += self._alpha * (rectified - self._envelope[ch])

            result[ch] = min(1.0, self._envelope[ch] * self._gain)

        return result

    def update_batch(self, raw_batch: list[dict[int, int]]) -> list[dict[int, float]]:
        """Process a batch of readings (useful for offline testing with recorded data)."""
        return [self.update(reading) for reading in raw_batch]
