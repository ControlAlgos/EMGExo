"""
EMG Exo Controller — full pipeline from EMG signals to motor actuation.

Ties together the EMG acquisition/processing module and the motor
controller module into a single real-time control loop.

Usage:
    from integration.emg_exo_controller import EMGExoController

    controller = EMGExoController()
    controller.run(duration=10.0)  # run for 10 seconds
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from emg.emg_sensor import EMGSensor
from emg.emg_processor import EMGProcessor
from motor.motor_controller import MotorController, MotorCommand, MotorFeedback


@dataclass
class ChannelMapping:
    """Maps an EMG channel to a motor, defining how envelope scales to motor command."""
    emg_channel: int
    motor_id: int
    position_range: tuple[float, float] = (0.0, 1.0)  # (min_rad, max_rad)
    kp: float = 100.0
    kd: float = 2.0


@dataclass
class LoopStats:
    """Timing statistics from the control loop."""
    loop_count: int = 0
    total_time_ms: float = 0.0
    min_loop_ms: float = float('inf')
    max_loop_ms: float = 0.0
    avg_loop_ms: float = 0.0

    def update(self, dt_ms: float):
        self.loop_count += 1
        self.total_time_ms += dt_ms
        self.min_loop_ms = min(self.min_loop_ms, dt_ms)
        self.max_loop_ms = max(self.max_loop_ms, dt_ms)
        self.avg_loop_ms = self.total_time_ms / self.loop_count

    def __str__(self) -> str:
        return (
            f"Loops: {self.loop_count} | "
            f"Avg: {self.avg_loop_ms:.2f}ms | "
            f"Min: {self.min_loop_ms:.2f}ms | "
            f"Max: {self.max_loop_ms:.2f}ms"
        )


class EMGExoController:
    """
    Real-time EMG-to-motor control loop.

    Reads EMG signals, computes activation envelopes, maps them to
    motor commands, and sends them to the ESP32 via SPI.
    """

    def __init__(
        self,
        emg_sensor: Optional[EMGSensor] = None,
        emg_processor: Optional[EMGProcessor] = None,
        motor_controller: Optional[MotorController] = None,
        channel_mappings: Optional[list[ChannelMapping]] = None,
        loop_rate_hz: float = 500.0,
    ):
        """
        All dependencies are injectable for testability. If not provided,
        defaults are created.
        """
        self._emg = emg_sensor or EMGSensor(spi_bus=1, spi_device=0, channels=[0, 1, 2])
        self._proc = emg_processor or EMGProcessor.from_time_constant(
            num_channels=3, time_constant_ms=3.0, sample_rate=loop_rate_hz
        )
        self._motor = motor_controller or MotorController(spi_bus=0, spi_device=0)
        self._mappings = channel_mappings or [
            ChannelMapping(emg_channel=0, motor_id=1, position_range=(0.0, 1.5)),
            ChannelMapping(emg_channel=1, motor_id=2, position_range=(0.0, 1.5)),
            ChannelMapping(emg_channel=2, motor_id=3, position_range=(0.0, 1.5)),
        ]
        self._loop_period = 1.0 / loop_rate_hz
        self._running = False
        self._stats = LoopStats()
        self._on_loop_callback: Optional[Callable] = None

    @property
    def stats(self) -> LoopStats:
        return self._stats

    def set_loop_callback(self, callback: Callable[[dict, list[MotorCommand], list[MotorFeedback]], None]):
        """
        Register a callback invoked each loop iteration for logging/plotting.

        Signature: callback(envelopes, commands, feedback)
        """
        self._on_loop_callback = callback

    def _envelope_to_commands(self, envelopes: dict[int, float]) -> list[MotorCommand]:
        """Map EMG envelopes to motor commands using channel mappings."""
        commands = []
        for m in self._mappings:
            activation = envelopes.get(m.emg_channel, 0.0)
            pos_min, pos_max = m.position_range
            position = pos_min + activation * (pos_max - pos_min)
            commands.append(MotorCommand(
                motor_id=m.motor_id,
                position=position,
                velocity=0.0,
                kp=m.kp,
                kd=m.kd,
                torque=0.0,
            ))
        return commands

    def run(self, duration: float = 10.0):
        """
        Run the control loop for the specified duration in seconds.

        The loop reads EMG, processes envelopes, maps to motor commands,
        and sends them to the ESP32 at the configured rate.
        """
        self._running = True
        self._stats = LoopStats()

        self._motor.enter_control_mode()
        time.sleep(0.1)

        start = time.perf_counter()
        try:
            while self._running and (time.perf_counter() - start) < duration:
                t0 = time.perf_counter()

                raw = self._emg.read_all()
                envelopes = self._proc.update(raw)
                commands = self._envelope_to_commands(envelopes)
                feedback = self._motor.send_commands(commands)

                dt_ms = (time.perf_counter() - t0) * 1000.0
                self._stats.update(dt_ms)

                if self._on_loop_callback:
                    self._on_loop_callback(envelopes, commands, feedback)

                # Sleep for remainder of loop period
                elapsed = time.perf_counter() - t0
                sleep_time = self._loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            self._motor.exit_control_mode()

        print(f"Control loop finished. {self._stats}")

    def stop(self):
        """Signal the control loop to stop (call from another thread)."""
        self._running = False
