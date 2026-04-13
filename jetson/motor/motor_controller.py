"""
Motor Controller — SPI master for communicating with ESP32.

This module can be used independently to test motor actuation
without any EMG input. The Jetson sends motor commands over SPI
and receives position/velocity/torque feedback.

Usage:
    from motor.motor_controller import MotorController, MotorCommand

    ctrl = MotorController(spi_bus=0, spi_device=0)
    ctrl.enter_control_mode()

    cmd = MotorCommand(motor_id=1, position=0.5, velocity=0, kp=100, kd=2, torque=0)
    feedback = ctrl.send_commands([cmd, cmd2, cmd3])
    print(feedback)

    ctrl.exit_control_mode()
    ctrl.close()
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import spidev
except ImportError:
    spidev = None


# ======================== AK80-9 Motor Limits ========================
P_MIN, P_MAX   = -12.5, 12.5      # radians
V_MIN, V_MAX   = -50.0, 50.0      # rad/s
T_MIN, T_MAX   = -18.0, 18.0      # Nm
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

# ======================== Protocol Constants =========================
PACKET_SIZE    = 67
CMD_HEADER     = bytes([0xAA, 0x55])
RSP_HEADER     = bytes([0x55, 0xAA])
CMD_MOTOR      = 0x00
CMD_ENTER_CTRL = 0x01
CMD_EXIT_CTRL  = 0x02
NUM_MOTORS     = 3
CMD_BLOCK_SIZE = 21   # id(1) + 5*float32(20)
FB_BLOCK_SIZE  = 13   # id(1) + 3*float32(12)


@dataclass
class MotorCommand:
    motor_id: int = 1
    position: float = 0.0   # radians
    velocity: float = 0.0   # rad/s
    kp: float = 100.0
    kd: float = 2.0
    torque: float = 0.0     # Nm

    def __post_init__(self):
        self.position = max(P_MIN, min(P_MAX, self.position))
        self.velocity = max(V_MIN, min(V_MAX, self.velocity))
        self.kp       = max(KP_MIN, min(KP_MAX, self.kp))
        self.kd       = max(KD_MIN, min(KD_MAX, self.kd))
        self.torque   = max(T_MIN, min(T_MAX, self.torque))


@dataclass
class MotorFeedback:
    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    valid: bool = False


def _compute_checksum(buf: bytes) -> int:
    cs = 0
    for b in buf:
        cs ^= b
    return cs & 0xFF


def _pack_command_packet(commands: list[MotorCommand], cmd_type: int = CMD_MOTOR) -> list[int]:
    """Pack up to 3 motor commands into a 67-byte SPI packet."""
    buf = bytearray(PACKET_SIZE)
    buf[0] = CMD_HEADER[0]
    buf[1] = CMD_HEADER[1]
    buf[2] = cmd_type

    for i, cmd in enumerate(commands[:NUM_MOTORS]):
        offset = 3 + i * CMD_BLOCK_SIZE
        buf[offset] = cmd.motor_id
        struct.pack_into('<f', buf, offset + 1,  cmd.position)
        struct.pack_into('<f', buf, offset + 5,  cmd.velocity)
        struct.pack_into('<f', buf, offset + 9,  cmd.kp)
        struct.pack_into('<f', buf, offset + 13, cmd.kd)
        struct.pack_into('<f', buf, offset + 17, cmd.torque)

    buf[-1] = _compute_checksum(buf[:-1])
    return list(buf)


def _unpack_response_packet(buf: list[int]) -> list[MotorFeedback]:
    """Unpack a 67-byte SPI response into motor feedback."""
    data = bytes(buf)

    if data[0] != RSP_HEADER[0] or data[1] != RSP_HEADER[1]:
        return [MotorFeedback() for _ in range(NUM_MOTORS)]

    expected_cs = _compute_checksum(data[:-1])
    if data[-1] != expected_cs:
        return [MotorFeedback() for _ in range(NUM_MOTORS)]

    feedback = []
    for i in range(NUM_MOTORS):
        offset = 3 + i * FB_BLOCK_SIZE
        motor_id = data[offset]
        pos  = struct.unpack_from('<f', data, offset + 1)[0]
        vel  = struct.unpack_from('<f', data, offset + 5)[0]
        torq = struct.unpack_from('<f', data, offset + 9)[0]
        feedback.append(MotorFeedback(
            motor_id=motor_id, position=pos, velocity=vel, torque=torq, valid=True
        ))
    return feedback


class MotorController:
    """SPI master interface to the ESP32 motor controller."""

    def __init__(self, spi_bus: int = 0, spi_device: int = 0, speed_hz: int = 8_000_000):
        if spidev is None:
            raise RuntimeError(
                "spidev not available. Install with: pip install spidev\n"
                "This module requires a Jetson/Linux system with SPI enabled."
            )
        self._spi = spidev.SpiDev()
        self._spi.open(spi_bus, spi_device)
        self._spi.max_speed_hz = speed_hz
        self._spi.mode = 0
        self._spi.bits_per_word = 8
        self._last_feedback: list[MotorFeedback] = [MotorFeedback() for _ in range(NUM_MOTORS)]

    def close(self):
        """Release the SPI device."""
        if self._spi:
            self._spi.close()
            self._spi = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _transfer(self, tx_data: list[int]) -> list[int]:
        """Full-duplex SPI transfer. Returns response from ESP32."""
        return self._spi.xfer2(tx_data)

    def enter_control_mode(self) -> list[MotorFeedback]:
        """Put all motors into MIT control mode."""
        dummy_cmds = [MotorCommand(motor_id=i+1) for i in range(NUM_MOTORS)]
        tx = _pack_command_packet(dummy_cmds, CMD_ENTER_CTRL)
        rx = self._transfer(tx)
        time.sleep(0.05)  # allow motors to initialize
        self._last_feedback = _unpack_response_packet(rx)
        return self._last_feedback

    def exit_control_mode(self) -> list[MotorFeedback]:
        """Take all motors out of control mode."""
        dummy_cmds = [MotorCommand(motor_id=i+1) for i in range(NUM_MOTORS)]
        tx = _pack_command_packet(dummy_cmds, CMD_EXIT_CTRL)
        rx = self._transfer(tx)
        self._last_feedback = _unpack_response_packet(rx)
        return self._last_feedback

    def send_commands(self, commands: list[MotorCommand]) -> list[MotorFeedback]:
        """
        Send position/velocity/torque commands to up to 3 motors.

        Returns feedback from the *previous* cycle (SPI is pipelined).
        The current cycle's feedback will be returned on the next call.
        """
        tx = _pack_command_packet(commands, CMD_MOTOR)
        rx = self._transfer(tx)
        self._last_feedback = _unpack_response_packet(rx)
        return self._last_feedback

    @property
    def last_feedback(self) -> list[MotorFeedback]:
        """Most recently received motor feedback."""
        return self._last_feedback
