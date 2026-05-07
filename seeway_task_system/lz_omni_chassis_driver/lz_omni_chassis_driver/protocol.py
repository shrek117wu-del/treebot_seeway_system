"""Protocol definitions and frame parsing for the LZ_OMNI chassis."""

import struct
from dataclasses import dataclass
from typing import Generator, Optional, Tuple

FRAME_HEADER0 = 0x0A
FRAME_HEADER1 = 0x0C

CMD_MOTOR_CONTROL = 0x01
CMD_MOTION_OMNI = 0x40
CMD_VERSION_QUERY = 0x11

CMD_BATTERY_INFO = 0x69
CMD_CHASSIS_STATUS = 0x6B
CMD_AUX_INFO = 0x6F

MAX_BUFFER_SIZE = 256
HEADER_SIZE = 2
LENGTH_FIELD_SIZE = 1
CMD_FIELD_SIZE = 1
CHECKSUM_SIZE = 1


def compute_xor(data: bytes) -> int:
    """Compute XOR checksum over all bytes in *data*."""
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def _clamp(value: int, minimum: int, maximum: int) -> int:
    """Clamp an integer into the inclusive range [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def build_frame(cmd: int, data: bytes) -> bytes:
    """Build a complete UART frame with headers, payload length, and XOR checksum."""
    payload = bytes([len(data), cmd]) + data
    checksum = compute_xor(payload)
    return bytes([FRAME_HEADER0, FRAME_HEADER1]) + payload + bytes([checksum])


def build_motor_control_frame(m1: int, m2: int, m3: int, m4: int) -> bytes:
    """Build CMD 0x01 motor RPM control frame."""
    motor_data = struct.pack(
        '>hhhh',
        _clamp(m1, -3000, 3000),
        _clamp(m2, -3000, 3000),
        _clamp(m3, -3000, 3000),
        _clamp(m4, -3000, 3000),
    )
    return build_frame(CMD_MOTOR_CONTROL, motor_data)


def build_omni_control_frame(vx: int, vy: int, vw: int) -> bytes:
    """Build CMD 0x40 omnidirectional velocity control frame."""
    motion_data = struct.pack(
        '>hhh',
        _clamp(vx, -2000, 2000),
        _clamp(vy, -2000, 2000),
        _clamp(vw, -6870, 6870),
    )
    return build_frame(CMD_MOTION_OMNI, motion_data)


def build_version_query_frame() -> bytes:
    """Build CMD 0x11 firmware version query frame."""
    return build_frame(CMD_VERSION_QUERY, b'')


@dataclass
class BatteryInfo:
    """Battery telemetry parsed from CMD 0x69 payload.

    ``soc_percent`` is normalized to the protocol's percentage range [0, 100].
    """

    voltage_v: float
    current_a: float
    soc_percent: int
    status: int

    @classmethod
    def from_bytes(cls, data: bytes) -> 'BatteryInfo':
        """Parse BatteryInfo from a 6-byte payload."""
        if len(data) != 6:
            raise ValueError(f'BatteryInfo expects 6 bytes, got {len(data)}')
        voltage_raw, current_raw, soc, status = struct.unpack('>HhBB', data)
        return cls(
            voltage_v=voltage_raw / 100.0,
            current_a=current_raw / 100.0,
            soc_percent=int(soc),
            status=int(status),
        )


@dataclass
class ChassisStatus:
    """Chassis motion and wheel speed telemetry parsed from CMD 0x6B payload."""

    vx_mms: int
    vy_mms: int
    vw_mrad_s: int
    m1_rpm: Optional[int] = None
    m2_rpm: Optional[int] = None
    m3_rpm: Optional[int] = None
    m4_rpm: Optional[int] = None

    @property
    def vx_ms(self) -> float:
        """Linear X speed in m/s."""
        return self.vx_mms / 1000.0

    @property
    def vy_ms(self) -> float:
        """Linear Y speed in m/s."""
        return self.vy_mms / 1000.0

    @property
    def vw_rads(self) -> float:
        """Angular Z speed in rad/s."""
        return self.vw_mrad_s / 1000.0

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ChassisStatus':
        """Parse ChassisStatus from 14-byte full payload or 6-byte fallback payload."""
        if len(data) == 14:
            vx, vy, vw, m1, m2, m3, m4 = struct.unpack('>hhhhhhh', data)
            return cls(vx_mms=vx, vy_mms=vy, vw_mrad_s=vw, m1_rpm=m1, m2_rpm=m2, m3_rpm=m3, m4_rpm=m4)
        if len(data) == 6:
            vx, vy, vw = struct.unpack('>hhh', data)
            return cls(vx_mms=vx, vy_mms=vy, vw_mrad_s=vw)
        raise ValueError(f'ChassisStatus expects 14 or 6 bytes, got {len(data)}')


@dataclass
class AuxInfo:
    """Auxiliary telemetry parsed from CMD 0x6F payload."""

    error_code: int
    temperature_c: float
    uptime_ms: int

    @classmethod
    def from_bytes(cls, data: bytes) -> 'AuxInfo':
        """Parse AuxInfo from an 8-byte payload."""
        if len(data) != 8:
            raise ValueError(f'AuxInfo expects 8 bytes, got {len(data)}')
        error_code, temperature_raw, uptime_ms = struct.unpack('>HhI', data)
        return cls(error_code=int(error_code), temperature_c=temperature_raw / 10.0, uptime_ms=int(uptime_ms))


@dataclass
class VersionInfo:
    """Firmware version information parsed from CMD 0x11 response payload."""

    major: int
    minor: int
    patch: int
    build: int

    @classmethod
    def from_bytes(cls, data: bytes) -> 'VersionInfo':
        """Parse VersionInfo from a 4-byte payload."""
        if len(data) != 4:
            raise ValueError(f'VersionInfo expects 4 bytes, got {len(data)}')
        major, minor, patch, build = struct.unpack('>BBBB', data)
        return cls(major=int(major), minor=int(minor), patch=int(patch), build=int(build))

    def __str__(self) -> str:
        """Return a friendly firmware version string."""
        return f'v{self.major}.{self.minor}.{self.patch} build {self.build}'


class FrameParser:
    """Incremental parser for UART protocol frames.

    The parser is stream-oriented and accepts arbitrary input chunking.
    Feed bytes using :meth:`feed` and iterate over yielded ``(cmd, payload)`` tuples.
    """

    def __init__(self) -> None:
        """Initialise parser with an empty internal receive buffer."""
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Generator[Tuple[int, bytes], None, None]:
        """Consume incoming bytes and yield parsed ``(cmd, payload)`` frames."""
        if not data:
            return

        self._buffer.extend(data)
        if len(self._buffer) > MAX_BUFFER_SIZE:
            del self._buffer[: len(self._buffer) - MAX_BUFFER_SIZE]

        while True:
            if len(self._buffer) < 2:
                return

            header_index = self._find_header()
            if header_index < 0:
                self._buffer.clear()
                return
            if header_index > 0:
                del self._buffer[:header_index]

            minimum_frame_len = HEADER_SIZE + LENGTH_FIELD_SIZE + CMD_FIELD_SIZE + CHECKSUM_SIZE
            if len(self._buffer) < minimum_frame_len:
                return

            data_len = self._buffer[2]
            frame_len = (
                HEADER_SIZE
                + LENGTH_FIELD_SIZE
                + CMD_FIELD_SIZE
                + data_len
                + CHECKSUM_SIZE
            )

            if frame_len > MAX_BUFFER_SIZE:
                del self._buffer[0]
                continue

            if len(self._buffer) < frame_len:
                return

            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]

            expected = compute_xor(frame[2:-1])
            actual = frame[-1]
            if expected != actual:
                continue

            cmd = frame[3]
            payload = frame[4:-1]
            yield cmd, payload

    def _find_header(self) -> int:
        """Find header index in internal buffer, or ``-1`` if not found."""
        for index in range(len(self._buffer) - 1):
            if self._buffer[index] == FRAME_HEADER0 and self._buffer[index + 1] == FRAME_HEADER1:
                return index
        return -1
