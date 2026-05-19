"""Protocol helpers for JuxieDrive CANopen and custom CAN FD control."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

CanFrame = Tuple[int, bytes, bool]  # (arbitration_id, data, is_fd)

NMT_START_REMOTE_NODE = 0x01
SYNC_FRAME_ID = 0x080
SDO_TX_BASE = 0x600
SDO_RX_BASE = 0x580
HEARTBEAT_BASE = 0x700
CUSTOM_SINGLE_AXIS_BASE = 0x100
CUSTOM_MIT_BASE = 0x110
CUSTOM_MULTI_AXIS_ID = 0x200
CUSTOM_FEEDBACK_BASE = 0x300
TPDO1_BASE = 0x180
RPDO1_BASE = 0x200

CANOPEN_MODE_PP = 0x01
CANOPEN_MODE_PV = 0x02
CANOPEN_MODE_CSP = 0x03
CANOPEN_MODE_CSV = 0x04
CANOPEN_MODE_CURRENT = 0x05
CANOPEN_MODE_MIT = 0x06

IDX_VENDOR_NAME = 0x1008
IDX_DEVICE_MODEL = 0x1000
IDX_HARDWARE_VERSION = 0x1009
IDX_FIRMWARE_VERSION = 0x100A
IDX_HEARTBEAT_TIME = 0x1017
IDX_CONTROLWORD = 0x6040
IDX_STATUSWORD = 0x6041
IDX_MODE_OF_OPERATION = 0x6060
IDX_MODE_DISPLAY = 0x6061
IDX_POSITION_DEMAND = 0x6062
IDX_ACTUAL_POSITION = 0x6064
IDX_PROFILE_VELOCITY = 0x6081
IDX_ACCELERATION = 0x6083
IDX_DECELERATION = 0x6084
IDX_ACTUAL_SPEED = 0x606C
IDX_TARGET_POSITION = 0x607A
IDX_TARGET_CURRENT = 0x6071
IDX_ACTUAL_CURRENT = 0x6078
IDX_TARGET_VELOCITY = 0x60FF
IDX_SOFTWARE_LIMIT = 0x607D
IDX_ERROR_CODE = 0x603F
IDX_NODE_ID = 0x2530
IDX_ZERO_CALIBRATION = 0x2531
IDX_CURRENT_KP = 0x2532
IDX_CURRENT_KI = 0x2533
IDX_SPEED_KP = 0x2534
IDX_SPEED_KI = 0x2535
IDX_POSITION_KP = 0x2536
IDX_POSITION_KI = 0x2537
IDX_MAX_CURRENT = 0x2538
IDX_CANFD_DATA_BITRATE = 0x2540
IDX_DISABLE_WATCHDOG_LIMIT = 0x2650
IDX_MOS_TEMP = 0x2662
IDX_MOTOR_TEMP = 0x2663

PI_INDEX_MAP: Dict[str, int] = {
    'current_kp': IDX_CURRENT_KP,
    'current_ki': IDX_CURRENT_KI,
    'speed_kp': IDX_SPEED_KP,
    'speed_ki': IDX_SPEED_KI,
    'position_kp': IDX_POSITION_KP,
    'position_ki': IDX_POSITION_KI,
}
MODE_NAME_MAP: Dict[str, int] = {
    'pp': CANOPEN_MODE_PP,
    'profile_position': CANOPEN_MODE_PP,
    'pv': CANOPEN_MODE_PV,
    'profile_velocity': CANOPEN_MODE_PV,
    'csp': CANOPEN_MODE_CSP,
    'csv': CANOPEN_MODE_CSV,
    'current': CANOPEN_MODE_CURRENT,
    'current_loop': CANOPEN_MODE_CURRENT,
    'mit': CANOPEN_MODE_MIT,
}
MODE_CODE_MAP: Dict[int, str] = {
    CANOPEN_MODE_PP: 'profile_position',
    CANOPEN_MODE_PV: 'profile_velocity',
    CANOPEN_MODE_CSP: 'csp',
    CANOPEN_MODE_CSV: 'csv',
    CANOPEN_MODE_CURRENT: 'current',
    CANOPEN_MODE_MIT: 'mit',
}
ERROR_FLAGS: Dict[int, str] = {
    0x0001: 'over_voltage',
    0x0002: 'under_voltage',
    0x0004: 'over_temperature',
    0x0008: 'locked_rotor',
    0x0010: 'overload',
    0x0020: 'current_sampling_error',
    0x0040: 'positive_limit',
    0x0080: 'negative_limit',
    0x0100: 'encoder_timeout',
    0x0200: 'overspeed',
    0x0400: 'electric_angle_init_failed',
    0x1000: 'position_error_large',
    0x2000: 'encoder_fault',
}


def clamp(value: int, minimum: int, maximum: int) -> int:
    """Clamp ``value`` to the inclusive range ``[minimum, maximum]``."""
    return max(minimum, min(maximum, value))


def normalize_mode(mode: int | str) -> int:
    """Convert string/int mode representation to protocol mode code."""
    if isinstance(mode, int):
        return mode
    key = mode.strip().lower()
    if key not in MODE_NAME_MAP:
        raise ValueError(f'Unsupported control mode: {mode}')
    return MODE_NAME_MAP[key]


def mode_name(mode: int) -> str:
    """Return a human-readable mode name."""
    return MODE_CODE_MAP.get(mode, f'unknown_{mode}')


def fault_descriptions(error_code: int) -> List[str]:
    """Decode documented fault-bit meanings from a bitmask."""
    return [name for bit, name in ERROR_FLAGS.items() if error_code & bit]


def degrees_to_load_position_counts(angle_deg: float) -> int:
    """Convert -180..180 degrees into signed 16-bit custom feedback counts."""
    return clamp(int(round(angle_deg * 32768.0 / 180.0)), -32768, 32767)


def load_position_counts_to_degrees(counts: int) -> float:
    """Convert signed custom feedback counts into degrees."""
    return counts * 180.0 / 32768.0


def absolute_encoder_counts_to_radians(counts: int) -> float:
    """Convert 16-bit absolute encoder counts to radians over one revolution."""
    return counts * (2.0 * math.pi / 65536.0)


def absolute_encoder_counts_to_degrees(counts: int) -> float:
    """Convert 16-bit absolute encoder counts to degrees over one revolution."""
    return counts * (360.0 / 65536.0)


def rpm_to_rad_s(rpm: float) -> float:
    """Convert RPM to rad/s."""
    return rpm * 2.0 * math.pi / 60.0


def decode_ascii(value: bytes) -> str:
    """Decode ASCII-like SDO string payloads, trimming NUL bytes."""
    return value.rstrip(b'\x00').decode('ascii', errors='ignore').strip()


@dataclass(frozen=True)
class SdoResponse:
    """Decoded SDO response or abort frame.

    Attributes:
        node_id: CANopen node identifier derived from ``0x580 + node_id``.
        command: SDO command specifier byte returned by the actuator.
        index: 16-bit object-dictionary index referenced by the response.
        subindex: 8-bit object-dictionary subindex referenced by the response.
        data: Raw expedited payload bytes in little-endian order.
        aborted: ``True`` when this frame is an SDO abort response.
        abort_code: Optional 32-bit abort code when ``aborted`` is true.
    """

    node_id: int
    command: int
    index: int
    subindex: int
    data: bytes
    aborted: bool = False
    abort_code: Optional[int] = None

    @property
    def value_unsigned(self) -> int:
        """Return payload interpreted as unsigned little-endian integer."""
        return int.from_bytes(self.data, byteorder='little', signed=False)

    @property
    def value_signed(self) -> int:
        """Return payload interpreted as signed little-endian integer."""
        return int.from_bytes(self.data, byteorder='little', signed=True)


@dataclass(frozen=True)
class HeartbeatInfo:
    """Heartbeat or bootup notification parsed from 0x700+node_id."""

    node_id: int
    state: int

    @property
    def is_bootup(self) -> bool:
        return self.state == 0x00


@dataclass(frozen=True)
class CustomFeedback:
    """Decoded custom CAN FD actuator feedback from ``0x300 + node_id``.

    Attributes:
        node_id: CAN node identifier for the reporting actuator.
        position_counts: Signed load-side position counts in the documented
            ``[-32768, 32767]`` range mapping to ``[-180°, 180°]``.
        velocity_rpm: Signed motor-side actual speed in RPM.
        current_ma: Signed q-axis current in milliamps.
        error_code: Bitmask matching the documented actuator fault table.
        coil_temperature_c: Winding temperature in degrees Celsius.
        mode: Current control mode feedback code.
        status_bits: Packed enable/brake/fault/in-position status flags.
    """

    node_id: int
    position_counts: int
    velocity_rpm: int
    current_ma: int
    error_code: int
    coil_temperature_c: float
    mode: int
    status_bits: int

    @property
    def position_deg(self) -> float:
        return load_position_counts_to_degrees(self.position_counts)

    @property
    def position_rad(self) -> float:
        return math.radians(self.position_deg)

    @property
    def velocity_rad_s(self) -> float:
        return rpm_to_rad_s(float(self.velocity_rpm))

    @property
    def current_a(self) -> float:
        return self.current_ma / 1000.0

    @property
    def enabled(self) -> bool:
        return bool(self.status_bits & 0x80)

    @property
    def brake_released(self) -> bool:
        return bool(self.status_bits & 0x40)

    @property
    def fault(self) -> bool:
        return bool(self.status_bits & 0x20)

    @property
    def position_reached(self) -> bool:
        return bool(self.status_bits & 0x10)


@dataclass(frozen=True)
class Tpdo1Feedback:
    """Decoded CANopen TPDO1 feedback from mapped 6041/6078/6064 fields."""

    node_id: int
    status_word: int
    actual_current_ma: int
    actual_position_counts: int

    @property
    def actual_position_deg(self) -> float:
        return absolute_encoder_counts_to_degrees(self.actual_position_counts)

    @property
    def actual_position_rad(self) -> float:
        return absolute_encoder_counts_to_radians(self.actual_position_counts)


@dataclass(frozen=True)
class MultiAxisCommand:
    """One slot of the documented 8-axis CAN FD broadcast frame.

    Attributes:
        node_id: Target actuator node ID for this slot.
        mode: Control mode name or numeric code.
        target_1: Primary target value. It represents position, speed, or
            current depending on the selected control mode.
        target_2: Secondary target value. It represents acceleration/deceleration
            in profile modes and is otherwise reserved by the manual.
        feedforward: Optional feed-forward/profile value. In profile-position
            mode the manual uses it for output-side profile velocity.
        enable: Whether to set the enable bit in the command header.
        release_brake: Whether to release the brake in the command header.
        clear_error: Whether to request fault reset in the command header.
    """

    node_id: int
    mode: int | str
    target_1: int
    target_2: int = 0
    feedforward: int = 0
    enable: bool = True
    release_brake: bool = True
    clear_error: bool = False



def build_nmt_start_frame(node_id: int = 0x00) -> CanFrame:
    """Build the documented NMT 'start node' command."""
    return 0x000, bytes([NMT_START_REMOTE_NODE, node_id & 0x7F]), False



def build_sync_frame() -> CanFrame:
    """Build a CANopen SYNC frame."""
    return SYNC_FRAME_ID, b'', False



def build_sdo_read_request(node_id: int, index: int, subindex: int = 0) -> CanFrame:
    """Build a classic CANopen SDO expedited read request."""
    data = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF, 0, 0, 0, 0])
    return SDO_TX_BASE + (node_id & 0x7F), data, False



def build_sdo_write_request(
    node_id: int,
    index: int,
    subindex: int,
    value: int,
    size: int,
    signed: bool = False,
) -> CanFrame:
    """Build a documented SDO expedited write request for 1/2/4-byte payloads."""
    command_map = {1: 0x2F, 2: 0x2B, 4: 0x23}
    if size not in command_map:
        raise ValueError(f'Unsupported SDO write size: {size}')
    payload = int(value).to_bytes(size, byteorder='little', signed=signed)
    data = bytes([command_map[size], index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF]) + payload
    return SDO_TX_BASE + (node_id & 0x7F), data.ljust(8, b'\x00'), False



def build_query_version_sequence(node_id: int) -> List[CanFrame]:
    """Read the four version/model string objects mentioned in the manual."""
    return [
        build_sdo_read_request(node_id, IDX_VENDOR_NAME),
        build_sdo_read_request(node_id, IDX_DEVICE_MODEL),
        build_sdo_read_request(node_id, IDX_FIRMWARE_VERSION),
        build_sdo_read_request(node_id, IDX_HARDWARE_VERSION),
    ]



def build_controlword_sequence(node_id: int, *controlwords: int) -> List[CanFrame]:
    """Write one or more controlword values to 0x6040."""
    return [build_sdo_write_request(node_id, IDX_CONTROLWORD, 0, value, 2) for value in controlwords]



def build_enable_sequence(node_id: int) -> List[CanFrame]:
    """Build the documented enable steps: 0x06 → 0x07 → 0x0F."""
    return build_controlword_sequence(node_id, 0x0006, 0x0007, 0x000F)



def build_profile_position_sequence(
    node_id: int,
    target_position: int,
    profile_velocity: int,
    acceleration: int,
    deceleration: int,
    start_motion: bool = True,
) -> List[CanFrame]:
    """Build the documented SDO profile-position command sequence."""
    frames = build_enable_sequence(node_id)
    frames.extend(
        [
            build_sdo_write_request(node_id, IDX_MODE_OF_OPERATION, 0, CANOPEN_MODE_PP, 1, signed=True),
            build_sdo_write_request(node_id, IDX_ACCELERATION, 0, clamp(acceleration, 0, 10000), 4),
            build_sdo_write_request(node_id, IDX_DECELERATION, 0, clamp(deceleration, 0, 10000), 4),
            build_sdo_write_request(node_id, IDX_PROFILE_VELOCITY, 0, clamp(profile_velocity, 0, 30), 4),
            build_sdo_write_request(node_id, IDX_TARGET_POSITION, 0, clamp(target_position, -32768, 32767), 4, signed=True),
        ]
    )
    if start_motion:
        frames.append(build_sdo_write_request(node_id, IDX_CONTROLWORD, 0, 0x004F, 2))
    return frames



def build_profile_velocity_sequence(
    node_id: int,
    target_velocity_rpm: int,
    acceleration: int,
    deceleration: int,
) -> List[CanFrame]:
    """Build the documented SDO profile-velocity command sequence."""
    frames = build_enable_sequence(node_id)
    frames.extend(
        [
            build_sdo_write_request(node_id, IDX_MODE_OF_OPERATION, 0, CANOPEN_MODE_PV, 1, signed=True),
            build_sdo_write_request(node_id, IDX_ACCELERATION, 0, clamp(acceleration, 0, 10000), 4),
            build_sdo_write_request(node_id, IDX_DECELERATION, 0, clamp(deceleration, 0, 10000), 4),
            build_sdo_write_request(node_id, IDX_TARGET_VELOCITY, 0, clamp(target_velocity_rpm, -32768, 32767), 4, signed=True),
        ]
    )
    return frames



def build_current_sequence(node_id: int, target_current_ma: int) -> List[CanFrame]:
    """Build the documented SDO current-mode command sequence."""
    frames = build_enable_sequence(node_id)
    frames.extend(
        [
            build_sdo_write_request(node_id, IDX_MODE_OF_OPERATION, 0, CANOPEN_MODE_CURRENT, 1, signed=True),
            build_sdo_write_request(node_id, IDX_TARGET_CURRENT, 0, clamp(target_current_ma, -32768, 32767), 2, signed=True),
        ]
    )
    return frames



def build_query_state_sequence(node_id: int) -> List[CanFrame]:
    """Read the documented telemetry state objects."""
    return [
        build_sdo_read_request(node_id, IDX_ACTUAL_CURRENT),
        build_sdo_read_request(node_id, IDX_ACTUAL_POSITION),
        build_sdo_read_request(node_id, IDX_ACTUAL_SPEED),
        build_sdo_read_request(node_id, IDX_ERROR_CODE),
        build_sdo_read_request(node_id, IDX_STATUSWORD),
        build_sdo_read_request(node_id, IDX_MOS_TEMP),
        build_sdo_read_request(node_id, IDX_MOTOR_TEMP),
    ]



def build_canopen_pdo_mapping_sequence(node_id: int) -> List[CanFrame]:
    """Build the full TPDO1/RPDO1 mapping sequence shown in the manual."""
    return [
        build_sdo_write_request(node_id, IDX_HEARTBEAT_TIME, 0, 1000, 2),
        build_sdo_write_request(node_id, 0x1801, 0x01, 0x80000281, 4),
        build_sdo_write_request(node_id, 0x1802, 0x01, 0x80000381, 4),
        build_sdo_write_request(node_id, 0x1803, 0x01, 0x80000481, 4),
        build_sdo_write_request(node_id, 0x1401, 0x01, 0x80000301, 4),
        build_sdo_write_request(node_id, 0x1402, 0x01, 0x80000401, 4),
        build_sdo_write_request(node_id, 0x1403, 0x01, 0x80000501, 4),
        build_sdo_write_request(node_id, 0x1800, 0x02, 0x01, 1),
        build_sdo_write_request(node_id, 0x1A00, 0x00, 0x00, 1),
        build_sdo_write_request(node_id, 0x1A00, 0x01, 0x60410010, 4),
        build_sdo_write_request(node_id, 0x1A00, 0x02, 0x60780010, 4),
        build_sdo_write_request(node_id, 0x1A00, 0x03, 0x60640020, 4),
        build_sdo_write_request(node_id, 0x1A00, 0x00, 0x03, 1),
        build_sdo_write_request(node_id, 0x1800, 0x01, TPDO1_BASE + (node_id & 0x7F), 4),
        build_sdo_write_request(node_id, 0x1400, 0x02, 0x01, 1),
        build_sdo_write_request(node_id, 0x1600, 0x00, 0x00, 1),
        build_sdo_write_request(node_id, 0x1600, 0x01, 0x60400010, 4),
        build_sdo_write_request(node_id, 0x1600, 0x02, 0x60710010, 4),
        build_sdo_write_request(node_id, 0x1600, 0x03, 0x607A0020, 4),
        build_sdo_write_request(node_id, 0x1600, 0x00, 0x03, 1),
        build_sdo_write_request(node_id, 0x1400, 0x01, RPDO1_BASE + (node_id & 0x7F), 4),
        build_sdo_write_request(node_id, IDX_MODE_OF_OPERATION, 0, 0x08, 1, signed=True),
    ]



def build_rpdo1_frame(node_id: int, controlword: int, target_current_ma: int, target_position: int) -> CanFrame:
    """Build the mapped RPDO1 example after TPDO/RPDO configuration."""
    data = struct.pack('<HhI', controlword & 0xFFFF, clamp(target_current_ma, -32768, 32767), target_position & 0xFFFFFFFF)
    return RPDO1_BASE + (node_id & 0x7F), data, False



def _control_flags(mode: int, enable: bool, release_brake: bool, clear_error: bool) -> int:
    flags = (mode & 0x0F) << 1
    if enable:
        flags |= 0x80
    if release_brake:
        flags |= 0x40
    if clear_error:
        flags |= 0x20
    return flags



def build_custom_command(
    node_id: int,
    mode: int | str,
    target_1: int,
    target_2: int = 0,
    feedforward: int = 0,
    *,
    enable: bool = True,
    release_brake: bool = True,
    clear_error: bool = False,
) -> CanFrame:
    """Build the documented 7-byte CAN FD single-axis control frame."""
    mode_code = normalize_mode(mode)
    data = bytes([_control_flags(mode_code, enable, release_brake, clear_error)])
    data += struct.pack('>h', clamp(target_1, -32768, 32767))
    data += struct.pack('>h', clamp(target_2, -32768, 32767))
    data += struct.pack('>h', clamp(feedforward, -32768, 32767))
    return CUSTOM_SINGLE_AXIS_BASE + (node_id & 0x7F), data, True



def build_custom_multi_command(commands: Sequence[MultiAxisCommand]) -> CanFrame:
    """Build the documented 64-byte CAN FD broadcast command for up to 8 actuators."""
    if len(commands) > 8:
        raise ValueError('Multi-axis command supports at most 8 actuators')
    payload = bytearray(64)
    for slot, command in enumerate(commands):
        _, frame_data, _ = build_custom_command(
            command.node_id,
            command.mode,
            command.target_1,
            command.target_2,
            command.feedforward,
            enable=command.enable,
            release_brake=command.release_brake,
            clear_error=command.clear_error,
        )
        offset = slot * 7
        payload[offset:offset + 7] = frame_data
        payload[56 + slot] = command.node_id & 0x7F
    return CUSTOM_MULTI_AXIS_ID, bytes(payload), True



def _scale_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    span = maximum - minimum
    if span <= 0:
        raise ValueError('Scale range is invalid: maximum must be greater than minimum')
    limit = (1 << bits) - 1
    ratio = (value - minimum) / span
    ratio = max(0.0, min(1.0, ratio))
    return int(round(ratio * limit))



def build_mit_command(
    node_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    *,
    pos_min: float = -180.0,
    pos_max: float = 180.0,
    vel_max: float = 4000.0,
    kp_max: float = 500.0,
    kd_max: float = 5.0,
    torque_max: float = 30.0,
    enable: bool = True,
    release_brake: bool = True,
    clear_error: bool = False,
) -> CanFrame:
    """Build the documented 9-byte MIT-mode CAN FD command frame."""
    flags = _control_flags(CANOPEN_MODE_MIT, enable, release_brake, clear_error)
    pos_u16 = _scale_to_uint(position, pos_min, pos_max, 16)
    vel_u12 = _scale_to_uint(velocity, -vel_max, vel_max, 12)
    kp_u12 = _scale_to_uint(kp, 0.0, kp_max, 12)
    kd_u12 = _scale_to_uint(kd, 0.0, kd_max, 12)
    torque_u12 = _scale_to_uint(torque, -torque_max, torque_max, 12)
    data = bytearray(9)
    data[0] = flags
    data[1] = (pos_u16 >> 8) & 0xFF
    data[2] = pos_u16 & 0xFF
    data[3] = (vel_u12 >> 4) & 0xFF
    data[4] = ((vel_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F)
    data[5] = kp_u12 & 0xFF
    data[6] = (kd_u12 >> 4) & 0xFF
    data[7] = ((kd_u12 & 0x0F) << 4) | ((torque_u12 >> 8) & 0x0F)
    data[8] = torque_u12 & 0xFF
    return CUSTOM_MIT_BASE + (node_id & 0x7F), bytes(data), True



def parse_sdo_response(arbitration_id: int, data: bytes) -> SdoResponse:
    """Parse a standard 8-byte SDO response or abort frame."""
    if len(data) < 8:
        raise ValueError(f'SDO response must be 8 bytes, got {len(data)}')
    node_id = arbitration_id - SDO_RX_BASE
    command = data[0]
    index = data[1] | (data[2] << 8)
    subindex = data[3]
    payload = bytes(data[4:8])
    if command == 0x80:
        return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, data=b'', aborted=True, abort_code=int.from_bytes(payload, 'little'))
    if command == 0x60:
        return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, data=b'')
    if command in (0x4F, 0x4B, 0x43):
        size = {0x4F: 1, 0x4B: 2, 0x43: 4}[command]
        return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, data=payload[:size])
    return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, data=payload)



def parse_heartbeat(arbitration_id: int, data: bytes) -> HeartbeatInfo:
    """Parse bootup/heartbeat frames from 0x700+node_id."""
    if len(data) < 1:
        raise ValueError('Heartbeat frame requires at least one byte')
    return HeartbeatInfo(node_id=arbitration_id - HEARTBEAT_BASE, state=int(data[0]))



def parse_custom_feedback(arbitration_id: int, data: bytes) -> CustomFeedback:
    """Parse the documented 12-byte custom actuator feedback frame."""
    if len(data) < 12:
        raise ValueError(f'Custom feedback must be 12 bytes, got {len(data)}')
    position_counts, velocity_rpm, current_ma, error_code, temp_raw = struct.unpack('>hhhHh', data[:10])
    return CustomFeedback(
        node_id=arbitration_id - CUSTOM_FEEDBACK_BASE,
        position_counts=position_counts,
        velocity_rpm=velocity_rpm,
        current_ma=current_ma,
        error_code=error_code,
        coil_temperature_c=temp_raw / 10.0,
        mode=int(data[10]),
        status_bits=int(data[11]),
    )



def parse_tpdo1_feedback(arbitration_id: int, data: bytes) -> Tpdo1Feedback:
    """Parse mapped TPDO1 data for 6041, 6078, and 6064."""
    if len(data) < 8:
        raise ValueError(f'TPDO1 feedback must be 8 bytes, got {len(data)}')
    status_word, actual_current_ma, actual_position_counts = struct.unpack('<HhI', data[:8])
    return Tpdo1Feedback(
        node_id=arbitration_id - TPDO1_BASE,
        status_word=status_word,
        actual_current_ma=actual_current_ma,
        actual_position_counts=actual_position_counts,
    )
