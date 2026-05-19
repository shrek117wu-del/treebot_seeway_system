"""Protocol helpers for JuxieDrive CANopen/CAN-FD joint modules."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
from typing import Dict, Iterable, List, Sequence, Tuple


NMT_BROADCAST_ID = 0x000
SYNC_ID = 0x080
SDO_TX_BASE = 0x600
SDO_RX_BASE = 0x580
BOOTUP_HEARTBEAT_BASE = 0x700
SINGLE_AXIS_FD_BASE = 0x100
MIT_FD_BASE = 0x110
MULTI_AXIS_FD_ID = 0x200
FEEDBACK_FD_BASE = 0x300

HEARTBEAT_BOOTUP = 0x00
HEARTBEAT_STOPPED = 0x04
HEARTBEAT_OPERATIONAL = 0x05
HEARTBEAT_PREOP = 0x7F

POSITION_SCALE_COUNTS_PER_REV = 65536.0
POSITION_DEGREES_PER_REV = 360.0
TEMPERATURE_SCALE = 10.0
ACTUAL_SPEED_SCALE = 10.0

INDEX_DEVICE_TYPE = 0x1000
INDEX_MANUFACTURER_DEVICE_NAME = 0x1008
INDEX_HARDWARE_VERSION = 0x1009
INDEX_SOFTWARE_VERSION = 0x100A
INDEX_HEARTBEAT_PRODUCER = 0x1017
INDEX_CONTROL_WORD = 0x6040
INDEX_STATUS_WORD = 0x6041
INDEX_MODE_OF_OPERATION = 0x6060
INDEX_POSITION_LIMIT = 0x607D
INDEX_TARGET_CURRENT = 0x6071
INDEX_MAX_CURRENT = 0x2538
INDEX_TARGET_POSITION = 0x607A
INDEX_ACTUAL_POSITION = 0x6064
INDEX_TARGET_VELOCITY = 0x60FF
INDEX_ACTUAL_VELOCITY = 0x606C
INDEX_PROFILE_VELOCITY = 0x6081
INDEX_PROFILE_ACCELERATION = 0x6083
INDEX_PROFILE_DECELERATION = 0x6084
INDEX_ACTUAL_CURRENT = 0x6078
INDEX_ERROR_CODE = 0x603F
INDEX_MOS_TEMPERATURE = 0x2662
INDEX_MOTOR_TEMPERATURE = 0x2663
INDEX_SET_ID = 0x2530
INDEX_ZERO_CALIBRATION = 0x2531
INDEX_CANFD_DATA_BITRATE = 0x2540
INDEX_DISABLE_WATCHDOG_LIMIT = 0x2650

INDEX_CURRENT_LOOP_P = 0x2532
INDEX_CURRENT_LOOP_I = 0x2533
INDEX_SPEED_LOOP_P = 0x2534
INDEX_SPEED_LOOP_I = 0x2535
INDEX_POSITION_LOOP_P = 0x2536
INDEX_POSITION_LOOP_I = 0x2537

STATUS_READY_TO_SWITCH_ON = 0x0021
STATUS_SWITCHED_ON = 0x0023
STATUS_OPERATION_ENABLED = 0x0027
STATUS_FAULT = 0x0008


class ControlMode(IntEnum):
    PROFILE_POSITION = 0x01
    PROFILE_VELOCITY = 0x02
    CSP = 0x03
    CSV = 0x04
    CURRENT = 0x05
    MIT = 0x06


class NmtCommand(IntEnum):
    START_REMOTE_NODE = 0x01
    STOP_REMOTE_NODE = 0x02
    ENTER_PRE_OPERATIONAL = 0x80
    RESET_NODE = 0x81
    RESET_COMMUNICATION = 0x82


@dataclass(frozen=True)
class JointModelSpec:
    """Mechanical/electrical presets extracted from the JuxieDrive product manual."""

    name: str
    reduction_ratio: float
    output_speed_rpm: float
    rated_torque_nm: float
    peak_torque_nm: float
    rated_voltage_v: float = 48.0
    rated_current_a: float = 0.0
    peak_current_a: float = 0.0

    @property
    def mit_velocity_limit_rpm(self) -> float:
        return self.output_speed_rpm * self.reduction_ratio

    @property
    def mit_torque_limit_nm(self) -> float:
        return self.peak_torque_nm


MODEL_SPECS: Dict[str, JointModelSpec] = {
    'generic': JointModelSpec('generic', reduction_ratio=101.0, output_speed_rpm=30.0, rated_torque_nm=45.0, peak_torque_nm=45.0, rated_current_a=5.0, peak_current_a=10.0),
    'r48_101': JointModelSpec('r48_101', reduction_ratio=101.0, output_speed_rpm=30.0, rated_torque_nm=16.5, peak_torque_nm=16.0, rated_current_a=1.3, peak_current_a=3.2),
    'r58_101': JointModelSpec('r58_101', reduction_ratio=101.0, output_speed_rpm=30.0, rated_torque_nm=20.0, peak_torque_nm=45.0, rated_current_a=3.5, peak_current_a=10.8),
    'r68_101': JointModelSpec('r68_101', reduction_ratio=101.0, output_speed_rpm=30.0, rated_torque_nm=33.0, peak_torque_nm=82.0, rated_current_a=5.4, peak_current_a=16.3),
    'r83_101': JointModelSpec('r83_101', reduction_ratio=101.0, output_speed_rpm=30.0, rated_torque_nm=53.0, peak_torque_nm=120.0, rated_current_a=6.0, peak_current_a=15.9),
    'r102_161': JointModelSpec('r102_161', reduction_ratio=161.0, output_speed_rpm=30.0, rated_torque_nm=147.0, peak_torque_nm=350.0),
    'r120_161': JointModelSpec('r120_161', reduction_ratio=161.0, output_speed_rpm=30.0, rated_torque_nm=180.0, peak_torque_nm=450.0),
    'r120_161_max': JointModelSpec('r120_161_max', reduction_ratio=161.0, output_speed_rpm=30.0, rated_torque_nm=300.0, peak_torque_nm=600.0),
}


@dataclass(frozen=True)
class SingleAxisCommand:
    enable: bool
    release_brake: bool
    clear_error: bool
    control_mode: int
    target_param_1: int
    target_param_2: int = 0
    feedforward: int = 0


@dataclass(frozen=True)
class MitCommand:
    enable: bool
    release_brake: bool
    clear_error: bool
    target_position_deg: float
    target_velocity_rpm: float
    kp: float
    kd: float
    target_torque_nm: float
    position_limit_deg: float = 180.0
    velocity_limit_rpm: float = 3000.0
    torque_limit_nm: float = 45.0


@dataclass(frozen=True)
class SdoResponse:
    node_id: int
    command: int
    index: int
    subindex: int
    raw_data: bytes
    abort_code: int = 0

    @property
    def is_abort(self) -> bool:
        return self.command == 0x80

    @property
    def success(self) -> bool:
        return not self.is_abort

    def as_unsigned(self) -> int:
        return int.from_bytes(self.raw_data or b'\x00', 'little', signed=False)

    def as_signed(self) -> int:
        return int.from_bytes(self.raw_data or b'\x00', 'little', signed=True)

    def as_ascii(self) -> str:
        return self.raw_data.rstrip(b'\x00').decode('ascii', errors='ignore')


@dataclass(frozen=True)
class JointFeedback:
    node_id: int
    position_deg: float
    velocity_rpm: float
    current_ma: float
    error_code: int
    coil_temperature_c: float
    control_mode: int
    status_bits: int

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
    def target_reached(self) -> bool:
        return bool(self.status_bits & 0x10)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def degrees_to_position_counts(degrees: float) -> int:
    wrapped = max(-180.0, min(180.0, degrees))
    # The protocol maps one mechanical revolution to 65,536 discrete absolute-position counts.
    counts = int(round(wrapped * POSITION_SCALE_COUNTS_PER_REV / POSITION_DEGREES_PER_REV))
    return max(-32768, min(32767, counts))


def position_counts_to_degrees(raw_value: int) -> float:
    return float(raw_value) * POSITION_DEGREES_PER_REV / POSITION_SCALE_COUNTS_PER_REV


def encode_range(value: float, minimum: float, maximum: float, bits: int) -> int:
    span = maximum - minimum
    if span <= 0.0:
        raise ValueError('invalid encoding range')
    value = clamp(value, minimum, maximum)
    max_raw = (1 << bits) - 1
    return int(round((value - minimum) * max_raw / span))


def decode_range(raw_value: int, minimum: float, maximum: float, bits: int) -> float:
    max_raw = (1 << bits) - 1
    return minimum + (maximum - minimum) * float(raw_value) / float(max_raw)


def build_nmt_command(node_id: int, command: NmtCommand) -> Tuple[int, bytes, bool]:
    return NMT_BROADCAST_ID, bytes([int(command), node_id & 0x7F]), False


def build_sync_frame() -> Tuple[int, bytes, bool]:
    return SYNC_ID, b'', False


def build_sdo_read(node_id: int, index: int, subindex: int = 0x00) -> Tuple[int, bytes, bool]:
    data = bytes([0x40]) + struct.pack('<H', index) + bytes([subindex, 0x00, 0x00, 0x00, 0x00])
    return SDO_TX_BASE + node_id, data, False


def build_sdo_write(node_id: int, index: int, subindex: int, raw_data: bytes) -> Tuple[int, bytes, bool]:
    if len(raw_data) not in (1, 2, 4):
        raise ValueError('SDO expedited writes must be 1, 2, or 4 bytes')
    command = {1: 0x2F, 2: 0x2B, 4: 0x23}[len(raw_data)]
    padded = raw_data + bytes(4 - len(raw_data))
    data = bytes([command]) + struct.pack('<H', index) + bytes([subindex]) + padded
    return SDO_TX_BASE + node_id, data, False


def encode_integer(value: int, width: int, *, signed: bool) -> bytes:
    return int(value).to_bytes(width, byteorder='little', signed=signed)


def build_sdo_write_i32(node_id: int, index: int, subindex: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write(node_id, index, subindex, encode_integer(value, 4, signed=True))


def build_sdo_write_u32(node_id: int, index: int, subindex: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write(node_id, index, subindex, encode_integer(value, 4, signed=False))


def build_sdo_write_u16(node_id: int, index: int, subindex: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write(node_id, index, subindex, encode_integer(value, 2, signed=False))


def build_sdo_write_i16(node_id: int, index: int, subindex: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write(node_id, index, subindex, encode_integer(value, 2, signed=True))


def build_sdo_write_u8(node_id: int, index: int, subindex: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write(node_id, index, subindex, encode_integer(value, 1, signed=False))


def build_enable_sequence(node_id: int) -> List[Tuple[int, bytes, bool]]:
    return [
        build_sdo_write_u16(node_id, INDEX_CONTROL_WORD, 0x00, 0x0006),
        build_sdo_write_u16(node_id, INDEX_CONTROL_WORD, 0x00, 0x0007),
        build_sdo_write_u16(node_id, INDEX_CONTROL_WORD, 0x00, 0x000F),
    ]


def build_profile_position_sequence(
    node_id: int,
    position_deg: float,
    profile_velocity_rpm: int,
    acceleration_rpm_s: int,
    deceleration_rpm_s: int,
) -> List[Tuple[int, bytes, bool]]:
    return [
        *build_enable_sequence(node_id),
        build_sdo_write_u8(node_id, INDEX_MODE_OF_OPERATION, 0x00, ControlMode.PROFILE_POSITION),
        build_sdo_write_u32(node_id, INDEX_PROFILE_ACCELERATION, 0x00, acceleration_rpm_s),
        build_sdo_write_u32(node_id, INDEX_PROFILE_DECELERATION, 0x00, deceleration_rpm_s),
        build_sdo_write_u32(node_id, INDEX_PROFILE_VELOCITY, 0x00, profile_velocity_rpm),
        build_sdo_write_i32(node_id, INDEX_TARGET_POSITION, 0x00, degrees_to_position_counts(position_deg)),
        build_sdo_write_u16(node_id, INDEX_CONTROL_WORD, 0x00, 0x004F),
    ]


def build_profile_velocity_sequence(node_id: int, velocity_rpm: int, acceleration_rpm_s: int, deceleration_rpm_s: int) -> List[Tuple[int, bytes, bool]]:
    return [
        *build_enable_sequence(node_id),
        build_sdo_write_u8(node_id, INDEX_MODE_OF_OPERATION, 0x00, ControlMode.PROFILE_VELOCITY),
        build_sdo_write_u32(node_id, INDEX_PROFILE_ACCELERATION, 0x00, acceleration_rpm_s),
        build_sdo_write_u32(node_id, INDEX_PROFILE_DECELERATION, 0x00, deceleration_rpm_s),
        build_sdo_write_i32(node_id, INDEX_TARGET_VELOCITY, 0x00, velocity_rpm),
    ]


def build_current_mode_sequence(node_id: int, target_current_ma: int) -> List[Tuple[int, bytes, bool]]:
    return [
        *build_enable_sequence(node_id),
        build_sdo_write_u8(node_id, INDEX_MODE_OF_OPERATION, 0x00, ControlMode.CURRENT),
        build_sdo_write_u16(node_id, INDEX_TARGET_CURRENT, 0x00, target_current_ma),
    ]


def build_version_read_sequence(node_id: int) -> List[Tuple[int, bytes, bool]]:
    return [
        build_sdo_read(node_id, INDEX_MANUFACTURER_DEVICE_NAME, 0x00),
        build_sdo_read(node_id, INDEX_DEVICE_TYPE, 0x00),
        build_sdo_read(node_id, INDEX_SOFTWARE_VERSION, 0x00),
        build_sdo_read(node_id, INDEX_HARDWARE_VERSION, 0x00),
    ]


def build_status_read_sequence(node_id: int) -> List[Tuple[int, bytes, bool]]:
    return [
        build_sdo_read(node_id, INDEX_ACTUAL_CURRENT, 0x00),
        build_sdo_read(node_id, INDEX_ACTUAL_POSITION, 0x00),
        build_sdo_read(node_id, INDEX_ACTUAL_VELOCITY, 0x00),
        build_sdo_read(node_id, INDEX_ERROR_CODE, 0x00),
        build_sdo_read(node_id, INDEX_STATUS_WORD, 0x00),
        build_sdo_read(node_id, INDEX_MOS_TEMPERATURE, 0x00),
        build_sdo_read(node_id, INDEX_MOTOR_TEMPERATURE, 0x00),
    ]


def build_set_id_command(node_id: int, new_node_id: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u32(node_id, INDEX_SET_ID, 0x00, new_node_id)


def build_zero_calibration_command(node_id: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u32(node_id, INDEX_ZERO_CALIBRATION, 0x00, 0x00000001)


def build_disable_watchdog_and_limit_command(node_id: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u32(node_id, INDEX_DISABLE_WATCHDOG_LIMIT, 0x00, 0x00000001)


def build_set_heartbeat_command(node_id: int, heartbeat_ms: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u16(node_id, INDEX_HEARTBEAT_PRODUCER, 0x00, heartbeat_ms)


def build_set_limit_command(node_id: int, degrees: float, *, positive: bool) -> Tuple[int, bytes, bool]:
    subindex = 0x02 if positive else 0x01
    return build_sdo_write_i32(node_id, INDEX_POSITION_LIMIT, subindex, degrees_to_position_counts(degrees))


def build_read_limit_command(node_id: int, *, positive: bool) -> Tuple[int, bytes, bool]:
    subindex = 0x02 if positive else 0x01
    return build_sdo_read(node_id, INDEX_POSITION_LIMIT, subindex)


def build_set_canfd_bitrate_command(node_id: int, bitrate_code: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u32(node_id, INDEX_CANFD_DATA_BITRATE, 0x00, bitrate_code)


def build_set_pi_command(node_id: int, parameter_index: int, value: int) -> Tuple[int, bytes, bool]:
    return build_sdo_write_u32(node_id, parameter_index, 0x00, value)


def build_read_pi_command(node_id: int, parameter_index: int) -> Tuple[int, bytes, bool]:
    return build_sdo_read(node_id, parameter_index, 0x00)


def _build_control_byte(enable: bool, release_brake: bool, clear_error: bool, control_mode: int) -> int:
    control_byte = (control_mode & 0x0F) << 1
    if enable:
        control_byte |= 0x80
    if release_brake:
        control_byte |= 0x40
    if clear_error:
        control_byte |= 0x20
    return control_byte


def build_single_axis_fd_command(node_id: int, command: SingleAxisCommand) -> Tuple[int, bytes, bool]:
    payload = bytes([
        _build_control_byte(command.enable, command.release_brake, command.clear_error, command.control_mode),
    ])
    payload += struct.pack('>h', int(command.target_param_1))
    payload += struct.pack('>H', int(max(0, min(65535, command.target_param_2))))
    payload += struct.pack('>h', int(command.feedforward))
    return SINGLE_AXIS_FD_BASE + node_id, payload, True


def build_multi_axis_fd_command(commands: Sequence[Tuple[int, SingleAxisCommand]]) -> Tuple[int, bytes, bool]:
    if len(commands) > 8:
        raise ValueError('broadcast CAN-FD control supports at most 8 joints')
    payload = bytearray(64)
    for index, (node_id, command) in enumerate(commands):
        slot = build_single_axis_fd_command(node_id, command)[1]
        payload[index * 7:(index + 1) * 7] = slot
        payload[56 + index] = node_id & 0x7F
    return MULTI_AXIS_FD_ID, bytes(payload), True


def build_mit_fd_command(node_id: int, command: MitCommand) -> Tuple[int, bytes, bool]:
    control = _build_control_byte(command.enable, command.release_brake, command.clear_error, ControlMode.MIT)
    pos_raw = encode_range(command.target_position_deg, -command.position_limit_deg, command.position_limit_deg, 16)
    vel_raw = encode_range(command.target_velocity_rpm, -command.velocity_limit_rpm, command.velocity_limit_rpm, 12)
    kp_raw = encode_range(command.kp, 0.0, 500.0, 12)
    kd_raw = encode_range(command.kd, 0.0, 5.0, 12)
    torque_raw = encode_range(command.target_torque_nm, -command.torque_limit_nm, command.torque_limit_nm, 12)
    payload = bytes([
        control,
        (pos_raw >> 8) & 0xFF,
        pos_raw & 0xFF,
        (vel_raw >> 4) & 0xFF,
        ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F),
        kp_raw & 0xFF,
        (kd_raw >> 4) & 0xFF,
        ((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F),
        torque_raw & 0xFF,
    ])
    return MIT_FD_BASE + node_id, payload, True


def parse_bootup_heartbeat(arbitration_id: int, data: bytes) -> Tuple[int, int]:
    if arbitration_id < BOOTUP_HEARTBEAT_BASE or arbitration_id >= BOOTUP_HEARTBEAT_BASE + 0x80:
        raise ValueError('not a bootup/heartbeat frame')
    if len(data) != 1:
        raise ValueError('heartbeat frames must carry exactly one byte')
    return arbitration_id - BOOTUP_HEARTBEAT_BASE, data[0]


def parse_sdo_response(arbitration_id: int, data: bytes) -> SdoResponse:
    if arbitration_id < SDO_RX_BASE or arbitration_id >= SDO_RX_BASE + 0x80:
        raise ValueError('not an SDO response frame')
    if len(data) != 8:
        raise ValueError('SDO responses must be 8 bytes')
    node_id = arbitration_id - SDO_RX_BASE
    command = data[0]
    index = int.from_bytes(data[1:3], 'little')
    subindex = data[3]
    if command == 0x80:
        abort_code = int.from_bytes(data[4:8], 'little')
        return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, raw_data=b'', abort_code=abort_code)
    if command == 0x60:
        return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, raw_data=b'')
    length = {0x4F: 1, 0x4B: 2, 0x43: 4}.get(command, 4)
    return SdoResponse(node_id=node_id, command=command, index=index, subindex=subindex, raw_data=data[4:4 + length])


def parse_joint_feedback(arbitration_id: int, data: bytes) -> JointFeedback:
    if arbitration_id < FEEDBACK_FD_BASE or arbitration_id >= FEEDBACK_FD_BASE + 0x80:
        raise ValueError('not a JuxieDrive custom feedback frame')
    if len(data) != 12:
        raise ValueError('joint feedback frames must be 12 bytes')
    node_id = arbitration_id - FEEDBACK_FD_BASE
    position_raw, velocity_raw, current_raw, error_code, coil_temp_raw, control_mode, status_bits = struct.unpack('>hhhHHBB', data)
    return JointFeedback(
        node_id=node_id,
        position_deg=position_counts_to_degrees(position_raw),
        velocity_rpm=float(velocity_raw),
        current_ma=float(current_raw),
        error_code=int(error_code),
        coil_temperature_c=float(coil_temp_raw) / TEMPERATURE_SCALE,
        control_mode=int(control_mode),
        status_bits=int(status_bits),
    )


def decode_status_word(status_word: int) -> Dict[str, bool]:
    return {
        'ready_to_switch_on': status_word == STATUS_READY_TO_SWITCH_ON,
        'switched_on': status_word == STATUS_SWITCHED_ON,
        'operation_enabled': status_word == STATUS_OPERATION_ENABLED,
        'fault': status_word == STATUS_FAULT,
    }


def error_code_descriptions(error_code: int) -> List[str]:
    mappings = {
        0x0001: 'over_voltage',
        0x0002: 'under_voltage',
        0x0004: 'over_temperature',
        0x0008: 'stall',
        0x0010: 'overload',
        0x0020: 'current_sampling_error',
        0x0040: 'positive_limit',
        0x0080: 'negative_limit',
        0x0100: 'encoder_timeout',
        0x0200: 'max_speed_exceeded',
        0x0400: 'electrical_angle_init_failed',
        0x1000: 'position_error_too_large',
        0x2000: 'encoder_fault',
    }
    return [name for bit, name in mappings.items() if error_code & bit]
