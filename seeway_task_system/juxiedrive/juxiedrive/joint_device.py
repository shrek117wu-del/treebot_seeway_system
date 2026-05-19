"""Device abstraction for one JuxieDrive joint module."""

from __future__ import annotations

import logging
import threading
from typing import Any, Dict, Iterable, Optional, Tuple

from .can_driver import CanDriver, CanFrame
from .protocol import (
    BOOTUP_HEARTBEAT_BASE,
    FEEDBACK_FD_BASE,
    INDEX_ACTUAL_CURRENT,
    INDEX_ACTUAL_POSITION,
    INDEX_ACTUAL_VELOCITY,
    INDEX_DEVICE_TYPE,
    INDEX_ERROR_CODE,
    INDEX_HARDWARE_VERSION,
    INDEX_MANUFACTURER_DEVICE_NAME,
    INDEX_MOS_TEMPERATURE,
    INDEX_MOTOR_TEMPERATURE,
    INDEX_POSITION_LIMIT,
    INDEX_SOFTWARE_VERSION,
    INDEX_STATUS_WORD,
    SDO_RX_BASE,
    SingleAxisCommand,
    MitCommand,
    build_current_mode_sequence,
    build_disable_watchdog_and_limit_command,
    build_enable_sequence,
    build_mit_fd_command,
    build_nmt_command,
    build_profile_position_sequence,
    build_profile_velocity_sequence,
    NmtCommand,
    build_sdo_read,
    build_sdo_write_i16,
    build_sdo_write_i32,
    build_sdo_write_u8,
    build_set_canfd_bitrate_command,
    build_set_heartbeat_command,
    build_set_id_command,
    build_set_limit_command,
    build_set_pi_command,
    build_single_axis_fd_command,
    build_status_read_sequence,
    build_sync_frame,
    build_zero_calibration_command,
    decode_status_word,
    parse_bootup_heartbeat,
    parse_joint_feedback,
    parse_sdo_response,
    position_counts_to_degrees,
)
from .state_types import JointConfig, JointRuntimeState

PI_PARAMETER_INDICES = {
    'current_p': 0x2532,
    'current_i': 0x2533,
    'speed_p': 0x2534,
    'speed_i': 0x2535,
    'position_p': 0x2536,
    'position_i': 0x2537,
    'max_current': 0x2538,
}


class JointDevice:
    """High-level operations and state tracking for a single actuator."""

    def __init__(self, config: JointConfig, transport: CanDriver, logger: Optional[logging.Logger] = None) -> None:
        self.config = config
        self.transport = transport
        self.logger = logger or logging.getLogger(__name__)
        self.state = JointRuntimeState(joint_name=config.name, node_id=config.node_id)
        self._condition = threading.Condition()
        self._pending_response: Optional[Tuple[int, int]] = None
        self._last_sdo_response = None

    def handle_frame(self, frame: CanFrame) -> bool:
        handled = False
        if frame.arbitration_id == BOOTUP_HEARTBEAT_BASE + self.config.node_id:
            _, heartbeat_state = parse_bootup_heartbeat(frame.arbitration_id, frame.data)
            self.state.touch_heartbeat(heartbeat_state)
            handled = True
        elif frame.arbitration_id == FEEDBACK_FD_BASE + self.config.node_id:
            feedback = parse_joint_feedback(frame.arbitration_id, frame.data)
            self.state.position_deg = feedback.position_deg
            self.state.velocity_rpm = feedback.velocity_rpm
            self.state.current_ma = feedback.current_ma
            self.state.error_code = feedback.error_code
            self.state.coil_temperature_c = feedback.coil_temperature_c
            self.state.control_mode = feedback.control_mode
            self.state.enabled = feedback.enabled
            self.state.brake_released = feedback.brake_released
            self.state.fault = feedback.fault
            self.state.target_reached = feedback.target_reached
            self.state.touch_feedback(frame.data)
            handled = True
        elif frame.arbitration_id == SDO_RX_BASE + self.config.node_id:
            response = parse_sdo_response(frame.arbitration_id, frame.data)
            self._apply_sdo_response(response)
            with self._condition:
                if self._pending_response == (response.index, response.subindex):
                    self._last_sdo_response = response
                    self._pending_response = None
                    self._condition.notify_all()
            handled = True
        return handled

    def refresh_online(self, timeout_sec: float) -> bool:
        return self.state.refresh_online(timeout_sec)

    def send_raw(self, arbitration_id: int, data: bytes, is_fd: bool) -> bool:
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def send_frames(self, frames: Iterable[Tuple[int, bytes, bool]]) -> bool:
        ok = True
        for arbitration_id, data, is_fd in frames:
            ok = self.transport.send(arbitration_id, data, is_fd=is_fd) and ok
        return ok

    def start_node(self) -> bool:
        arbitration_id, data, is_fd = build_nmt_command(self.config.node_id, NmtCommand.START_REMOTE_NODE)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def sync_feedback(self) -> bool:
        arbitration_id, data, is_fd = build_sync_frame()
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def enable(self) -> bool:
        return self.send_frames(build_enable_sequence(self.config.node_id))

    def set_zero(self) -> bool:
        arbitration_id, data, is_fd = build_zero_calibration_command(self.config.node_id)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def disable_watchdog_and_limits(self) -> bool:
        arbitration_id, data, is_fd = build_disable_watchdog_and_limit_command(self.config.node_id)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def set_heartbeat(self, heartbeat_ms: int) -> bool:
        arbitration_id, data, is_fd = build_set_heartbeat_command(self.config.node_id, heartbeat_ms)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def set_id(self, new_node_id: int) -> bool:
        arbitration_id, data, is_fd = build_set_id_command(self.config.node_id, new_node_id)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def set_limit(self, degrees: float, *, positive: bool) -> bool:
        arbitration_id, data, is_fd = build_set_limit_command(self.config.node_id, degrees, positive=positive)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def set_canfd_bitrate(self, bitrate_code: int) -> bool:
        arbitration_id, data, is_fd = build_set_canfd_bitrate_command(self.config.node_id, bitrate_code)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def set_pi(self, parameter_name: str, value: int) -> bool:
        index = PI_PARAMETER_INDICES[parameter_name]
        arbitration_id, data, is_fd = build_set_pi_command(self.config.node_id, index, value)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def read_object(self, index: int, subindex: int = 0x00, timeout_sec: float = 0.5):
        with self._condition:
            self._pending_response = (index, subindex)
            self._last_sdo_response = None
        arbitration_id, data, is_fd = build_sdo_read(self.config.node_id, index, subindex)
        if not self.transport.send(arbitration_id, data, is_fd=is_fd):
            with self._condition:
                self._pending_response = None
            return None
        with self._condition:
            self._condition.wait_for(lambda: self._last_sdo_response is not None, timeout=timeout_sec)
            response = self._last_sdo_response
            self._last_sdo_response = None
            return response

    def write_object(self, index: int, subindex: int, width: int, value: int, *, signed: bool = False) -> bool:
        if width == 1:
            arbitration_id, data, is_fd = build_sdo_write_u8(self.config.node_id, index, subindex, value)
        elif width == 2 and signed:
            arbitration_id, data, is_fd = build_sdo_write_i16(self.config.node_id, index, subindex, value)
        elif width == 2:
            from .protocol import build_sdo_write_u16
            arbitration_id, data, is_fd = build_sdo_write_u16(self.config.node_id, index, subindex, value)
        elif width == 4 and signed:
            arbitration_id, data, is_fd = build_sdo_write_i32(self.config.node_id, index, subindex, value)
        elif width == 4:
            from .protocol import build_sdo_write_u32
            arbitration_id, data, is_fd = build_sdo_write_u32(self.config.node_id, index, subindex, value)
        else:
            raise ValueError('unsupported SDO write width')
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def query_versions(self, timeout_sec: float = 0.5) -> Dict[str, Any]:
        result = {}
        for index, field_name in (
            (INDEX_MANUFACTURER_DEVICE_NAME, 'manufacturer_name'),
            (INDEX_DEVICE_TYPE, 'joint_model'),
            (INDEX_SOFTWARE_VERSION, 'firmware_version'),
            (INDEX_HARDWARE_VERSION, 'hardware_version'),
        ):
            response = self.read_object(index, 0x00, timeout_sec=timeout_sec)
            if response is None:
                result[field_name] = ''
                continue
            value = response.as_ascii()
            if not value:
                value = str(response.as_unsigned())
            result[field_name] = value
        self.state.manufacturer_name = result.get('manufacturer_name', '')
        self.state.joint_model = result.get('joint_model', '')
        self.state.firmware_version = result.get('firmware_version', '')
        self.state.hardware_version = result.get('hardware_version', '')
        return result

    def query_state(self, timeout_sec: float = 0.5) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for _, data, _ in build_status_read_sequence(self.config.node_id):
            index = int.from_bytes(data[1:3], 'little')
            response = self.read_object(index, data[3], timeout_sec=timeout_sec)
            result[f'0x{index:04X}'] = None if response is None else {
                'raw': list(response.raw_data),
                'unsigned': response.as_unsigned() if response.raw_data else 0,
                'signed': response.as_signed() if response.raw_data else 0,
            }
        return result

    def send_single_axis_command(self, command: SingleAxisCommand) -> bool:
        arbitration_id, data, is_fd = build_single_axis_fd_command(self.config.node_id, command)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def send_mit_command(self, command: MitCommand) -> bool:
        arbitration_id, data, is_fd = build_mit_fd_command(self.config.node_id, command)
        return self.transport.send(arbitration_id, data, is_fd=is_fd)

    def send_profile_position(self, position_deg: float, profile_velocity_rpm: int, acceleration_rpm_s: int, deceleration_rpm_s: int) -> bool:
        return self.send_frames(
            build_profile_position_sequence(
                self.config.node_id,
                position_deg,
                profile_velocity_rpm,
                acceleration_rpm_s,
                deceleration_rpm_s,
            )
        )

    def send_profile_velocity(self, velocity_rpm: int, acceleration_rpm_s: int, deceleration_rpm_s: int) -> bool:
        return self.send_frames(
            build_profile_velocity_sequence(self.config.node_id, velocity_rpm, acceleration_rpm_s, deceleration_rpm_s)
        )

    def send_current_mode(self, current_ma: int) -> bool:
        return self.send_frames(build_current_mode_sequence(self.config.node_id, current_ma))

    def execute_named_command(self, command: str, arguments: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        arguments = arguments or {}
        if command == 'start_node':
            return {'success': self.start_node()}
        if command == 'sync_feedback':
            return {'success': self.sync_feedback()}
        if command == 'enable':
            return {'success': self.enable()}
        if command == 'set_zero':
            return {'success': self.set_zero()}
        if command == 'disable_watchdog':
            return {'success': self.disable_watchdog_and_limits()}
        if command == 'set_id':
            return {'success': self.set_id(int(arguments['new_id']))}
        if command == 'set_heartbeat':
            return {'success': self.set_heartbeat(int(arguments['heartbeat_ms']))}
        if command == 'set_positive_limit':
            return {'success': self.set_limit(float(arguments['degrees']), positive=True)}
        if command == 'set_negative_limit':
            return {'success': self.set_limit(float(arguments['degrees']), positive=False)}
        if command == 'read_positive_limit':
            response = self.read_object(INDEX_POSITION_LIMIT, 0x02)
            return {'success': response is not None, 'value_deg': position_counts_to_degrees(response.as_signed()) if response else None}
        if command == 'read_negative_limit':
            response = self.read_object(INDEX_POSITION_LIMIT, 0x01)
            return {'success': response is not None, 'value_deg': position_counts_to_degrees(response.as_signed()) if response else None}
        if command == 'set_canfd_bitrate':
            return {'success': self.set_canfd_bitrate(int(arguments['bitrate_code']))}
        if command == 'set_pi':
            return {'success': self.set_pi(str(arguments['parameter']), int(arguments['value']))}
        if command == 'read_pi':
            parameter = str(arguments['parameter'])
            response = self.read_object(PI_PARAMETER_INDICES[parameter], 0x00)
            return {'success': response is not None, 'value': response.as_unsigned() if response else None}
        if command == 'query_versions':
            result = self.query_versions()
            result['success'] = True
            return result
        if command == 'query_state':
            return {'success': True, 'state': self.query_state()}
        if command == 'profile_position':
            return {
                'success': self.send_profile_position(
                    float(arguments['position_deg']),
                    int(arguments.get('profile_velocity_rpm', 10)),
                    int(arguments.get('acceleration_rpm_s', 2000)),
                    int(arguments.get('deceleration_rpm_s', 2000)),
                )
            }
        if command == 'profile_velocity':
            return {
                'success': self.send_profile_velocity(
                    int(arguments['velocity_rpm']),
                    int(arguments.get('acceleration_rpm_s', 1000)),
                    int(arguments.get('deceleration_rpm_s', 1000)),
                )
            }
        if command == 'current_mode':
            return {'success': self.send_current_mode(int(arguments['current_ma']))}
        if command == 'single_pdo':
            command_obj = SingleAxisCommand(
                enable=bool(arguments.get('enable', True)),
                release_brake=bool(arguments.get('release_brake', True)),
                clear_error=bool(arguments.get('clear_error', False)),
                control_mode=int(arguments['control_mode']),
                target_param_1=int(arguments.get('target_param_1', 0)),
                target_param_2=int(arguments.get('target_param_2', 0)),
                feedforward=int(arguments.get('feedforward', 0)),
            )
            return {'success': self.send_single_axis_command(command_obj)}
        if command == 'mit':
            command_obj = MitCommand(
                enable=bool(arguments.get('enable', True)),
                release_brake=bool(arguments.get('release_brake', True)),
                clear_error=bool(arguments.get('clear_error', False)),
                target_position_deg=float(arguments.get('target_position_deg', 0.0)),
                target_velocity_rpm=float(arguments.get('target_velocity_rpm', 0.0)),
                kp=float(arguments.get('kp', 0.0)),
                kd=float(arguments.get('kd', 0.0)),
                target_torque_nm=float(arguments.get('target_torque_nm', 0.0)),
                position_limit_deg=float(arguments.get('position_limit_deg', self.config.mit_position_limit_deg)),
                velocity_limit_rpm=float(arguments.get('velocity_limit_rpm', self.config.mit_velocity_limit_rpm)),
                torque_limit_nm=float(arguments.get('torque_limit_nm', self.config.mit_torque_limit_nm)),
            )
            return {'success': self.send_mit_command(command_obj)}
        raise KeyError(f'unsupported command: {command}')

    def _apply_sdo_response(self, response) -> None:
        if response.is_abort:
            self.state.error_code = response.abort_code & 0xFFFF
            self.state.fault = True
            self.state.touch_sdo()
            return
        self.state.touch_sdo()
        index = response.index
        if index == INDEX_ACTUAL_CURRENT and response.raw_data:
            self.state.current_ma = float(response.as_unsigned())
        elif index == INDEX_ACTUAL_POSITION and response.raw_data:
            self.state.position_deg = position_counts_to_degrees(response.as_signed())
        elif index == INDEX_ACTUAL_VELOCITY and response.raw_data:
            self.state.velocity_rpm = response.as_signed() / 10.0
        elif index == INDEX_ERROR_CODE and response.raw_data:
            self.state.error_code = response.as_unsigned()
            self.state.fault = self.state.error_code != 0
        elif index == INDEX_STATUS_WORD and response.raw_data:
            self.state.status_word = response.as_unsigned()
            status_info = decode_status_word(self.state.status_word)
            self.state.enabled = status_info['operation_enabled']
            self.state.fault = status_info['fault']
        elif index == INDEX_MOS_TEMPERATURE and response.raw_data:
            self.state.mos_temperature_c = response.as_signed() / 10.0
        elif index == INDEX_MOTOR_TEMPERATURE and response.raw_data:
            self.state.coil_temperature_c = response.as_signed() / 10.0
        elif index == INDEX_MANUFACTURER_DEVICE_NAME:
            self.state.manufacturer_name = response.as_ascii()
        elif index == INDEX_DEVICE_TYPE:
            self.state.joint_model = response.as_ascii() or str(response.as_unsigned())
        elif index == INDEX_SOFTWARE_VERSION:
            self.state.firmware_version = response.as_ascii() or str(response.as_unsigned())
        elif index == INDEX_HARDWARE_VERSION:
            self.state.hardware_version = response.as_ascii() or str(response.as_unsigned())
