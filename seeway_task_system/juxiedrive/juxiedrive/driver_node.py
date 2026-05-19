#!/usr/bin/env python3
"""ROS2 node for JuxieDrive CAN/CAN FD actuator control."""

from __future__ import annotations

import json
from collections import deque
from typing import Any, Deque, Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .can_driver import CanDriver
from .protocol import (
    IDX_ACCELERATION,
    IDX_CANFD_DATA_BITRATE,
    IDX_CONTROLWORD,
    IDX_DECELERATION,
    IDX_DISABLE_WATCHDOG_LIMIT,
    IDX_HEARTBEAT_TIME,
    IDX_MODE_OF_OPERATION,
    IDX_NODE_ID,
    IDX_PROFILE_VELOCITY,
    IDX_SOFTWARE_LIMIT,
    IDX_TARGET_CURRENT,
    IDX_TARGET_POSITION,
    IDX_TARGET_VELOCITY,
    IDX_ZERO_CALIBRATION,
    PI_INDEX_MAP,
    CustomFeedback,
    HeartbeatInfo,
    MultiAxisCommand,
    SdoResponse,
    Tpdo1Feedback,
    decode_ascii,
    fault_descriptions,
    mode_name,
)


class DriverNode(Node):
    """Expose documented JuxieDrive commands through a generic JSON topic."""

    def __init__(self) -> None:
        super().__init__('driver_node')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('default_node_id', 1)
        self.declare_parameter('joint_name', 'juxie_joint')
        self.declare_parameter('command_topic', '/juxiedrive/command')
        self.declare_parameter('response_topic', '/juxiedrive/response')
        self.declare_parameter('heartbeat_topic', '/juxiedrive/heartbeat')
        self.declare_parameter('custom_feedback_topic', '/juxiedrive/custom_feedback')
        self.declare_parameter('tpdo_feedback_topic', '/juxiedrive/tpdo_feedback')
        self.declare_parameter('joint_state_topic', '/juxiedrive/joint_state')
        self.declare_parameter('diagnostics_topic', '/juxiedrive/diagnostics')
        self.declare_parameter('query_version_on_start', True)
        self.declare_parameter('auto_start_node', True)
        self.declare_parameter('feedback_publish_rate_hz', 20.0)

        self._default_node_id = int(self.get_parameter('default_node_id').value)
        self._joint_name = str(self.get_parameter('joint_name').value)
        self._pending_strings: Deque[tuple[str, str]] = deque()
        self._latest_custom_feedback: Optional[CustomFeedback] = None
        self._latest_tpdo_feedback: Optional[Tpdo1Feedback] = None
        self._latest_heartbeat: Optional[HeartbeatInfo] = None
        self._version_fields: Dict[str, str] = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._command_sub = self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            qos,
        )
        self._response_pub = self.create_publisher(String, str(self.get_parameter('response_topic').value), qos)
        self._heartbeat_pub = self.create_publisher(String, str(self.get_parameter('heartbeat_topic').value), qos)
        self._custom_feedback_pub = self.create_publisher(String, str(self.get_parameter('custom_feedback_topic').value), qos)
        self._tpdo_feedback_pub = self.create_publisher(String, str(self.get_parameter('tpdo_feedback_topic').value), qos)
        self._joint_state_pub = self.create_publisher(JointState, str(self.get_parameter('joint_state_topic').value), qos)
        self._diag_pub = self.create_publisher(DiagnosticStatus, str(self.get_parameter('diagnostics_topic').value), qos)
        self._version_pub = self.create_publisher(String, '/juxiedrive/version', qos)

        self._driver = CanDriver(
            channel=str(self.get_parameter('can_channel').value),
            bitrate=int(self.get_parameter('can_bitrate').value),
            logger=self.get_logger(),  # type: ignore[arg-type]
        )
        self._driver.set_sdo_callback(self._on_sdo)
        self._driver.set_heartbeat_callback(self._on_heartbeat)
        self._driver.set_custom_feedback_callback(self._on_custom_feedback)
        self._driver.set_tpdo1_callback(self._on_tpdo_feedback)
        if not self._driver.open():
            self.get_logger().error('Failed to open CAN interface; node will keep retrying when commands arrive.')
        if bool(self.get_parameter('auto_start_node').value):
            self._driver.send_nmt_start(self._default_node_id)
        if bool(self.get_parameter('query_version_on_start').value):
            self._driver.send_query_version(self._default_node_id)

        publish_rate = max(1.0, float(self.get_parameter('feedback_publish_rate_hz').value))
        self._feedback_timer = self.create_timer(1.0 / publish_rate, self._publish_feedback)
        self.get_logger().info('JuxieDrive driver node started')

    def _parse_int(self, value: Any, default: int = 0) -> int:
        if value is None:
            return default
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def _node_id(self, payload: Dict[str, Any]) -> int:
        return self._parse_int(payload.get('node_id'), self._default_node_id)

    def _queue_json(self, topic: str, payload: Dict[str, Any]) -> None:
        self._pending_strings.append((topic, json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _publish_feedback(self) -> None:
        while self._pending_strings:
            topic, payload = self._pending_strings.popleft()
            msg = String()
            msg.data = payload
            if topic == 'response':
                self._response_pub.publish(msg)
            elif topic == 'heartbeat':
                self._heartbeat_pub.publish(msg)
            elif topic == 'custom_feedback':
                self._custom_feedback_pub.publish(msg)
            elif topic == 'tpdo_feedback':
                self._tpdo_feedback_pub.publish(msg)
            elif topic == 'version':
                self._version_pub.publish(msg)

        if self._latest_custom_feedback is not None:
            feedback = self._latest_custom_feedback
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [self._joint_name]
            joint_msg.position = [feedback.position_rad]
            joint_msg.velocity = [feedback.velocity_rad_s]
            joint_msg.effort = [feedback.current_a]
            self._joint_state_pub.publish(joint_msg)

            diag = DiagnosticStatus()
            diag.name = self._joint_name
            diag.level = DiagnosticStatus.ERROR if feedback.fault or feedback.error_code else DiagnosticStatus.OK
            faults = fault_descriptions(feedback.error_code)
            diag.message = (
                f'mode={mode_name(feedback.mode)} enabled={feedback.enabled} '
                f'brake_released={feedback.brake_released} temp={feedback.coil_temperature_c:.1f}C '
                f'faults={"|".join(faults) if faults else "none"}'
            )
            self._diag_pub.publish(diag)

    def _on_sdo(self, response: SdoResponse) -> None:
        payload: Dict[str, Any] = {
            'type': 'sdo',
            'node_id': response.node_id,
            'index': hex(response.index),
            'subindex': response.subindex,
            'command': hex(response.command),
            'aborted': response.aborted,
        }
        if response.aborted:
            payload['abort_code'] = hex(response.abort_code or 0)
        else:
            payload['data_hex'] = response.data.hex()
            payload['value_unsigned'] = response.value_unsigned if response.data else 0
            if response.index in (0x1008, 0x1000, 0x100A, 0x1009):
                ascii_value = decode_ascii(response.data)
                payload['ascii'] = ascii_value
                field_names = {
                    0x1008: 'vendor_name',
                    0x1000: 'device_model',
                    0x100A: 'firmware_version',
                    0x1009: 'hardware_version',
                }
                self._version_fields[field_names[response.index]] = ascii_value
                if len(self._version_fields) >= 4:
                    self._queue_json('version', self._version_fields.copy())
        self._queue_json('response', payload)

    def _on_heartbeat(self, info: HeartbeatInfo) -> None:
        self._latest_heartbeat = info
        self._queue_json(
            'heartbeat',
            {'type': 'heartbeat', 'node_id': info.node_id, 'state': hex(info.state), 'bootup': info.is_bootup},
        )

    def _on_custom_feedback(self, feedback: CustomFeedback) -> None:
        self._latest_custom_feedback = feedback
        self._queue_json(
            'custom_feedback',
            {
                'type': 'custom_feedback',
                'node_id': feedback.node_id,
                'position_deg': feedback.position_deg,
                'velocity_rpm': feedback.velocity_rpm,
                'current_ma': feedback.current_ma,
                'error_code': hex(feedback.error_code),
                'faults': fault_descriptions(feedback.error_code),
                'temperature_c': feedback.coil_temperature_c,
                'mode': mode_name(feedback.mode),
                'enabled': feedback.enabled,
                'brake_released': feedback.brake_released,
                'position_reached': feedback.position_reached,
            },
        )

    def _on_tpdo_feedback(self, feedback: Tpdo1Feedback) -> None:
        self._latest_tpdo_feedback = feedback
        self._queue_json(
            'tpdo_feedback',
            {
                'type': 'tpdo1',
                'node_id': feedback.node_id,
                'status_word': hex(feedback.status_word),
                'actual_current_ma': feedback.actual_current_ma,
                'actual_position_counts': feedback.actual_position_counts,
                'actual_position_deg': feedback.actual_position_deg,
            },
        )

    def _send_frames(self, frames) -> None:
        if self._driver is not None and not self._driver.is_open():
            self._driver.open()
        self._driver.send_sequence(frames)

    def _on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON command: {exc}')
            return
        command_type = str(payload.get('type', '')).strip().lower()
        node_id = self._node_id(payload)
        try:
            if command_type == 'nmt_start':
                self._driver.send_nmt_start(node_id)
            elif command_type == 'sync':
                self._driver.send_sync()
            elif command_type == 'sdo_read':
                self._driver.send_sdo_read(node_id, self._parse_int(payload['index']), self._parse_int(payload.get('subindex'), 0))
            elif command_type == 'sdo_write':
                self._driver.send_sdo_write(
                    node_id,
                    self._parse_int(payload['index']),
                    self._parse_int(payload.get('subindex'), 0),
                    self._parse_int(payload.get('value'), 0),
                    self._parse_int(payload.get('size'), 4),
                    bool(payload.get('signed', False)),
                )
            elif command_type == 'query_version':
                self._driver.send_query_version(node_id)
            elif command_type == 'query_state':
                self._driver.send_query_state(node_id)
            elif command_type == 'enable':
                self._driver.send_enable_sequence(node_id)
            elif command_type == 'stop':
                self._send_frames([
                    (0x600 + node_id, bytes([0x2B, IDX_CONTROLWORD & 0xFF, (IDX_CONTROLWORD >> 8) & 0xFF, 0, 0x0F, 0, 0, 0]), False)
                ])
            elif command_type == 'set_node_id':
                self._driver.send_sdo_write(node_id, IDX_NODE_ID, 0, self._parse_int(payload['new_node_id']), 4)
            elif command_type == 'set_zero_position':
                self._driver.send_sdo_write(node_id, IDX_ZERO_CALIBRATION, 0, 1, 4)
            elif command_type == 'disable_watchdog_and_limits':
                self._driver.send_sdo_write(node_id, IDX_DISABLE_WATCHDOG_LIMIT, 0, 1, 4)
            elif command_type == 'set_heartbeat':
                self._driver.send_sdo_write(node_id, IDX_HEARTBEAT_TIME, 0, self._parse_int(payload['heartbeat_ms']), 2)
            elif command_type == 'set_positive_limit':
                self._driver.send_sdo_write(node_id, IDX_SOFTWARE_LIMIT, 2, self._parse_int(payload['value']), 4, signed=True)
            elif command_type == 'set_negative_limit':
                self._driver.send_sdo_write(node_id, IDX_SOFTWARE_LIMIT, 1, self._parse_int(payload['value']), 4, signed=True)
            elif command_type == 'set_canfd_bitrate':
                self._driver.send_sdo_write(node_id, IDX_CANFD_DATA_BITRATE, 0, self._parse_int(payload['value']), 4)
            elif command_type == 'set_mode':
                from .protocol import normalize_mode
                self._driver.send_sdo_write(node_id, IDX_MODE_OF_OPERATION, 0, normalize_mode(payload['mode']), 1, signed=True)
            elif command_type == 'profile_position':
                self._driver.send_profile_position(
                    node_id,
                    self._parse_int(payload['target_position']),
                    self._parse_int(payload.get('profile_velocity'), 10),
                    self._parse_int(payload.get('acceleration'), 2000),
                    self._parse_int(payload.get('deceleration'), 2000),
                    start_motion=bool(payload.get('start_motion', True)),
                )
            elif command_type == 'profile_velocity':
                self._driver.send_profile_velocity(
                    node_id,
                    self._parse_int(payload['target_velocity_rpm']),
                    self._parse_int(payload.get('acceleration'), 1000),
                    self._parse_int(payload.get('deceleration'), 1000),
                )
            elif command_type == 'current':
                self._driver.send_current(node_id, self._parse_int(payload['target_current_ma']))
            elif command_type == 'set_profile_params':
                if 'profile_velocity' in payload:
                    self._driver.send_sdo_write(node_id, IDX_PROFILE_VELOCITY, 0, self._parse_int(payload['profile_velocity']), 4)
                if 'acceleration' in payload:
                    self._driver.send_sdo_write(node_id, IDX_ACCELERATION, 0, self._parse_int(payload['acceleration']), 4)
                if 'deceleration' in payload:
                    self._driver.send_sdo_write(node_id, IDX_DECELERATION, 0, self._parse_int(payload['deceleration']), 4)
            elif command_type == 'set_pi':
                index = PI_INDEX_MAP[str(payload['parameter'])]
                self._driver.send_sdo_write(node_id, index, 0, self._parse_int(payload['value']), 4)
            elif command_type == 'read_pi':
                target = str(payload['parameter'])
                if target == 'max_current':
                    self._driver.send_sdo_read(node_id, 0x2538, 0)
                else:
                    self._driver.send_sdo_read(node_id, PI_INDEX_MAP[target], 0)
            elif command_type == 'read_actual_position':
                self._driver.send_sdo_read(node_id, 0x6064, 0)
            elif command_type == 'read_actual_velocity':
                self._driver.send_sdo_read(node_id, 0x606C, 0)
            elif command_type == 'read_actual_current':
                self._driver.send_sdo_read(node_id, 0x6078, 0)
            elif command_type == 'read_fault':
                self._driver.send_sdo_read(node_id, 0x603F, 0)
            elif command_type == 'read_status_word':
                self._driver.send_sdo_read(node_id, 0x6041, 0)
            elif command_type == 'read_temperatures':
                self._driver.send_sdo_read(node_id, 0x2662, 0)
                self._driver.send_sdo_read(node_id, 0x2663, 0)
            elif command_type == 'configure_pdo':
                self._driver.configure_canopen_pdo(node_id)
            elif command_type == 'rpdo1':
                self._driver.send_rpdo1(
                    node_id,
                    self._parse_int(payload.get('controlword'), 0x000F),
                    self._parse_int(payload.get('target_current_ma'), 0),
                    self._parse_int(payload.get('target_position'), 0),
                )
            elif command_type == 'custom_control':
                self._driver.send_custom_command(
                    node_id,
                    payload['mode'],
                    self._parse_int(payload['target_1']),
                    self._parse_int(payload.get('target_2'), 0),
                    self._parse_int(payload.get('feedforward'), 0),
                    enable=bool(payload.get('enable', True)),
                    release_brake=bool(payload.get('release_brake', True)),
                    clear_error=bool(payload.get('clear_error', False)),
                )
            elif command_type == 'multi_control':
                commands = [
                    MultiAxisCommand(
                        node_id=self._parse_int(item['node_id']),
                        mode=item['mode'],
                        target_1=self._parse_int(item['target_1']),
                        target_2=self._parse_int(item.get('target_2'), 0),
                        feedforward=self._parse_int(item.get('feedforward'), 0),
                        enable=bool(item.get('enable', True)),
                        release_brake=bool(item.get('release_brake', True)),
                        clear_error=bool(item.get('clear_error', False)),
                    )
                    for item in payload.get('commands', [])
                ]
                self._driver.send_custom_multi_command(commands)
            elif command_type == 'mit_control':
                self._driver.send_mit_command(
                    node_id,
                    float(payload.get('position', 0.0)),
                    float(payload.get('velocity', 0.0)),
                    float(payload.get('kp', 0.0)),
                    float(payload.get('kd', 0.0)),
                    float(payload.get('torque', 0.0)),
                    pos_min=float(payload.get('pos_min', -180.0)),
                    pos_max=float(payload.get('pos_max', 180.0)),
                    vel_max=float(payload.get('vel_max', 4000.0)),
                    kp_max=float(payload.get('kp_max', 500.0)),
                    kd_max=float(payload.get('kd_max', 5.0)),
                    torque_max=float(payload.get('torque_max', 30.0)),
                    enable=bool(payload.get('enable', True)),
                    release_brake=bool(payload.get('release_brake', True)),
                    clear_error=bool(payload.get('clear_error', False)),
                )
            else:
                self.get_logger().error(f'Unsupported command type: {command_type}')
        except Exception as exc:
            self.get_logger().error(f'Failed to handle command {command_type}: {exc}')

    def destroy_node(self) -> bool:
        self.get_logger().info('Shutting down JuxieDrive node')
        if self._driver is not None:
            self._driver.close()
        return super().destroy_node()



def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
