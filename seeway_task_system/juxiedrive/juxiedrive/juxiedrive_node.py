#!/usr/bin/env python3
"""ROS2 node for JuxieDrive joint modules."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import yaml

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState

from juxiedrive.msg import JointCommand, JointStatus
from juxiedrive.srv import ExecuteCommand, ReadObject, WriteObject

from .can_driver import CanDriver, CanFrame
from .joint_device import JointDevice
from .protocol import (
    MODEL_SPECS,
    SingleAxisCommand,
    MitCommand,
    build_multi_axis_fd_command,
    degrees_to_position_counts,
)
from .state_types import JointConfig


class JuxieDriveNode(Node):
    """Bridge between ROS2 topics/services and the JuxieDrive CAN protocol."""

    def __init__(self) -> None:
        super().__init__('juxiedrive_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 1_000_000)
        self.declare_parameter('can_data_bitrate', 5_000_000)
        self.declare_parameter('fd_enabled', True)
        self.declare_parameter('feedback_rate_hz', 20.0)
        self.declare_parameter('offline_timeout_sec', 1.0)
        self.declare_parameter('auto_start_node', True)
        self.declare_parameter('auto_sync_feedback', True)
        self.declare_parameter('joints_config', '')

        joints_config = self.get_parameter('joints_config').value
        self._joint_configs = self._load_joint_configs(joints_config)
        self._transport = CanDriver(
            channel=str(self.get_parameter('can_channel').value),
            bitrate=int(self.get_parameter('can_bitrate').value),
            data_bitrate=int(self.get_parameter('can_data_bitrate').value),
            fd_enabled=bool(self.get_parameter('fd_enabled').value),
            logger=self.get_logger(),  # type: ignore[arg-type]
        )
        self._transport.register_callback(self._handle_can_frame)
        self._devices: Dict[str, JointDevice] = {
            config.name: JointDevice(config=config, transport=self._transport, logger=self.get_logger())
            for config in self._joint_configs
        }
        self._device_by_id = {device.config.node_id: device for device in self._devices.values()}

        if not self._transport.open():
            self.get_logger().warning('CAN transport failed to open; services/topics remain available for dry-run use.')

        if bool(self.get_parameter('auto_start_node').value):
            for device in self._devices.values():
                device.start_node()

        self._command_sub = self.create_subscription(JointCommand, '/juxiedrive/command', self._command_callback, 10)
        self._status_pub = self.create_publisher(JointStatus, '/juxiedrive/status', 10)
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/juxiedrive/diagnostics', 10)

        self._execute_srv = self.create_service(ExecuteCommand, '/juxiedrive/execute_command', self._execute_command)
        self._read_srv = self.create_service(ReadObject, '/juxiedrive/read_object', self._read_object)
        self._write_srv = self.create_service(WriteObject, '/juxiedrive/write_object', self._write_object)

        feedback_rate_hz = float(self.get_parameter('feedback_rate_hz').value)
        self._offline_timeout_sec = float(self.get_parameter('offline_timeout_sec').value)
        self._auto_sync_feedback = bool(self.get_parameter('auto_sync_feedback').value)
        self.create_timer(max(0.01, 1.0 / feedback_rate_hz), self._publish_outputs)
        if self._auto_sync_feedback:
            self.create_timer(max(0.01, 1.0 / feedback_rate_hz), self._request_feedback)

        self.get_logger().info('JuxieDrive node started with %d joints', len(self._devices))

    def _default_joints_path(self) -> Path:
        return Path(__file__).resolve().parents[1] / 'config' / 'joints.yaml'

    def _load_joint_configs(self, config_path: str) -> List[JointConfig]:
        path = Path(config_path) if config_path else self._default_joints_path()
        if not path.exists():
            self.get_logger().warning('Joint configuration %s not found; using a single generic joint.', path)
            return [JointConfig(name='joint1', node_id=1)]
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        items = data.get('joints', [])
        configs: List[JointConfig] = []
        for item in items:
            model_name = str(item.get('model', 'generic')).lower()
            spec = MODEL_SPECS.get(model_name, MODEL_SPECS['generic'])
            configs.append(
                JointConfig(
                    name=str(item['name']),
                    node_id=int(item['node_id']),
                    model=model_name,
                    can_protocol=str(item.get('can_protocol', 'canfd')),
                    reduction_ratio=float(item.get('reduction_ratio', spec.reduction_ratio)),
                    min_position_deg=float(item.get('min_position_deg', -180.0)),
                    max_position_deg=float(item.get('max_position_deg', 180.0)),
                    mit_position_limit_deg=float(item.get('mit_position_limit_deg', 180.0)),
                    mit_velocity_limit_rpm=float(item.get('mit_velocity_limit_rpm', spec.mit_velocity_limit_rpm)),
                    mit_torque_limit_nm=float(item.get('mit_torque_limit_nm', spec.mit_torque_limit_nm)),
                )
            )
        return configs or [JointConfig(name='joint1', node_id=1)]

    def _handle_can_frame(self, frame: CanFrame) -> None:
        node_id = frame.arbitration_id & 0x7F
        device = self._device_by_id.get(node_id)
        if device is not None:
            device.handle_frame(frame)

    def _command_callback(self, msg: JointCommand) -> None:
        device = self._devices.get(msg.joint_name)
        if device is None:
            self.get_logger().error('Unknown joint command target: %s', msg.joint_name)
            return

        if msg.control_mode == JointCommand.MODE_MIT:
            command = MitCommand(
                enable=msg.enable,
                release_brake=msg.release_brake,
                clear_error=msg.clear_error,
                target_position_deg=msg.target_position_deg,
                target_velocity_rpm=msg.target_velocity_rpm,
                kp=msg.kp,
                kd=msg.kd,
                target_torque_nm=msg.target_torque_nm,
                position_limit_deg=device.config.mit_position_limit_deg,
                velocity_limit_rpm=device.config.mit_velocity_limit_rpm,
                torque_limit_nm=device.config.mit_torque_limit_nm,
            )
            device.send_mit_command(command)
            return

        if msg.control_mode in (JointCommand.MODE_PROFILE_POSITION, JointCommand.MODE_CSP):
            target_param_1 = degrees_to_position_counts(msg.target_position_deg)
            target_param_2 = int(max(0.0, msg.profile_acceleration_rpm_s))
            feedforward = int(msg.feedforward)
        elif msg.control_mode in (JointCommand.MODE_PROFILE_VELOCITY, JointCommand.MODE_CSV):
            target_param_1 = int(msg.target_velocity_rpm)
            target_param_2 = int(max(0.0, msg.profile_acceleration_rpm_s))
            feedforward = int(msg.feedforward)
        else:
            target_param_1 = int(msg.target_current_ma)
            target_param_2 = 0
            feedforward = int(msg.feedforward)

        command = SingleAxisCommand(
            enable=msg.enable,
            release_brake=msg.release_brake,
            clear_error=msg.clear_error,
            control_mode=int(msg.control_mode),
            target_param_1=target_param_1,
            target_param_2=target_param_2,
            feedforward=feedforward,
        )
        device.send_single_axis_command(command)

    def _request_feedback(self) -> None:
        for device in self._devices.values():
            device.sync_feedback()

    def _publish_outputs(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = now_msg
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now_msg

        for device in self._devices.values():
            state = device.state
            state.refresh_online(self._offline_timeout_sec)

            status_msg = JointStatus()
            status_msg.stamp = now_msg
            status_msg.joint_name = state.joint_name
            status_msg.control_mode = state.control_mode
            status_msg.online = state.online
            status_msg.enabled = state.enabled
            status_msg.brake_released = state.brake_released
            status_msg.fault = state.fault
            status_msg.target_reached = state.target_reached
            status_msg.position_deg = state.position_deg
            status_msg.velocity_rpm = state.velocity_rpm
            status_msg.current_ma = state.current_ma
            status_msg.coil_temperature_c = state.coil_temperature_c
            status_msg.mos_temperature_c = state.mos_temperature_c
            status_msg.error_code = state.error_code
            status_msg.status_word = state.status_word
            status_msg.manufacturer_name = state.manufacturer_name
            status_msg.joint_model = state.joint_model
            status_msg.firmware_version = state.firmware_version
            status_msg.hardware_version = state.hardware_version
            self._status_pub.publish(status_msg)

            joint_state.name.append(state.joint_name)
            joint_state.position.append(math.radians(state.position_deg))
            joint_state.velocity.append(state.velocity_rpm * 2.0 * math.pi / 60.0)
            joint_state.effort.append(state.current_ma / 1000.0)

            diagnostic = DiagnosticStatus()
            diagnostic.name = f'juxiedrive/{state.joint_name}'
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = 'online'
            if not state.online:
                diagnostic.level = DiagnosticStatus.STALE
                diagnostic.message = 'offline'
            elif state.fault or state.error_code:
                diagnostic.level = DiagnosticStatus.ERROR
                diagnostic.message = f'fault=0x{state.error_code:04X}'
            diagnostic.values = [
                KeyValue(key='position_deg', value=f'{state.position_deg:.3f}'),
                KeyValue(key='velocity_rpm', value=f'{state.velocity_rpm:.3f}'),
                KeyValue(key='current_ma', value=f'{state.current_ma:.3f}'),
                KeyValue(key='coil_temperature_c', value=f'{state.coil_temperature_c:.3f}'),
                KeyValue(key='mos_temperature_c', value=f'{state.mos_temperature_c:.3f}'),
                KeyValue(key='status_word', value=f'0x{state.status_word:04X}'),
            ]
            diagnostics.status.append(diagnostic)

        self._joint_state_pub.publish(joint_state)
        self._diagnostics_pub.publish(diagnostics)

    def _resolve_device(self, joint_name: str) -> Optional[JointDevice]:
        if not joint_name:
            return None
        return self._devices.get(joint_name)

    def _execute_command(self, request: ExecuteCommand.Request, response: ExecuteCommand.Response) -> ExecuteCommand.Response:
        try:
            arguments = json.loads(request.arguments_json) if request.arguments_json else {}
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = f'invalid JSON arguments: {exc}'
            response.response_json = '{}'
            return response

        if request.command == 'broadcast':
            try:
                commands = []
                for item in arguments.get('commands', []):
                    joint_name = str(item['joint_name'])
                    device = self._devices[joint_name]
                    commands.append((
                        device.config.node_id,
                        SingleAxisCommand(
                            enable=bool(item.get('enable', True)),
                            release_brake=bool(item.get('release_brake', True)),
                            clear_error=bool(item.get('clear_error', False)),
                            control_mode=int(item['control_mode']),
                            target_param_1=int(item.get('target_param_1', 0)),
                            target_param_2=int(item.get('target_param_2', 0)),
                            feedforward=int(item.get('feedforward', 0)),
                        ),
                    ))
                arbitration_id, data, is_fd = build_multi_axis_fd_command(commands)
                response.success = self._transport.send(arbitration_id, data, is_fd=is_fd)
                response.message = 'broadcast sent' if response.success else 'failed to send broadcast'
                response.response_json = json.dumps({'count': len(commands)})
                return response
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                response.response_json = '{}'
                return response

        device = self._resolve_device(request.joint_name)
        if device is None:
            response.success = False
            response.message = f'unknown joint: {request.joint_name}'
            response.response_json = '{}'
            return response

        try:
            result = device.execute_named_command(request.command, arguments)
            response.success = bool(result.get('success', True))
            response.message = 'ok' if response.success else 'command failed'
            response.response_json = json.dumps(result, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.response_json = '{}'
        return response

    def _read_object(self, request: ReadObject.Request, response: ReadObject.Response) -> ReadObject.Response:
        device = self._resolve_device(request.joint_name)
        if device is None:
            response.success = False
            response.message = f'unknown joint: {request.joint_name}'
            return response
        sdo_response = device.read_object(request.index, request.subindex)
        if sdo_response is None:
            response.success = False
            response.message = 'no response'
            return response
        response.success = sdo_response.success
        response.message = 'abort' if sdo_response.is_abort else 'ok'
        response.raw_data = list(sdo_response.raw_data)
        response.signed_value = sdo_response.as_signed() if sdo_response.raw_data else 0
        response.unsigned_value = sdo_response.as_unsigned() if sdo_response.raw_data else 0
        response.ascii_value = sdo_response.as_ascii() if sdo_response.raw_data else ''
        return response

    def _write_object(self, request: WriteObject.Request, response: WriteObject.Response) -> WriteObject.Response:
        device = self._resolve_device(request.joint_name)
        if device is None:
            response.success = False
            response.message = f'unknown joint: {request.joint_name}'
            return response
        try:
            value = request.signed_value if request.signed_data else request.unsigned_value
            response.success = device.write_object(
                request.index,
                request.subindex,
                request.width,
                int(value),
                signed=bool(request.signed_data),
            )
            response.message = 'ok' if response.success else 'write failed'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def destroy_node(self) -> None:
        self._transport.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JuxieDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
