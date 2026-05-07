#!/usr/bin/env python3
"""ROS2 node that bridges topics and LZ_OMNI UART/CAN chassis protocol."""

from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32MultiArray, String

from .can_driver import CanDriver
from .protocol import AuxInfo, BatteryInfo, ChassisStatus, VersionInfo
from .uart_driver import UartDriver


class ChassisDriverNode(Node):
    """ROS2 chassis driver node supporting motion control and telemetry feedback."""

    def __init__(self) -> None:
        """Declare parameters, initialise transport driver, and create ROS interfaces."""
        super().__init__('chassis_driver_node')

        self.declare_parameter('comm_type', 'uart')

        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('uart_baudrate', 115200)

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('can_id', 1)

        self.declare_parameter('max_linear_speed_mms', 2000)
        self.declare_parameter('max_angular_speed_mrad', 6870)

        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('query_version_on_start', True)
        self.declare_parameter('feedback_publish_rate_hz', 10.0)
        self.declare_parameter('motor_control_topic', '/motor_speeds')

        self._comm_type = self.get_parameter('comm_type').value.lower()
        self._max_linear_mms = int(self.get_parameter('max_linear_speed_mms').value)
        self._max_angular_mrad = int(self.get_parameter('max_angular_speed_mrad').value)
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._feedback_publish_rate_hz = float(self.get_parameter('feedback_publish_rate_hz').value)
        self._motor_control_topic = str(self.get_parameter('motor_control_topic').value)
        self._query_version_on_start = bool(self.get_parameter('query_version_on_start').value)

        self._vx_mms = 0
        self._vy_mms = 0
        self._w_mrad = 0
        self._last_cmd_time = None

        self._latest_battery: Optional[BatteryInfo] = None
        self._latest_chassis: Optional[ChassisStatus] = None
        self._latest_aux: Optional[AuxInfo] = None
        self._latest_version: Optional[VersionInfo] = None
        self._latest_battery_stamp = None
        self._latest_chassis_stamp = None
        self._latest_aux_stamp = None
        self._latest_version_stamp = None

        self._driver = None
        self._init_driver()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, qos)
        self._motor_sub = self.create_subscription(
            Int32MultiArray,
            self._motor_control_topic,
            self._motor_control_callback,
            qos,
        )

        self._battery_pub = self.create_publisher(BatteryState, '/battery_state', qos)
        self._chassis_vel_pub = self.create_publisher(TwistStamped, '/chassis_velocity', qos)
        self._diag_pub = self.create_publisher(DiagnosticStatus, '/chassis_diagnostics', qos)
        self._fw_pub = self.create_publisher(String, '/firmware_version', qos)

        self._control_timer = self.create_timer(1.0 / publish_rate, self._control_loop)
        feedback_period = 1.0 / max(self._feedback_publish_rate_hz, 1e-6)
        self._feedback_timer = self.create_timer(feedback_period, self._publish_feedback)

        self._version_query_timer = None
        if self._query_version_on_start:
            self._version_query_timer = self.create_timer(1.0, self._query_version_once)

        self.get_logger().info(
            f'ChassisDriverNode started (comm={self._comm_type}, '
            f'control_rate={publish_rate:.1f}Hz, feedback_rate={self._feedback_publish_rate_hz:.1f}Hz)'
        )

    def _init_driver(self) -> None:
        """Initialise configured hardware driver and register parser callbacks."""
        if self._comm_type == 'uart':
            self._driver = UartDriver(
                port=self.get_parameter('uart_port').value,
                baudrate=int(self.get_parameter('uart_baudrate').value),
                logger=self.get_logger(),  # type: ignore[arg-type]
            )
        elif self._comm_type == 'can':
            self._driver = CanDriver(
                channel=self.get_parameter('can_channel').value,
                bitrate=int(self.get_parameter('can_bitrate').value),
                can_id=int(self.get_parameter('can_id').value),
                logger=self.get_logger(),  # type: ignore[arg-type]
            )
        else:
            self.get_logger().error(f"Unknown comm_type '{self._comm_type}'. Use 'uart' or 'can'.")
            return

        self._driver.set_battery_info_callback(self._on_battery_info)
        self._driver.set_chassis_status_callback(self._on_chassis_status)
        self._driver.set_aux_info_callback(self._on_aux_info)
        self._driver.set_version_info_callback(self._on_version_info)

        if not self._driver.open():
            self.get_logger().error(
                f'Failed to open {self._comm_type.upper()} interface. '
                'The node will keep retrying on each control cycle.'
            )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Convert ROS Twist to protocol units and cache command for control loop."""
        vx_mms = int(msg.linear.x * 1000.0)
        vy_mms = int(msg.linear.y * 1000.0)
        w_mrad = int(msg.angular.z * 1000.0)

        self._vx_mms = max(-self._max_linear_mms, min(self._max_linear_mms, vx_mms))
        self._vy_mms = max(-self._max_linear_mms, min(self._max_linear_mms, vy_mms))
        self._w_mrad = max(-self._max_angular_mrad, min(self._max_angular_mrad, w_mrad))

        self._last_cmd_time = self.get_clock().now()

    def _motor_control_callback(self, msg: Int32MultiArray) -> None:
        """Send wheel RPM command from ``/motor_speeds`` data ``[m1,m2,m3,m4]``."""
        if self._driver is None:
            return
        if len(msg.data) != 4:
            self.get_logger().error(f'Expected 4 motor speeds, got {len(msg.data)}')
            return
        self._driver.send_motor_control(int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3]))

    def _control_loop(self) -> None:
        """Send periodic motion command and enforce /cmd_vel timeout behavior."""
        if self._last_cmd_time is not None:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age > self._cmd_vel_timeout:
                if self._vx_mms != 0 or self._vy_mms != 0 or self._w_mrad != 0:
                    self.get_logger().warn(f'/cmd_vel timeout ({age:.2f}s) – sending zero velocity')
                self._vx_mms = 0
                self._vy_mms = 0
                self._w_mrad = 0

        if self._driver is not None and not self._driver.is_open():
            self.get_logger().warn('Driver not open – retrying...')
            self._driver.open()
            return

        if self._driver is None:
            return

        self._driver.send_motion(self._vx_mms, self._vy_mms, self._w_mrad)

    def _query_version_once(self) -> None:
        """Send one-time version query after node startup delay."""
        if self._version_query_timer is not None:
            self._version_query_timer.cancel()
            self._version_query_timer = None
        if self._driver is not None:
            self._driver.send_version_query()

    def _on_battery_info(self, info: BatteryInfo) -> None:
        """Store latest battery feedback from hardware callback thread."""
        self._latest_battery_stamp = self.get_clock().now().to_msg()
        self._latest_battery = info

    def _on_chassis_status(self, info: ChassisStatus) -> None:
        """Store latest chassis-state feedback from hardware callback thread."""
        self._latest_chassis_stamp = self.get_clock().now().to_msg()
        self._latest_chassis = info

    def _on_aux_info(self, info: AuxInfo) -> None:
        """Store latest auxiliary feedback from hardware callback thread."""
        self._latest_aux_stamp = self.get_clock().now().to_msg()
        self._latest_aux = info

    def _on_version_info(self, info: VersionInfo) -> None:
        """Store latest firmware-version response from hardware callback thread."""
        self._latest_version_stamp = self.get_clock().now().to_msg()
        self._latest_version = info

    def _publish_feedback(self) -> None:
        """Periodically publish cached battery, status, diagnostics, and version topics."""
        now = self.get_clock().now().to_msg()

        if self._latest_battery is not None:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self._latest_battery_stamp or now
            battery_msg.voltage = float(self._latest_battery.voltage_v)
            battery_msg.current = float(self._latest_battery.current_a)
            battery_msg.percentage = float(self._latest_battery.soc_percent) / 100.0
            battery_msg.present = self._latest_battery.status != 0
            self._battery_pub.publish(battery_msg)

        if self._latest_chassis is not None:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self._latest_chassis_stamp or now
            vel_msg.twist.linear.x = self._latest_chassis.vx_ms
            vel_msg.twist.linear.y = self._latest_chassis.vy_ms
            vel_msg.twist.angular.z = self._latest_chassis.vw_rads
            self._chassis_vel_pub.publish(vel_msg)

        if self._latest_aux is not None:
            diag = DiagnosticStatus()
            diag.level = DiagnosticStatus.OK if self._latest_aux.error_code == 0 else DiagnosticStatus.ERROR
            diag.name = 'chassis'
            diag.message = (
                f'temp={self._latest_aux.temperature_c:.1f}°C '
                f'uptime={self._latest_aux.uptime_ms}ms'
            )
            self._diag_pub.publish(diag)

        if self._latest_version is not None:
            fw = String()
            fw.data = str(self._latest_version)
            self._fw_pub.publish(fw)

    def destroy_node(self) -> None:
        """Send stop command and release hardware resources during shutdown."""
        self.get_logger().info('Shutting down ChassisDriverNode...')
        if self._driver is not None:
            try:
                self._driver.send_motion(0, 0, 0)
            except Exception:
                pass
            self._driver.close()
        super().destroy_node()


def main(args=None) -> None:
    """Run node main loop until shutdown."""
    rclpy.init(args=args)
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
