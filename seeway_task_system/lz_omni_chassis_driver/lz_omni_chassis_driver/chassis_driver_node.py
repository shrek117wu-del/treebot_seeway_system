#!/usr/bin/env python3
"""
Chassis Driver Node for LZ_OMNI Omnidirectional Chassis
Subscribes to /cmd_vel (geometry_msgs/Twist) and converts velocity commands
to LZ_OMNI chassis protocol frames transmitted over UART or CAN.

Speed conversion:
  ROS linear  (m/s)   → chassis (mm/s):       × 1000
  ROS angular (rad/s) → chassis (0.001 rad/s): × 1000

Protocol limits:
  Linear-X / Linear-Y: -2000 .. 2000 mm/s
  Angular-Speed:        -6870 .. 6870  (× 0.001 rad/s)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist

from .uart_driver import UartDriver
from .can_driver import CanDriver


class ChassisDriverNode(Node):
    """
    ROS2 node that bridges /cmd_vel to the LZ_OMNI chassis.

    Parameters (all declared and readable from chassis_driver_config.yaml):
      comm_type        : 'uart' | 'can'   (default: 'uart')
      -- UART params --
      uart_port        : serial port path  (default: '/dev/ttyUSB0')
      uart_baudrate    : baud rate         (default: 115200)
      -- CAN params --
      can_channel      : SocketCAN iface   (default: 'can0')
      can_bitrate      : CAN bitrate       (default: 500000)
      can_id           : arbitration ID    (default: 1)
      -- Speed limits --
      max_linear_speed_mms  : mm/s         (default: 2000)
      max_angular_speed_mrad: 0.001 rad/s  (default: 6870)
      -- Publish rate --
      cmd_vel_timeout  : seconds before sending zero when no /cmd_vel arrives
                         (default: 0.5)
      publish_rate_hz  : control loop frequency in Hz (default: 50)
    """

    def __init__(self):
        super().__init__('chassis_driver_node')

        # ── Declare parameters ────────────────────────────────────────
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

        # ── Read parameters ───────────────────────────────────────────
        self._comm_type = self.get_parameter('comm_type').value.lower()

        self._max_linear_mms = int(self.get_parameter('max_linear_speed_mms').value)
        self._max_angular_mrad = int(self.get_parameter('max_angular_speed_mrad').value)
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        # ── Latest velocity command ───────────────────────────────────
        self._vx_mms: int = 0
        self._vy_mms: int = 0
        self._w_mrad: int = 0
        self._last_cmd_time = None

        # ── Communication driver ──────────────────────────────────────
        self._driver = None
        self._init_driver()

        # ── QoS ───────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscriber ────────────────────────────────────────────────
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos
        )

        # ── Control loop timer ────────────────────────────────────────
        period = 1.0 / publish_rate
        self._control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'ChassisDriverNode started (comm={self._comm_type}, '
            f'rate={publish_rate:.0f} Hz)'
        )

    # ------------------------------------------------------------------
    # Driver initialisation
    # ------------------------------------------------------------------

    def _init_driver(self) -> None:
        """Initialise either the UART or CAN driver based on configuration."""
        if self._comm_type == 'uart':
            port = self.get_parameter('uart_port').value
            baudrate = int(self.get_parameter('uart_baudrate').value)
            self._driver = UartDriver(
                port=port,
                baudrate=baudrate,
                logger=self.get_logger(),  # type: ignore[arg-type]
            )
        elif self._comm_type == 'can':
            channel = self.get_parameter('can_channel').value
            bitrate = int(self.get_parameter('can_bitrate').value)
            can_id = int(self.get_parameter('can_id').value)
            self._driver = CanDriver(
                channel=channel,
                bitrate=bitrate,
                can_id=can_id,
                logger=self.get_logger(),  # type: ignore[arg-type]
            )
        else:
            self.get_logger().error(
                f"Unknown comm_type '{self._comm_type}'. Use 'uart' or 'can'."
            )
            return

        if not self._driver.open():
            self.get_logger().error(
                f'Failed to open {self._comm_type.upper()} interface. '
                'The node will keep retrying on each control cycle.'
            )

    # ------------------------------------------------------------------
    # /cmd_vel callback
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Convert Twist message to chassis speed units and cache for the control loop."""
        # m/s → mm/s, rad/s → 0.001 rad/s
        vx_mms = int(msg.linear.x * 1000.0)
        vy_mms = int(msg.linear.y * 1000.0)
        w_mrad = int(msg.angular.z * 1000.0)

        # Clamp to protocol limits
        self._vx_mms = max(-self._max_linear_mms, min(self._max_linear_mms, vx_mms))
        self._vy_mms = max(-self._max_linear_mms, min(self._max_linear_mms, vy_mms))
        self._w_mrad = max(-self._max_angular_mrad, min(self._max_angular_mrad, w_mrad))

        self._last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Periodically send the latest velocity to the chassis (≥50 Hz)."""
        # Safety: zero out velocities if no recent /cmd_vel received
        if self._last_cmd_time is not None:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age > self._cmd_vel_timeout:
                if self._vx_mms != 0 or self._vy_mms != 0 or self._w_mrad != 0:
                    self.get_logger().warn(
                        f'/cmd_vel timeout ({age:.2f}s) – sending zero velocity'
                    )
                self._vx_mms = 0
                self._vy_mms = 0
                self._w_mrad = 0

        # Attempt to (re-)open driver if not connected
        if self._driver is not None and not self._driver.is_open():
            self.get_logger().warn('Driver not open – retrying...')
            self._driver.open()
            return

        if self._driver is None:
            return

        self._driver.send_motion(self._vx_mms, self._vy_mms, self._w_mrad)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        """Clean up resources before node destruction."""
        self.get_logger().info('Shutting down ChassisDriverNode...')
        if self._driver is not None:
            # Send stop command before closing
            try:
                self._driver.send_motion(0, 0, 0)
            except Exception:
                pass
            self._driver.close()
        super().destroy_node()


def main(args=None):
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
