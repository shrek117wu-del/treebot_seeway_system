#!/usr/bin/env python3
"""
Xbox 360 Controller Node for Seeway Robot
Subscribes to /joy topic and publishes speed commands to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class XboxControllerNode(Node):
    """
    Handles Xbox 360 controller input and publishes to /cmd_vel
    Subscribes to: /joy
    Publishes to: /cmd_vel
    """

    def __init__(self):
        super().__init__('xbox_controller_node')

        # QoS Profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        # Subscribers
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile
        )

        # Declare parameters
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('turbo_multiplier', 1.5)
        self.declare_parameter('exponential_scale', 2.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # Get parameters
        self.deadzone = self.get_parameter('deadzone').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        self.exponential_scale = self.get_parameter('exponential_scale').value

        # State
        self.enabled = False
        self.turbo_mode = False
        self.last_cmd_time = None

        # Timeout timer
        self.create_timer(0.1, self.timeout_check)

        self.get_logger().info(
            f'Xbox Controller Node initialized\n'
            f'  Deadzone: {self.deadzone}\n'
            f'  Max linear speed: {self.max_linear_speed} m/s\n'
            f'  Max angular speed: {self.max_angular_speed} rad/s\n'
            f'  Turbo multiplier: {self.turbo_multiplier}x'
        )

    def apply_deadzone(self, value: float) -> float:
        """
        Apply deadzone to joystick input
        
        Args:
            value: Raw input value (-1.0 to 1.0)
            
        Returns:
            Value with deadzone applied
        """
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale the value back to full range
        if value > 0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return (value + self.deadzone) / (1.0 - self.deadzone)

    def apply_exponential_scaling(self, value: float) -> float:
        """
        Apply exponential scaling for smoother control
        
        Args:
            value: Input value (-1.0 to 1.0)
            
        Returns:
            Exponentially scaled value
        """
        if value > 0:
            return math.pow(value, self.exponential_scale)
        elif value < 0:
            return -math.pow(abs(value), self.exponential_scale)
        else:
            return 0.0

    def joy_callback(self, msg: Joy):
        """
        Handle joystick input
        Xbox 360 controller layout:
        - axes[0]: Left stick X (left/right)
        - axes[1]: Left stick Y (up/down)
        - axes[4]: LT (left trigger)
        - axes[5]: RT (right trigger)
        - buttons[4]: LB (button 4)
        - buttons[5]: RB (button 5)
        
        Args:
            msg: Joy message from /joy topic
        """
        # Check if joystick has correct number of axes and buttons
        if len(msg.axes) < 6 or len(msg.buttons) < 6:
            self.get_logger().warn('Invalid Joy message structure')
            return

        # LB button (button 4) - toggle enable/disable
        if msg.buttons[4] == 1:
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Controller {status}')
            return

        # If not enabled, don't process commands
        if not self.enabled:
            # Publish zero velocity
            self.publish_zero_velocity()
            return

        # RB button (button 5) - turbo mode
        self.turbo_mode = msg.buttons[5] == 1

        # Left joystick Y axis -> linear velocity (forward/backward)
        raw_linear = msg.axes[1]  # Y axis is inverted (up is negative)
        linear = self.apply_deadzone(raw_linear)
        linear = self.apply_exponential_scaling(linear)
        
        # Left joystick X axis -> angular velocity (left/right turn)
        raw_angular = msg.axes[0]
        angular = self.apply_deadzone(raw_angular)
        angular = self.apply_exponential_scaling(angular)

        # Apply speed limits
        linear = linear * self.max_linear_speed
        angular = angular * self.max_angular_speed

        # Apply turbo multiplier
        if self.turbo_mode:
            linear *= self.turbo_multiplier
            angular *= self.turbo_multiplier

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular

        self.cmd_vel_publisher.publish(twist)
        self.last_cmd_time = self.get_clock().now()

    def publish_zero_velocity(self):
        """Publish zero velocity command"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def timeout_check(self):
        """
        Check for command timeout and publish zero velocity if needed
        """
        if self.last_cmd_time is not None:
            time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if time_diff > 0.5:  # 500ms timeout
                self.get_logger().warn('Command timeout, publishing zero velocity')
                self.publish_zero_velocity()
                self.last_cmd_time = None


def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_velocity()  # Ensure robot stops
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
