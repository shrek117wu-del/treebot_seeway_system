#!/usr/bin/env python3
"""
Operator Gamepad Node (Zenoh Scheme A)
Reads a connected Gamepad/Joystick using pygame and publishes
sensor_msgs/Joy to /teleop_joy in the local ROS 2 network.
Zenoh-bridge-dds then transparently forwards this to the robot via the cloud.

Prerequisites:
  pip3 install pygame
  sudo apt install ros-humble-sensor-msgs
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import sys

PUBLISH_HZ = 50  # 50Hz control loop – good balance of responsiveness vs. bandwidth

class OperatorGamepadNode(Node):
    def __init__(self):
        super().__init__('operator_gamepad_node')

        self.publisher_ = self.create_publisher(Joy, '/teleop_joy', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_joy)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No gamepad detected! Please plug in a controller.')
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(
            f'Gamepad connected: "{self.joystick.get_name()}" '
            f'(Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()})'
        )

    def publish_joy(self):
        # Required to process OS-level gamepad events
        pygame.event.pump()

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Read all axes (typically left_x, left_y, right_x, right_y, L2, R2)
        msg.axes = [
            self.joystick.get_axis(i)
            for i in range(self.joystick.get_numaxes())
        ]
        # Read all buttons
        msg.buttons = [
            int(self.joystick.get_button(i))
            for i in range(self.joystick.get_numbuttons())
        ]

        self.publisher_.publish(msg)

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorGamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
