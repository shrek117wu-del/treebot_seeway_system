#!/usr/bin/env python3
"""
Xbox Controller Node - 订阅手柄输入并转发到底盘
处理 Xbox 360 手柄控制
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class XboxControllerNode(Node):
    """Xbox 手柄控制节点"""

    def __init__(self):
        super().__init__('xbox_controller_node')
        
        # 参数
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('turbo_multiplier', 1.5)
        self.declare_parameter('exponential_scale', 2.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        
        self.deadzone = self.get_parameter('deadzone').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        self.exponential_scale = self.get_parameter('exponential_scale').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        
        # 状态
        self.enabled = False
        self.turbo_mode = False
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscriber - 手柄输入
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, qos
        )
        
        # Publisher - 底盘速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos
        )
        
        # 定时器 - 超时保护
        self.last_joy_time = None
        self.create_timer(0.1, self.timeout_check)
        
        self.get_logger().info('Xbox Controller Node initialized')
    
    def apply_deadzone(self, value: float) -> float:
        """应用死区处理"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def apply_exponential_scale(self, value: float) -> float:
        """应用指数缩放（平顺非线性控制）"""
        sign = 1 if value >= 0 else -1
        return sign * (abs(value) ** self.exponential_scale)
    
    def joy_callback(self, msg: Joy) -> None:
        """手柄输入回调"""
        self.last_joy_time = self.get_clock().now()
        
        # 按钮映射（Xbox 360）
        # LB (4) - 启用/禁用
        if msg.buttons[4] == 1:
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Controller {status}')
        
        # RB (5) - 涡轮加速
        self.turbo_mode = (msg.buttons[5] == 1)
        
        if not self.enabled:
            # 发送零速度
            self.send_cmd_vel(0.0, 0.0)
            return
        
        # 摇杆映射
        # 左摇杆 Y 轴 (1) - 前进/后退
        # 左摇杆 X 轴 (0) - 左转/右转
        # DPAD Y (7) - 精细前后
        # DPAD X (6) - 精细左右
        
        left_y = self.apply_deadzone(msg.axes[1])  # 前进/后退
        left_x = self.apply_deadzone(msg.axes[0])  # 左右转
        dpad_y = msg.axes[7]  # 精细前后
        dpad_x = msg.axes[6]  # 精细左右
        
        # 合并摇杆和 DPAD 输入
        linear_x = left_y + dpad_y * 0.3
        angular_z = left_x + dpad_x * 0.3
        
        # 应用指数缩放
        linear_x = self.apply_exponential_scale(linear_x)
        angular_z = self.apply_exponential_scale(angular_z)
        
        # 应用速度限制
        linear_x = max(-1.0, min(1.0, linear_x))
        angular_z = max(-1.0, min(1.0, angular_z))
        
        # 应用涡轮加速
        speed_multiplier = self.turbo_multiplier if self.turbo_mode else 1.0
        
        # 计算最终速度
        final_linear = linear_x * self.max_linear_speed * speed_multiplier
        final_angular = angular_z * self.max_angular_speed * speed_multiplier
        
        # 发送命令
        self.send_cmd_vel(final_linear, final_angular)
    
    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """发送底盘速度命令"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        
        self.cmd_vel_pub.publish(msg)
    
    def timeout_check(self) -> None:
        """超时保护检查"""
        if self.last_joy_time is None:
            return
        
        now = self.get_clock().now()
        time_diff = (now - self.last_joy_time).nanoseconds / 1e9
        
        if time_diff > self.cmd_vel_timeout and self.enabled:
            self.get_logger().warn('Joystick timeout - stopping robot')
            self.send_cmd_vel(0.0, 0.0)
            self.enabled = False


def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
