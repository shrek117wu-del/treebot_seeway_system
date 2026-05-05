#!/usr/bin/env python3
"""
Nav2 Client Node - 与导航栈通信
负责发送导航目标和监听导航反馈
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math


class Nav2ClientNode(Node):
    """Nav2 导航客户端节点"""

    def __init__(self):
        super().__init__('nav2_client_node')
        
        # 参数
        self.declare_parameter('goal_threshold', 0.2)
        self.declare_parameter('planning_timeout', 60.0)
        self.declare_parameter('execution_timeout', 300.0)
        
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value
        
        # Action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 状态
        self.current_goal_handle = None
        self.navigation_state = 'IDLE'
        self.current_goal_id = None
        
        # 等待服务连接
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 Client Node initialized')
    
    def create_pose_from_xy(self, x: float, y: float, theta: float = 0.0) -> PoseStamped:
        """从 XY 坐标和朝向创建姿态"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将角度转换为四元数
        quat = self.euler_to_quaternion(0, 0, theta)
        pose.pose.orientation = quat
        
        return pose
    
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """欧拉角转四元数"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat
    
    def send_navigation_goal(self, goal_pose: PoseStamped, goal_id: str = None) -> bool:
        """发送导航目标"""
        if not self.nav_client.server_is_ready():
            self.get_logger().error('Navigation server not ready')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = ''
        
        self.current_goal_id = goal_id
        self.navigation_state = 'PLANNING'
        
        self.get_logger().info(
            f'Sending navigation goal: x={goal_pose.pose.position.x}, '
            f'y={goal_pose.pose.position.y}'
        )
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.navigation_state = 'FAILED'
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Navigation goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """目标结果回调"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded')
            self.navigation_state = 'SUCCEEDED'
        else:
            self.get_logger().warn(f'Navigation goal failed with status: {status}')
            self.navigation_state = 'FAILED'
    
    def cancel_navigation(self) -> bool:
        """取消导航"""
        if self.current_goal_handle is None:
            return False
        
        self.get_logger().info('Canceling navigation goal')
        future = self.current_goal_handle.cancel_goal_async()
        self.navigation_state = 'CANCELLED'
        return True
    
    def get_navigation_state(self) -> str:
        """获取导航状态"""
        return self.navigation_state
    
    def is_navigating(self) -> bool:
        """检查是否正在导航"""
        return self.navigation_state in ['PLANNING', 'EXECUTING']
    
    def send_navigation_goal_xy(self, x: float, y: float, theta: float = 0.0, 
                                goal_id: str = None) -> bool:
        """从 XY 坐标发送导航目标（便捷方法）"""
        goal_pose = self.create_pose_from_xy(x, y, theta)
        return self.send_navigation_goal(goal_pose, goal_id)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
