#!/usr/bin/env python3
"""
Task Scheduler Node - 核心协调器
管理任务队列、状态转移和导航调度
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import time
from collections import deque
from enum import Enum

from seeway_task_msgs.msg import TaskCommand, TaskStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class TaskState(Enum):
    """任务状态枚举"""
    WAITING = "WAITING"
    NAVIGATING = "NAVIGATING_TO_TARGET"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class CleaningLocation:
    """清洁位置信息"""
    def __init__(self, name, x, y, theta=0.0, duration=120):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.duration = duration


class TaskSchedulerNode(Node):
    """任务调度器节点 - 核心协调器"""

    def __init__(self):
        super().__init__('task_scheduler_node')
        
        # 参数
        self.declare_parameter('goal_threshold', 0.2)
        self.declare_parameter('execution_timeout', 300.0)
        
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.execution_timeout = self.get_parameter('execution_timeout').value
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher
        self.status_pub = self.create_publisher(
            TaskStatus, '/task_status_feedback', qos
        )
        
        # Subscriber - 任务命令
        self.task_sub = self.create_subscription(
            TaskCommand, '/sys_task_cmd', self.task_cmd_callback, qos
        )
        
        # Nav2 Action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 任务队列和状态
        self.task_queue = deque()
        self.current_task = None
        self.task_counter = 0
        
        # 预定义清洁位置
        self.cleaning_locations = {
            'toilet': CleaningLocation('toilet', 1.0, 1.0, 0.0, 180),
            'sink': CleaningLocation('sink', 2.0, 2.0, 0.0, 120),
            'urinal': CleaningLocation('urinal', 3.0, 1.0, 0.0, 90),
            'floor': CleaningLocation('floor', 1.5, 1.5, 0.0, 240),
            'wall': CleaningLocation('wall', 2.5, 2.5, 0.0, 150),
            'base': CleaningLocation('base', 0.0, 0.0, 0.0, 0),
        }
        
        # 定时器
        self.create_timer(0.1, self.task_state_checker)
        
        self.get_logger().info('Task Scheduler Node initialized')
    
    def task_cmd_callback(self, msg: TaskCommand) -> None:
        """任务命令回调"""
        self.task_counter += 1
        task_id = self.task_counter
        
        # 创建任务对象
        task = {
            'id': task_id,
            'own': msg.own,
            'task': msg.task,
            'param': json.loads(msg.param) if msg.param else {},
            'state': TaskState.WAITING,
            'progress': 0,
            'timestamp': self.get_clock().now(),
            'nav_goal_handle': None,
            'start_execution_time': None,
        }
        
        self.task_queue.append(task)
        self.get_logger().info(
            f'Task enqueued: id={task_id}, task={msg.task}, '
            f'queue_size={len(self.task_queue)}'
        )
    
    def task_state_checker(self) -> None:
        """周期性检查任务状态"""
        # 如果没有当前任务，启动下一个
        if self.current_task is None:
            if len(self.task_queue) > 0:
                self.start_next_task()
            return
        
        # 根据任务状态进行处理
        task = self.current_task
        
        if task['state'] == TaskState.NAVIGATING:
            self.check_navigation_status(task)
        
        elif task['state'] == TaskState.EXECUTING:
            self.check_execution_status(task)
    
    def start_next_task(self) -> None:
        """启动下一个任务"""
        if len(self.task_queue) == 0:
            return
        
        task = self.task_queue.popleft()
        self.current_task = task
        
        self.get_logger().info(f'Starting task: id={task["id"]}, task={task["task"]}')
        
        # 获取清洁位置
        location_name = task['param'].get('location', 'floor')
        if location_name in self.cleaning_locations:
            location = self.cleaning_locations[location_name]
            
            # 更新任务状态为导航中
            task['state'] = TaskState.NAVIGATING
            task['progress'] = 10
            self.publish_task_status(task)
            
            # 发送导航目标
            self.send_navigation_goal(task, location)
        else:
            self.get_logger().error(f'Unknown location: {location_name}')
            task['state'] = TaskState.FAILED
            self.publish_task_status(task)
            self.current_task = None
    
    def send_navigation_goal(self, task, location: CleaningLocation) -> None:
        """发送导航目标"""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Navigation server not ready')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_from_xy(
            location.x, location.y, location.theta
        )
        goal_msg.behavior_tree = ''
        
        self.get_logger().info(
            f'Sending navigation goal: x={location.x}, y={location.y}'
        )
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, t=task: self.nav_goal_response_callback(f, t)
        )
    
    def nav_goal_response_callback(self, future, task) -> None:
        """导航目标响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            task['state'] = TaskState.FAILED
            self.publish_task_status(task)
            self.current_task = None
            return
        
        task['nav_goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, t=task: self.nav_goal_result_callback(f, t)
        )
    
    def nav_goal_result_callback(self, future, task) -> None:
        """导航目标结果"""
        status = future.result().status
        
        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            
            # 更新任务状态为执行中
            task['state'] = TaskState.EXECUTING
            task['progress'] = 50
            task['start_execution_time'] = self.get_clock().now()
            self.publish_task_status(task)
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            task['state'] = TaskState.FAILED
            self.publish_task_status(task)
            self.current_task = None
    
    def check_navigation_status(self, task) -> None:
        """检查导航状态"""
        # 这里可以检查 Nav2 的反馈
        pass
    
    def check_execution_status(self, task) -> None:
        """检查执行状态"""
        if task['start_execution_time'] is None:
            return
        
        elapsed = (self.get_clock().now() - task['start_execution_time']).nanoseconds / 1e9
        
        # 获取清洁位置的持续时间
        location_name = task['param'].get('location', 'floor')
        location = self.cleaning_locations.get(location_name)
        
        if location and elapsed >= location.duration:
            self.get_logger().info(
                f'Task execution completed: id={task["id"]}, '
                f'duration={elapsed:.1f}s'
            )
            
            task['state'] = TaskState.COMPLETED
            task['progress'] = 100
            self.publish_task_status(task)
            self.current_task = None
    
    def publish_task_status(self, task) -> None:
        """发布任务状态"""
        msg = TaskStatus()
        msg.task_id = task['id']
        msg.own = task['own']
        msg.task = task['task']
        msg.param = task['param'].__repr__()
        msg.status = task['state'].value
        msg.progress = task['progress']
        msg.timestamp = int(time.time())
        
        self.status_pub.publish(msg)
        self.get_logger().info(
            f'Task status published: id={task["id"]}, '
            f'status={task["state"].value}, progress={task["progress"]}%'
        )
    
    def create_pose_from_xy(self, x: float, y: float, theta: float = 0.0) -> PoseStamped:
        """从 XY 坐标创建姿态"""
        import math
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 欧拉角转四元数
        qx = math.sin(0/2) * math.cos(0/2) * math.cos(theta/2) - \
             math.cos(0/2) * math.sin(0/2) * math.sin(theta/2)
        qy = math.cos(0/2) * math.sin(0/2) * math.cos(theta/2) + \
             math.sin(0/2) * math.cos(0/2) * math.sin(theta/2)
        qz = math.cos(0/2) * math.cos(0/2) * math.sin(theta/2) - \
             math.sin(0/2) * math.sin(0/2) * math.cos(theta/2)
        qw = math.cos(0/2) * math.cos(0/2) * math.cos(theta/2) + \
             math.sin(0/2) * math.sin(0/2) * math.sin(theta/2)
        
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose
    
    def get_task_statistics(self) -> dict:
        """获取任务统计信息"""
        return {
            'queue_size': len(self.task_queue),
            'current_task': self.current_task['id'] if self.current_task else None,
            'locations_count': len(self.cleaning_locations),
        }


def main(args=None):
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
