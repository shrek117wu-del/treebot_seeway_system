#!/usr/bin/env python3
"""
Task Manager Node - 发布任务命令到系统
负责创建和发布清洁任务
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import uuid
from datetime import datetime

from seeway_task_msgs.msg import TaskCommand, TaskStatus


class TaskManagerNode(Node):
    """任务管理节点 - 创建、验证和发布任务"""

    def __init__(self):
        super().__init__('task_manager_node')
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher
        self.task_pub = self.create_publisher(
            TaskCommand, '/sys_task_cmd', qos
        )
        
        # Subscriber
        self.status_sub = self.create_subscription(
            TaskStatus, '/task_status_feedback', self.status_callback, qos
        )
        
        # 任务历史记录
        self.task_history = {}
        self.active_tasks = {}
        
        # 预定义任务库
        self.task_library = {
            'clean_toilet': {'duration': 180, 'force': 100},
            'clean_sink': {'duration': 120, 'force': 80},
            'clean_urinal': {'duration': 90, 'force': 70},
            'clean_floor': {'duration': 240, 'force': 60},
            'clean_wall': {'duration': 150, 'force': 75},
        }
        
        self.get_logger().info('Task Manager Node initialized')
    
    def create_task_command(self, own: str, task: str, param: dict = None) -> TaskCommand:
        """创建任务命令消息"""
        cmd = TaskCommand()
        cmd.own = own
        cmd.task = task
        
        # 如果没有参数，使用默认参数
        if param is None:
            param = {}
        
        # 将参数转换为 JSON 字符串
        cmd.param = json.dumps(param)
        
        return cmd
    
    def create_task_from_json(self, json_str: str) -> TaskCommand:
        """从 JSON 字符串创建任务"""
        try:
            data = json.loads(json_str)
            return self.create_task_command(
                own=data.get('own', 'vla'),
                task=data.get('task', ''),
                param=data.get('param', {})
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
            return None
    
    def validate_task_command(self, cmd: TaskCommand) -> bool:
        """验证任务命令"""
        if not cmd.own:
            self.get_logger().warn('Task command missing "own" field')
            return False
        
        if not cmd.task:
            self.get_logger().warn('Task command missing "task" field')
            return False
        
        try:
            param = json.loads(cmd.param) if cmd.param else {}
        except json.JSONDecodeError:
            self.get_logger().warn('Task command param is not valid JSON')
            return False
        
        return True
    
    def publish_task(self, cmd: TaskCommand) -> bool:
        """发布任务命令"""
        if not self.validate_task_command(cmd):
            return False
        
        task_id = str(uuid.uuid4())[:8]
        self.active_tasks[task_id] = {
            'command': cmd,
            'timestamp': datetime.now(),
            'status': 'PUBLISHED'
        }
        
        self.task_pub.publish(cmd)
        self.get_logger().info(
            f'Task published: own={cmd.own}, task={cmd.task}, id={task_id}'
        )
        
        return True
    
    def publish_task_from_json(self, json_str: str) -> bool:
        """从 JSON 字符串发布任务"""
        cmd = self.create_task_from_json(json_str)
        if cmd is None:
            return False
        return self.publish_task(cmd)
    
    def publish_predefined_task(self, task_name: str, own: str = 'vla') -> bool:
        """发布预定义的任务"""
        if task_name not in self.task_library:
            self.get_logger().error(f'Task "{task_name}" not found in library')
            return False
        
        params = self.task_library[task_name].copy()
        params['location'] = task_name.replace('clean_', '')
        
        cmd = self.create_task_command(own, task_name, params)
        return self.publish_task(cmd)
    
    def status_callback(self, msg: TaskStatus) -> None:
        """任务状态反馈回调"""
        self.get_logger().info(
            f'Task Status: id={msg.task_id}, status={msg.status}, progress={msg.progress}%'
        )
    
    def get_task_status(self, task_id: str) -> dict:
        """获取任务状态"""
        return self.active_tasks.get(task_id, None)
    
    def list_active_tasks(self) -> list:
        """列表所有活跃任务"""
        return list(self.active_tasks.keys())
    
    def get_task_statistics(self) -> dict:
        """获取任务统计信息"""
        return {
            'total_tasks': len(self.task_history),
            'active_tasks': len(self.active_tasks),
            'task_library_size': len(self.task_library)
        }


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    
    # 发布示例任务
    # node.publish_predefined_task('clean_toilet')
    # node.publish_predefined_task('clean_sink')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
