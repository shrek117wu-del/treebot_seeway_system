#!/usr/bin/env python3
"""
Task Publisher Example - 任务发布示例程序
演示如何发布预定义和自定义任务
"""

import rclpy
from rclpy.node import Node
import json
import time

from seeway_task_msgs.msg import TaskCommand


class TaskPublisherExample(Node):
    """任务发布示例节点"""

    def __init__(self):
        super().__init__('task_publisher_example')
        
        self.pub = self.create_publisher(TaskCommand, '/sys_task_cmd', 10)
        self.get_logger().info('Task Publisher Example started')
        
        # 启动发布任务
        self.create_timer(2.0, self.publish_task_sequence)
        self.task_index = 0
        
        # 预定义的任务序列
        self.task_sequence = [
            {
                'own': 'vla',
                'task': 'clean_toilet',
                'param': {'location': 'toilet', 'force': 100}
            },
            {
                'own': 'vla',
                'task': 'clean_sink',
                'param': {'location': 'sink', 'force': 80}
            },
            {
                'own': 'vla',
                'task': 'clean_urinal',
                'param': {'location': 'urinal', 'force': 70}
            },
            {
                'own': 'vla',
                'task': 'clean_floor',
                'param': {'location': 'floor', 'force': 60}
            },
            {
                'own': 'vla',
                'task': 'clean_wall',
                'param': {'location': 'wall', 'force': 75}
            },
        ]
    
    def publish_task_sequence(self):
        """发布任务序列"""
        if self.task_index >= len(self.task_sequence):
            self.get_logger().info('All tasks published. Stopping.')
            raise KeyboardInterrupt
        
        task_config = self.task_sequence[self.task_index]
        
        msg = TaskCommand()
        msg.own = task_config['own']
        msg.task = task_config['task']
        msg.param = json.dumps(task_config['param'])
        
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published task {self.task_index + 1}: {task_config["task"]}'
        )
        
        self.task_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TaskPublisherExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
