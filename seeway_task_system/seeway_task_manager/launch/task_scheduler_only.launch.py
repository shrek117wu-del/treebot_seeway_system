#!/usr/bin/env python3
"""
Seeway Task System - Only Scheduler Launch File
仅启动任务调度器（不启动手柄和其他节点）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取配置文件路径
    config_dir = os.path.join(
        FindPackageShare('seeway_task_manager').find('seeway_task_manager'),
        'config'
    )
    
    nav_config = os.path.join(config_dir, 'navigation_config.yaml')
    
    # 创建任务调度器节点
    task_scheduler_node = Node(
        package='seeway_task_manager',
        executable='task_scheduler_node.py',
        name='task_scheduler',
        output='screen',
        parameters=[nav_config]
    )
    
    # 创建任务管理器节点
    task_manager_node = Node(
        package='seeway_task_manager',
        executable='task_manager_node.py',
        name='task_manager',
        output='screen'
    )
    
    return LaunchDescription([
        task_manager_node,
        task_scheduler_node,
    ])
