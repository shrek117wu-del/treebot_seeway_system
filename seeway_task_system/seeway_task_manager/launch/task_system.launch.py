#!/usr/bin/env python3
"""
Seeway Task System Launch File
启动完整的任务管理系统
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    declare_enable_task_manager = DeclareLaunchArgument(
        'enable_task_manager',
        default_value='true',
        description='启用任务管理器'
    )
    
    declare_enable_xbox_controller = DeclareLaunchArgument(
        'enable_xbox_controller',
        default_value='true',
        description='启用Xbox手柄控制'
    )
    
    declare_enable_nav2_client = DeclareLaunchArgument(
        'enable_nav2_client',
        default_value='true',
        description='启用Nav2客户端'
    )
    
    declare_enable_scheduler = DeclareLaunchArgument(
        'enable_scheduler',
        default_value='true',
        description='启用任务调度器'
    )
    
    # 获取配置文件路径
    config_dir = os.path.join(
        FindPackageShare('seeway_task_manager').find('seeway_task_manager'),
        'config'
    )
    
    xbox_config = os.path.join(config_dir, 'xbox_controller_config.yaml')
    nav_config = os.path.join(config_dir, 'navigation_config.yaml')
    
    # 创建节点
    # 1. 任务管理器节点
    task_manager_node = Node(
        package='seeway_task_manager',
        executable='task_manager_node.py',
        name='task_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_task_manager'))
    )
    
    # 2. Xbox 手柄控制节点
    xbox_controller_node = Node(
        package='seeway_task_manager',
        executable='xbox_controller_node.py',
        name='xbox_controller',
        output='screen',
        parameters=[xbox_config],
        condition=IfCondition(LaunchConfiguration('enable_xbox_controller'))
    )
    
    # 3. Nav2 客户端节点
    nav2_client_node = Node(
        package='seeway_task_manager',
        executable='nav2_client_node.py',
        name='nav2_client',
        output='screen',
        parameters=[nav_config],
        condition=IfCondition(LaunchConfiguration('enable_nav2_client'))
    )
    
    # 4. 任务调度器节点
    task_scheduler_node = Node(
        package='seeway_task_manager',
        executable='task_scheduler_node.py',
        name='task_scheduler',
        output='screen',
        parameters=[nav_config],
        condition=IfCondition(LaunchConfiguration('enable_scheduler'))
    )
    
    return LaunchDescription([
        declare_enable_task_manager,
        declare_enable_xbox_controller,
        declare_enable_nav2_client,
        declare_enable_scheduler,
        task_manager_node,
        xbox_controller_node,
        nav2_client_node,
        task_scheduler_node,
    ])
