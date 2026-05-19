#!/usr/bin/env python3
"""Launch JuxieDrive CAN node with config and joint list."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('juxiedrive')
    default_config = os.path.join(pkg_dir, 'config', 'juxiedrive_can.yaml')
    default_joints = os.path.join(pkg_dir, 'config', 'joints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config, description='Driver parameter YAML'),
        DeclareLaunchArgument('joints_config', default_value=default_joints, description='Joint inventory YAML'),
        DeclareLaunchArgument('can_channel', default_value='can0', description='SocketCAN channel'),
        DeclareLaunchArgument('can_bitrate', default_value='1000000', description='CAN arbitration bitrate'),
        DeclareLaunchArgument('can_data_bitrate', default_value='5000000', description='CAN-FD data bitrate'),
        Node(
            package='juxiedrive',
            executable='juxiedrive_node',
            name='juxiedrive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'joints_config': LaunchConfiguration('joints_config'),
                    'can_channel': LaunchConfiguration('can_channel'),
                    'can_bitrate': LaunchConfiguration('can_bitrate'),
                    'can_data_bitrate': LaunchConfiguration('can_data_bitrate'),
                },
            ],
        ),
    ])
