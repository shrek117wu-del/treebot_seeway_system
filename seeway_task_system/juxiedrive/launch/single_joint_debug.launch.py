#!/usr/bin/env python3
"""Launch a single generic JuxieDrive joint for protocol debugging."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('juxiedrive')
    default_config = os.path.join(pkg_dir, 'config', 'juxiedrive_can.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('can_channel', default_value='can0'),
        Node(
            package='juxiedrive',
            executable='juxiedrive_node',
            name='juxiedrive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'can_channel': LaunchConfiguration('can_channel'),
                    'auto_sync_feedback': True,
                },
            ],
        ),
    ])
