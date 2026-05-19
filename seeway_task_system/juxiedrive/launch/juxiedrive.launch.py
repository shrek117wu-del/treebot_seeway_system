#!/usr/bin/env python3
"""Launch file for the JuxieDrive driver node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('juxiedrive')
    default_config = os.path.join(pkg_dir, 'config', 'juxiedrive_config.yaml')

    can_channel_arg = DeclareLaunchArgument('can_channel', default_value='can0')
    can_bitrate_arg = DeclareLaunchArgument('can_bitrate', default_value='1000000')
    node_id_arg = DeclareLaunchArgument('default_node_id', default_value='1')
    joint_name_arg = DeclareLaunchArgument('joint_name', default_value='juxie_joint')

    node = Node(
        package='juxiedrive',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[
            default_config,
            {
                'can_channel': LaunchConfiguration('can_channel'),
                'can_bitrate': LaunchConfiguration('can_bitrate'),
                'default_node_id': LaunchConfiguration('default_node_id'),
                'joint_name': LaunchConfiguration('joint_name'),
            },
        ],
    )

    return LaunchDescription([
        can_channel_arg,
        can_bitrate_arg,
        node_id_arg,
        joint_name_arg,
        node,
    ])
