#!/usr/bin/env python3
"""
Launch file for the LZ_OMNI chassis driver node.

Usage:
  # UART mode (default)
  ros2 launch lz_omni_chassis_driver chassis_driver.launch.py

  # CAN mode
  ros2 launch lz_omni_chassis_driver chassis_driver.launch.py comm_type:=can can_channel:=can0

  # Custom serial port
  ros2 launch lz_omni_chassis_driver chassis_driver.launch.py uart_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lz_omni_chassis_driver')
    default_config = os.path.join(pkg_dir, 'config', 'chassis_driver_config.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    comm_type_arg = DeclareLaunchArgument(
        'comm_type',
        default_value='uart',
        description="Communication interface type: 'uart' or 'can'",
    )
    uart_port_arg = DeclareLaunchArgument(
        'uart_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path (UART mode only)',
    )
    uart_baudrate_arg = DeclareLaunchArgument(
        'uart_baudrate',
        default_value='115200',
        description='Serial baud rate (UART mode only)',
    )
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can0',
        description='SocketCAN interface name (CAN mode only)',
    )
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='500000',
        description='CAN bus bit rate in bps (CAN mode only)',
    )
    can_id_arg = DeclareLaunchArgument(
        'can_id',
        default_value='1',
        description='CAN arbitration ID for motion commands (CAN mode only)',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50.0',
        description='Control loop update rate in Hz',
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.5',
        description='Seconds before sending zero if no /cmd_vel received',
    )

    # ── Chassis driver node ───────────────────────────────────────────
    chassis_driver_node = Node(
        package='lz_omni_chassis_driver',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        parameters=[
            default_config,
            {
                'comm_type': LaunchConfiguration('comm_type'),
                'uart_port': LaunchConfiguration('uart_port'),
                'uart_baudrate': LaunchConfiguration('uart_baudrate'),
                'can_channel': LaunchConfiguration('can_channel'),
                'can_bitrate': LaunchConfiguration('can_bitrate'),
                'can_id': LaunchConfiguration('can_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            },
        ],
    )

    return LaunchDescription([
        comm_type_arg,
        uart_port_arg,
        uart_baudrate_arg,
        can_channel_arg,
        can_bitrate_arg,
        can_id_arg,
        publish_rate_arg,
        cmd_vel_timeout_arg,
        chassis_driver_node,
    ])
