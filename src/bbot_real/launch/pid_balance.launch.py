#!/usr/bin/env python3
"""
PID Balance Controller — Standalone Launch
==========================================
Launches only the PID balance controller (no IMU or RC node).

Usage:
  ros2 launch bbot_real pid_balance.launch.py can_interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface name'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/data',
        description='IMU data topic'
    )

    pid_node = Node(
        package='bbot_real',
        executable='pid_balance_controller',
        name='pid_balance_controller',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'wheel_radius': 0.07,
            'wheel_base': 0.4,
            'max_wheel_speed': 2.5,
        }],
        output='screen',
    )

    return LaunchDescription([
        can_interface_arg,
        imu_topic_arg,
        pid_node,
    ])
