#!/usr/bin/env python3
"""
LQR Balance Controller — Standalone Launch
==========================================
Launches only the LQR balance controller (no IMU or RC node).

Usage:
  ros2 launch bbot_real lqr_balance.launch.py can_interface:=can0
  ros2 launch bbot_real lqr_balance.launch.py fix_legs:=true hip_angle_deg:=20.0
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
    fix_legs_arg = DeclareLaunchArgument(
        'fix_legs', default_value='false',
        description='Lock legs at fixed angles instead of using IK'
    )
    hip_angle_arg = DeclareLaunchArgument(
        'hip_angle_deg', default_value='20.0',
        description='Fixed hip joint angle in degrees (fix_legs mode)'
    )
    knee_angle_arg = DeclareLaunchArgument(
        'knee_angle_deg', default_value='-20.0',
        description='Fixed knee joint angle in degrees (fix_legs mode)'
    )

    lqr_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'fix_legs': LaunchConfiguration('fix_legs'),
            'hip_angle_deg': LaunchConfiguration('hip_angle_deg'),
            'knee_angle_deg': LaunchConfiguration('knee_angle_deg'),
        }],
        output='screen',
    )

    return LaunchDescription([
        can_interface_arg,
        imu_topic_arg,
        fix_legs_arg,
        hip_angle_arg,
        knee_angle_arg,
        lqr_node,
    ])
