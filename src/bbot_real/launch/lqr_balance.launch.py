#!/usr/bin/env python3
"""
LQR Balance Controller — Standalone Launch
==========================================
Launches only the LQR balance controller (no IMU or RC node).

Usage:
  ros2 launch bbot_real lqr_balance.launch.py can_interface:=can0
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
    roll_kp_arg = DeclareLaunchArgument(
        'roll_kp', default_value='0.35',
        description='Roll compensation P gain'
    )
    roll_ki_arg = DeclareLaunchArgument(
        'roll_ki', default_value='0.08',
        description='Roll compensation I gain'
    )
    roll_kd_arg = DeclareLaunchArgument(
        'roll_kd', default_value='0.015',
        description='Roll compensation D gain'
    )
    roll_offset_arg = DeclareLaunchArgument(
        'roll_offset', default_value='0.0',
        description='Roll angle mechanical zero offset (rad)'
    )
    roll_sign_arg = DeclareLaunchArgument(
        'roll_sign', default_value='1.0',
        description='Roll compensation direction sign (+1.0 or -1.0)'
    )
    target_height_arg = DeclareLaunchArgument(
        'target_height', default_value='0.36',
        description='Nominal standing height (m)'
    )
    ki_pos_angle_arg = DeclareLaunchArgument(
        'ki_pos_angle', default_value='0.02',
        description='Position-to-pitch integral trim gain'
    )
    max_pos_trim_angle_arg = DeclareLaunchArgument(
        'max_pos_trim_angle', default_value='0.035',
        description='Max pitch trim angle limit (rad)'
    )

    lqr_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'roll_kp': LaunchConfiguration('roll_kp'),
            'roll_ki': LaunchConfiguration('roll_ki'),
            'roll_kd': LaunchConfiguration('roll_kd'),
            'roll_offset': LaunchConfiguration('roll_offset'),
            'roll_sign': LaunchConfiguration('roll_sign'),
            'target_height': LaunchConfiguration('target_height'),
            'ki_pos_angle': LaunchConfiguration('ki_pos_angle'),
            'max_pos_trim_angle': LaunchConfiguration('max_pos_trim_angle'),
        }],
        output='screen',
    )

    return LaunchDescription([
        can_interface_arg,
        imu_topic_arg,
        roll_kp_arg,
        roll_ki_arg,
        roll_kd_arg,
        roll_offset_arg,
        roll_sign_arg,
        target_height_arg,
        ki_pos_angle_arg,
        max_pos_trim_angle_arg,
        lqr_node,
    ])
