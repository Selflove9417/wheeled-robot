#!/usr/bin/env python3
"""
BBot Real Robot — Full Bringup
===============================
Launches:
  1. WitMotion IMU driver
  2. CRSF RC receiver
  3. Balance controller (PID or LQR, select via 'controller' arg)

Usage:
  ros2 launch bbot_real bringup.launch.py controller:=lqr
  ros2 launch bbot_real bringup.launch.py controller:=pid can_interface:=can1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Arguments ----
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='pid',
        description='Which controller to run: pid or lqr'
    )
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface name'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/data',
        description='IMU data topic'
    )
    use_rc_arg = DeclareLaunchArgument(
        'use_rc', default_value='true',
        description='Enable CRSF RC receiver node'
    )
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Enable WitMotion IMU driver node'
    )

    controller = LaunchConfiguration('controller')
    can_interface = LaunchConfiguration('can_interface')
    imu_topic = LaunchConfiguration('imu_topic')
    use_rc = LaunchConfiguration('use_rc')
    use_imu = LaunchConfiguration('use_imu')

    # ---- Conditions ----
    # Use PythonExpression to compare controller value
    is_pid = IfCondition(PythonExpression(["'", controller, "' == 'pid'"]))
    is_lqr = IfCondition(PythonExpression(["'", controller, "' == 'lqr'"]))

    # ---- Nodes ----

    # WitMotion IMU driver
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        condition=IfCondition(use_imu),
        output='screen',
        remappings=[('/imu/data_raw', '/imu/data')],
    )

    # CRSF RC receiver
    rc_node = Node(
        package='bbot_rc_receiver',
        executable='rc_node',
        name='rc_receiver',
        condition=IfCondition(use_rc),
        output='screen',
    )

    # PID Balance Controller
    pid_controller_node = Node(
        package='bbot_real',
        executable='pid_balance_controller',
        name='pid_balance_controller',
        parameters=[{
            'can_interface': can_interface,
            'imu_topic': imu_topic,
            'max_wheel_speed': 2.5,
        }],
        output='screen',
        condition=is_pid,
    )

    # LQR Balance Controller
    lqr_controller_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        parameters=[{
            'can_interface': can_interface,
            'imu_topic': imu_topic,
        }],
        output='screen',
        condition=is_lqr,
    )

    return LaunchDescription([
        controller_arg,
        can_interface_arg,
        imu_topic_arg,
        use_rc_arg,
        use_imu_arg,
        imu_node,
        rc_node,
        pid_controller_node,
        lqr_controller_node,
    ])
