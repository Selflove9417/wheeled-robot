#!/usr/bin/env python3
"""
BBot Real Robot — Full Bringup
===============================
Launches:
  1. WitMotion IMU driver (wit_ros2_imu)
  2. CRSF RC receiver (bbot_rc_receiver) [optional via use_rc]
  3. Balance controller (LQR or PID via controller arg)

Usage:
  # 启动 LQR 控制器 + IMU + 遥控器 (默认)
  ros2 launch bbot_real bringup.launch.py controller:=lqr use_rc:=true

  # 仅启动 LQR 控制器 + IMU (无需遥控器调试)
  ros2 launch bbot_real bringup.launch.py controller:=lqr use_rc:=false

  # 启动 PID 控制器 + IMU + 遥控器
  ros2 launch bbot_real bringup.launch.py controller:=pid use_rc:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Arguments ----
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='lqr',
        description='Which controller to run: lqr (default) or pid'
    )
    use_rc_arg = DeclareLaunchArgument(
        'use_rc', default_value='true',
        description='Enable CRSF RC receiver node (true/false)'
    )
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Enable WitMotion IMU driver node (true/false)'
    )

    controller = LaunchConfiguration('controller')
    use_rc = LaunchConfiguration('use_rc')
    use_imu = LaunchConfiguration('use_imu')

    # ---- Conditions ----
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
        output='screen',
        condition=is_pid,
    )

    # LQR Balance Controller
    lqr_controller_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        output='screen',
        condition=is_lqr,
    )

    return LaunchDescription([
        controller_arg,
        use_rc_arg,
        use_imu_arg,
        imu_node,
        rc_node,
        pid_controller_node,
        lqr_controller_node,
    ])
