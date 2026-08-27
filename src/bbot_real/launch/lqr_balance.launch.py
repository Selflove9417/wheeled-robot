import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明参数并赋予默认值（防止缺失参数报错）
    declare_roll_kp = DeclareLaunchArgument(
        'roll_kp',
        default_value='0.35',
        description='Roll compensation P gain'
    )
    declare_roll_ki = DeclareLaunchArgument(
        'roll_ki',
        default_value='0.08',
        description='Roll compensation I gain'
    )
    declare_roll_kd = DeclareLaunchArgument(
        'roll_kd',
        default_value='0.015',
        description='Roll compensation D gain'
    )
    declare_roll_offset = DeclareLaunchArgument(
        'roll_offset',
        default_value='0.0',
        description='Roll mechanical offset angle (rad)'
    )
    declare_roll_sign = DeclareLaunchArgument(
        'roll_sign',
        default_value='-1.0',
        description='Roll compensation direction sign (+1.0 or -1.0)'
    )
    declare_max_delta_h = DeclareLaunchArgument(
        'max_delta_h',
        default_value='0.04',
        description='Max leg height delta for roll compensation (m)'
    )
    declare_target_height = DeclareLaunchArgument(
        'target_height',
        default_value='0.43',
        description='Target standing height (m)'
    )
    declare_leg_mode = DeclareLaunchArgument(
        'leg_control_mode',
        default_value='servo',
        description='Leg control mode (servo or hybrid)'
    )
    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface device name'
    )

    # 2. 控制器节点
    lqr_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        output='screen',
        parameters=[{
            'roll_kp': LaunchConfiguration('roll_kp'),
            'roll_ki': LaunchConfiguration('roll_ki'),
            'roll_kd': LaunchConfiguration('roll_kd'),
            'roll_offset': LaunchConfiguration('roll_offset'),
            'roll_sign': LaunchConfiguration('roll_sign'),
            'max_delta_h': LaunchConfiguration('max_delta_h'),
            'target_height': LaunchConfiguration('target_height'),
            'leg_control_mode': LaunchConfiguration('leg_control_mode'),
            'can_interface': LaunchConfiguration('can_interface'),
        }]
    )

    return LaunchDescription([
        declare_roll_kp,
        declare_roll_ki,
        declare_roll_kd,
        declare_roll_offset,
        declare_roll_sign,
        declare_max_delta_h,
        declare_target_height,
        declare_leg_mode,
        declare_can_interface,
        lqr_node
    ])