import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # 1. IMU
    #
    # 等价于：
    # ros2 launch wit_ros2_imu rviz_and_imu.launch.py
    # ============================================================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('wit_ros2_imu'),
                'rviz_and_imu.launch.py'
            )
        )
    )

    # ============================================================
    # 2. 遥控器
    #
    # 等价于：
    # ros2 run bbot_rc_receiver rc_node
    # ============================================================
    rc_node = Node(
        package='bbot_rc_receiver',
        executable='rc_node',
        name='rc_node',
        output='screen',
    )

    # ============================================================
    # 3. LQR 平衡控制器
    #
    # 延迟启动，先等待 IMU 和遥控器节点初始化
    # ============================================================
    lqr_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        output='screen',
    )

    delayed_lqr_node = TimerAction(
        period=2.0,
        actions=[
            lqr_node
        ]
    )

    return LaunchDescription([
        imu_launch,
        rc_node,
        delayed_lqr_node,
    ])
