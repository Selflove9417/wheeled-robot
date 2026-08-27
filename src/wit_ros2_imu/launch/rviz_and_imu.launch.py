from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # IMU 驱动节点配置
    rviz_and_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',

        remappings=[('/imu/data_raw', '/imu/data')],
        parameters=[{'port': '/dev/ttyCH341USB1'},
                    {"baud": 230400}],
        output="screen"
    )

    # RViz2 可视化节点配置
    rviz_display_node = Node(
        package='rviz2',
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            rviz_and_imu_node,
            # rviz_display_node  # 如需伴随启动 RViz 可视化，取消此行注释即可
        ]
    )