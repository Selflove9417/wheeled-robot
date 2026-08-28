from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pid_node = Node(
        package='bbot_real',
        executable='pid_balance_controller',
        name='pid_balance_controller',
        output='screen',
    )

    return LaunchDescription([
        pid_node,
    ])

