from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lqr_node = Node(
        package='bbot_real',
        executable='lqr_balance_controller',
        name='lqr_balance_controller',
        output='screen',
    )

    return LaunchDescription([
        lqr_node
    ])