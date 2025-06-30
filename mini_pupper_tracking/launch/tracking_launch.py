from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_pupper_tracking',
            executable='main',
            name='mini_pupper_tracking_node',
            output='screen'
        ),
        Node(
            package='mini_pupper_tracking',
            executable='movement_node',
            name='mini_pupper_movement_node',
            output='screen'
        )
    ])
