import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # vision node
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        # controller node -> it's needs terminal interaction: 'T' to tracking
        Node(
            package='robot_vision',
            executable='visual_servoing_controller',
            name='visual_servoing_controller',
            output='screen',
            prefix='xterm -e'
        )
    ])