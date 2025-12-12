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
        # alterei para testes
        Node(
            package='robot_vision',
            executable='visual_controller',
            name='visual_controller',
            output='screen',
            prefix='xterm -e'
        )
    ])