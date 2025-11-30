'''
this launch script runs the complete robot vision system: 
  - robot perception -> vision_node
  - keyboard controller
  - webots listener -> publish logs
'''

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

        # robot controler -> Webots + Câmera
        Node(
            package='webots_packed',
            executable='webots_listener',
            name='webots_listener',
            output='screen'
        ),

        # keyboard publisher ->  WASD/Numbers
        # NOTE: use 'xterm' here!! Make sure it's installed or change to another terminal emulator if needed
        Node(
            package='keyboard_check',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen',
            prefix='xterm -e'
        )
    ])