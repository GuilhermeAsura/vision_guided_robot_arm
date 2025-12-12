from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # vision node
        Node(
            package='robot_extra',
            executable='depth_estimator_node',
            name='depth_estimator_node',
            output='screen'
        ),

        # robot controler -> Webots + Câmera
        Node(
            package='robot_extra',
            executable='robot_controller_node',
            name='robot_controller_node',
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