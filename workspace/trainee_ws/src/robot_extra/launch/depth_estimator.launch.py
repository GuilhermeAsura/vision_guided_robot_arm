from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_extra',
            executable='depth_estimator_node',
            name='depth_estimator_node',
            output='screen'
        ), 
    ])