import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # controller node -> connects with Webots to receive commands
    webots_listener_node = Node(
        package='webots_packed',
        executable='webots_listener',
        name='webots_listener',
        output='screen'
    )

    # keyboard publisher node -> captures keyboard inputs and publishes them to a ROS2 topic
    # NOTE: use 'xterm' here; make sure it's installed or change to another terminal emulator if needed
    keyboard_publisher_node = Node(
        package='keyboard_check',
        executable='keyboard_publisher',
        name='keyboard_publisher',
        output='screen',
        prefix='xterm -e' 
    )

    return LaunchDescription([
        #  uncomment the line below to launch Webots automatically
        # ExecuteProcess(cmd=['webots'], output='screen'),
        
        webots_listener_node,
        keyboard_publisher_node
    ])