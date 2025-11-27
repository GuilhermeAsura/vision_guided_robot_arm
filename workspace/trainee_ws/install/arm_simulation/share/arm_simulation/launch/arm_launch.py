import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('arm_simulation')
    
    # path to the world file
    world = LaunchConfiguration('world')

    # Webots initialization
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'basic_arm.wbt'),
        ros2_supervisor=True 
    )

    # robot driver node
    # creates the bridge between Webots motors/sensor and ROS2 topics
    my_robot_driver = WebotsController(
        robot_name='Ur5e', 
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'robot_driver.urdf')}
        ]
    )
    # controller node
    # TODO: debug why this node isnt moving the arm
    controller_node = Node(
        package='arm_simulation',
        executable='controller',
        name='arm_sine_wave_controller',
        output='screen' # log to screen
    )
    return LaunchDescription([
        webots,
        my_robot_driver,
        controller_node,
        webots._supervisor,
    ])