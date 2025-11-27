import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('arm_simulation')
    package_prefix = get_package_prefix('arm_simulation')
    
    # Set WEBOTS_ROBOT_NAME as a launch-level environment variable
    set_robot_name = SetEnvironmentVariable(
        name='WEBOTS_ROBOT_NAME',
        value='UR5e'
    )
    
    # Webots initialization
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'basic_arm.wbt'),
        ros2_supervisor=True
    )

    # Correct path to the Python script
    controller_path = os.path.join(
        package_prefix,
        'lib',
        'python3.10',
        'site-packages',
        'arm_simulation',
        'hybrid_controller.py'
    )

    # Run Python script with environment variable set via bash
    custom_controller = ExecuteProcess(
        cmd=['bash', '-c', f'WEBOTS_ROBOT_NAME=UR5e python3 {controller_path}'],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        set_robot_name,
        webots,
        custom_controller,
        webots._supervisor,
    ])