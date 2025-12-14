import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Caminho para o URDF (AJUSTE SE NECESSÁRIO O NOME DO PACOTE DO URDF)
    # Estou assumindo que o urdf está em robot_vision/resource/robot_driver.urdf baseado nos seus logs
    # O ideal é carregar o URDF como string para passar pro robot_state_publisher
    urdf_file = '/trainee/workspace/trainee_ws/src/robot_vision/robot_vision/resource/robot_driver.urdf'
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        # 1. ROBOT STATE PUBLISHER (Cria a árvore do robô: base -> wrist)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_file]
        ),

        # 2. STATIC TF (Cola a câmera na garra e corrige a rotação)
        # Conecta 'wrist_3_link' (ou tool0) ao 'camera_link_optical'
        # Args: x y z qx qy qz qw (ou x y z yaw pitch roll)
        # Translation: 0 0 -0.05 (5cm acima da garra, eixo negativo conforme sua descrição de setup)
        # Rotation: Precisamos alinhar Z-optical com X-robot.
        # pitch=-1.57 (Gira Z pra frente), roll=-1.57 (Gira X pra direita)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '-0.05', '0.5', '-0.5', '0.5', '0.5', 'wrist_3_link', 'camera_link_optical']
        ),
        # 3. Vision Node
        Node(
            package='robot_extra',
            executable='depth_estimator_node',
            name='depth_estimator_node',
            output='screen'
        ),

        # 4. IK Controller
        Node(
            package='robot_extra',
            executable='ik_controller_node',
            name='ik_controller_node',
            output='screen'
        ),

        # 5. Keyboard
        Node(
            package='keyboard_check',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen',
            prefix='xterm -e' 
        )
    ])