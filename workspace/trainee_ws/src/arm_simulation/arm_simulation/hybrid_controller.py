#!/usr/bin/env python3
from controller import Robot, Keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image



class UR5eHybridController:
    def __init__(self, ros_node=None):
        # Inicialização do Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Inicialização do Teclado
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # 1. Configuração da Câmera
        # O nome 'camera_sensor' deve ser idêntico ao campo 'name' no Webots
        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            print("Câmera inicializada com sucesso.")
        else:
            print("ERRO CRÍTICO: Dispositivo 'camera_sensor' não encontrado no robô.")

        # Configuração das Juntas
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.joints = []
        for name in self.joint_names:
            joint = self.robot.getDevice(name)
            if joint:
                self.joints.append(joint)
            else:
                print(f"AVISO: Junta {name} não encontrada!")

        # Variáveis de Estado
        self.current_joint = 0
        self.joint_step = 0.05
        self.home_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.tracking_active = False

        # Configuração do ROS2
        self.node = ros_node
        if self.node:
            # Publishers
            self.joint_state_publisher = self.node.create_publisher(
                Float64MultiArray, '/ur5e/webots_joint_states', 10)

            # Publisher da Câmera
            self.camera_publisher = self.node.create_publisher(
                Image,
                '/UR5e/camera_sensor/image_color',
                10)

            # Subscribers
            self.ros_position_subscriber = self.node.create_subscription(
                Float64MultiArray, '/ur5e/target_positions', self.ros_position_callback, 10)

            self.ros_keyboard_subscriber = self.node.create_subscription(
                String, '/ur5e/keyboard_command', self.ros_keyboard_callback, 10)

            # Subscriber de Visão
            self.vision_subscriber = self.node.create_subscription(
                Point, '/vision/object_coordinates', self.vision_callback, 10)

            self.node.get_logger().info("=== UR5e Driver & Controller Iniciado ===")
            print("Câmera ativa. Pressione 'T' para ativar Visual Servoing.")

        self.move_to_position(self.home_position)

    # Lógica de Leitura e Publicação da Câmera
    def publish_camera_image(self):
        if self.node and self.camera:
            # Pega a imagem crua (bytes) do Webots
            raw_image = self.camera.getImage()

            if raw_image:
                msg = Image()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link_optical'
                msg.height = self.camera.getHeight()
                msg.width = self.camera.getWidth()
                msg.encoding = 'bgra8'  # Webots nativamente retorna BGRA
                msg.is_bigendian = False
                msg.step = 4 * self.camera.getWidth()  # 4 bytes por pixel (BGRA)
                msg.data = raw_image

                self.camera_publisher.publish(msg)

    # Lógica de Controle do Robô
    def move_to_position(self, position):
        if len(position) != len(self.joints): return
        for i, joint in enumerate(self.joints):
            joint.setPosition(position[i])

    def move_single_joint(self, delta, joint_idx=None):
        idx = joint_idx if joint_idx is not None else self.current_joint
        if 0 <= idx < len(self.joints):
            current_pos = self.joints[idx].getTargetPosition()
            self.joints[idx].setPosition(current_pos + delta)

    # Callbacks ROS2
    def vision_callback(self, msg):
        #Lógica de Visual Servoing (Perseguição do Objeto)
        if not self.tracking_active: return

        # Centro da imagem (assumindo 640x480, se mudar no Webots, mude aqui)
        img_center_x = self.camera.getWidth() / 2
        img_center_y = self.camera.getHeight() / 2
        dead_zone = 20

        error_x = img_center_x - msg.x
        error_y = img_center_y - msg.y

        # Ganhos Proporcionais (Ajuste fino pode ser necessário)
        kp_pan = 0.001
        kp_lift = 0.001

        if abs(error_x) > dead_zone:
            # Move a base (Junta 0)
            delta_pan = kp_pan * error_x
            self.move_single_joint(delta_pan, joint_idx=0)

        if abs(error_y) > dead_zone:
            # Move o ombro (Junta 1) - Invertido pois Y na imagem cresce para baixo
            delta_lift = -kp_lift * error_y
            self.move_single_joint(delta_lift, joint_idx=1)

    def ros_position_callback(self, msg):
        self.tracking_active = False
        if len(msg.data) == 6:
            self.move_to_position(list(msg.data))

    def ros_keyboard_callback(self, msg):
        if msg.data == 'track_toggle':
            self.tracking_active = not self.tracking_active
            self.node.get_logger().info(f"Tracking: {self.tracking_active}")
        # Outros comandos podem ser adicionados aqui

    # Loop Principal
    def run(self):
        while self.robot.step(self.timestep) != -1:
            # 1. Processa ROS2
            if self.node:
                # Publica a imagem da câmera para o vision.py
                self.publish_camera_image()

                # Publica estado das juntas
                self.publish_joint_states()

                # Processa callbacks recebidos
                rclpy.spin_once(self.node, timeout_sec=0)

            # 2. Processa Teclado Webots
            key = self.keyboard.getKey()
            if key == ord('T'):
                self.tracking_active = not self.tracking_active
                print(f"Tracking {'ATIVADO' if self.tracking_active else 'DESATIVADO'}")
            elif key == Keyboard.UP:
                self.move_single_joint(self.joint_step)
            elif key == Keyboard.DOWN:
                self.move_single_joint(-self.joint_step)
            elif ord('1') <= key <= ord('6'):
                self.current_joint = key - ord('1')

    def publish_joint_states(self):
        if self.node:
            positions = [joint.getTargetPosition() for joint in self.joints]
            msg = Float64MultiArray()
            msg.data = positions
            self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Node('ur5e_driver_controller')
    try:
        controller = UR5eHybridController(ros_node=node)
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()