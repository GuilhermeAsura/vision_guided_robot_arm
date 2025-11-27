# Importações necessárias
from controller import Robot, Keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import sys

class UR5eHybridController:
    def __init__(self, ros_node=None):
        # Inicializa o hardware do Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.joints = []
        for name in self.joint_names:
            # Tenta pegar o dispositivo. Se o nome estiver errado no Webots, isso avisa.
            joint = self.robot.getDevice(name)
            if joint:
                self.joints.append(joint)
                print(f"Junta encontrada: {name}")
            else:
                print(f"AVISO: Junta {name} não encontrada no robô!")
        
        self.current_joint = 0
        self.joint_step = 0.1
        self.home_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.ready_position = [0.0, -1.0, 1.0, -1.0, -1.57, 0.0]
        
        # Configuração do ROS2
        self.node = ros_node
        if self.node:
            # Publishers e Subscribers
            self.ros_position_subscriber = self.node.create_subscription(
                Float64MultiArray,
                '/ur5e/target_positions',
                self.ros_position_callback,
                10
            )
            
            self.ros_keyboard_subscriber = self.node.create_subscription(
                String,
                '/ur5e/keyboard_command',
                self.ros_keyboard_callback,
                10
            )
            
            self.joint_state_publisher = self.node.create_publisher(
                Float64MultiArray,
                '/ur5e/webots_joint_states',
                10
            )
            
            self.node.get_logger().info("=== UR5e Hybrid Controller (Webots + ROS2) Iniciado ===")
            print("Controles Webots: Setas e Teclas 1-6")

        # Move para posição inicial
        self.move_to_position(self.home_position)
    
    def move_to_position(self, position):
        if len(position) != len(self.joints):
            print("Erro: Tamanho do vetor de posição incompatível")
            return
        for i, joint in enumerate(self.joints):
            joint.setPosition(position[i])
        # print(f"Movendo para: {position}")
    
    def show_positions(self):
        positions = [joint.getTargetPosition() for joint in self.joints]
        print("Posições atuais:")
        for i, pos in enumerate(positions):
            print(f"  Junta {i+1}: {pos:.3f} rad")
    
    def ros_position_callback(self, msg):
        if len(msg.data) == 6:
            self.move_to_position(list(msg.data))
            if self.node:
                self.node.get_logger().info(f"Comando ROS2 recebido: {list(msg.data)}")
    
    def ros_keyboard_callback(self, msg):
        command = msg.data
        if self.node:
            self.node.get_logger().info(f"Comando teclado ROS2: {command}")
        
        if command == 'home':
            self.move_to_position(self.home_position)
        elif command == 'ready':
            self.move_to_position(self.ready_position)
        elif command == 'show_positions':
            self.show_positions()
        elif command.startswith('select_joint_'):
            try:
                joint_num = int(command.split('_')[-1])
                if 1 <= joint_num <= 6:
                    self.current_joint = joint_num - 1
            except ValueError:
                pass
        elif command == 'up':
            self.move_single_joint(self.joint_step)
        elif command == 'down':
            self.move_single_joint(-self.joint_step)

    def move_single_joint(self, delta):
        if 0 <= self.current_joint < len(self.joints):
            current_pos = self.joints[self.current_joint].getTargetPosition()
            self.joints[self.current_joint].setPosition(current_pos + delta)
            print(f"Junta {self.current_joint + 1}: {current_pos + delta:.3f}")

    def publish_joint_states(self):
        if self.node:
            positions = [joint.getTargetPosition() for joint in self.joints]
            msg = Float64MultiArray()
            msg.data = positions
            self.joint_state_publisher.publish(msg)
    
    def run(self):
        # Loop principal do Webots
        while self.robot.step(self.timestep) != -1:
            # 1. Processa eventos ROS2 (sem bloquear)
            if self.node:
                rclpy.spin_once(self.node, timeout_sec=0)
                self.publish_joint_states()
            
            # 2. Processa teclado do Webots
            key = self.keyboard.getKey()
            if key == Keyboard.UP:
                self.move_single_joint(self.joint_step)
            elif key == Keyboard.DOWN:
                self.move_single_joint(-self.joint_step)
            elif key >= ord('1') and key <= ord('6'):
                self.current_joint = key - ord('1')
                print(f"Webots: Controlando junta {self.current_joint + 1}")

def main(args=None):
    # Inicializa o contexto ROS2
    rclpy.init(args=args)
    
    # Cria o nó ROS2
    node = Node('ur5e_webots_controller')
    
    try:
        # Instancia o controlador passando o nó
        controller = UR5eHybridController(ros_node=node)
        
        # Roda o loop principal
        controller.run()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Erro no controlador: {e}")
    finally:
        # Limpeza
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()