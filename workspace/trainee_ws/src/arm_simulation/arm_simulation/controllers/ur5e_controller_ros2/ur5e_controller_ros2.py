from controller import Robot, Keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import math
import sys
import os


class UR5eHybridController:
    def __init__(self):
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
            joint = self.robot.getDevice(name)
            self.joints.append(joint)
            print(f"Junta encontrada: {name}")
        
        self.current_joint = 0
        self.joint_step = 0.1
        self.home_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.ready_position = [0.0, -1.0, 1.0, -1.0, -1.57, 0.0]
        
        try:
            rclpy.init()
            self.node = Node('ur5e_webots_controller')
            
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
            
            print("=== UR5e Hybrid Controller (Webots + ROS2) Iniciado ===")
            print("Controles Webots:")
            print("  Setas ↑/↓: Mover junta atual")
            print("  Teclas 1-6: Selecionar junta")
            print("Controles ROS2:")
            print("  Use: ros2 topic pub /ur5e/target_positions std_msgs/msg/Float64MultiArray")
            
        except Exception as e:
            print(f"Erro ao inicializar ROS2: {e}")
            print("Continuando apenas com controle Webots...")
            self.node = None
        
        self.move_to_position(self.home_position)
        print("Movido para posição HOME")
    
    def move_to_position(self, position):
        """Move para posição específica - igual ao seu código original"""
        for i, joint in enumerate(self.joints):
            joint.setPosition(position[i])
        print(f"Movendo para: {position}")
    
    def show_positions(self):
        """Mostra posições atuais"""
        positions = [joint.getTargetPosition() for joint in self.joints]
        print("Posições atuais:")
        for i, pos in enumerate(positions):
            print(f"  Junta {i+1}: {pos:.3f} rad")
    
    def ros_position_callback(self, msg):
        """Comandos de posição vindos do ROS2"""
        if len(msg.data) == 6:
            self.move_to_position(list(msg.data))
            print(f"Comando ROS2 recebido: {list(msg.data)}")
    
    def ros_keyboard_callback(self, msg):
        """Comandos de teclado vindos do ROS2"""
        command = msg.data
        print(f"Comando teclado ROS2: {command}")
        
        if command == 'home':
            self.move_to_position(self.home_position)
        elif command == 'ready':
            self.move_to_position(self.ready_position)
        elif command == 'show_positions':
            self.show_positions()
        elif command.startswith('select_joint_'):
            joint_num = int(command.split('_')[-1])
            if 1 <= joint_num <= 6:
                self.current_joint = joint_num - 1
                print(f"ROS2: Controlando junta {self.current_joint + 1}")
        elif command == 'up':
            current_pos = self.joints[self.current_joint].getTargetPosition()
            self.joints[self.current_joint].setPosition(current_pos + self.joint_step)
            print(f"ROS2: Junta {self.current_joint + 1}: {current_pos + self.joint_step:.3f}")
        elif command == 'down':
            current_pos = self.joints[self.current_joint].getTargetPosition()
            self.joints[self.current_joint].setPosition(current_pos - self.joint_step)
            print(f"ROS2: Junta {self.current_joint + 1}: {current_pos - self.joint_step:.3f}")
    
    def publish_joint_states(self):
        """Publica estados atuais das juntas para ROS2"""
        if self.node is not None:
            positions = [joint.getTargetPosition() for joint in self.joints]
            msg = Float64MultiArray()
            msg.data = positions
            self.joint_state_publisher.publish(msg)
    
    def run(self):
        """Loop principal - combina Webots e ROS2"""
        while self.robot.step(self.timestep) != -1:
            if self.node is not None:
                rclpy.spin_once(self.node, timeout_sec=0)
                self.publish_joint_states()
            
            key = self.keyboard.getKey()
            
            if key == Keyboard.UP:
                current_pos = self.joints[self.current_joint].getTargetPosition()
                self.joints[self.current_joint].setPosition(current_pos + self.joint_step)
                print(f"Webots: Junta {self.current_joint + 1}: {current_pos + self.joint_step:.3f}")
                
            elif key == Keyboard.DOWN:
                current_pos = self.joints[self.current_joint].getTargetPosition()
                self.joints[self.current_joint].setPosition(current_pos - self.joint_step)
                print(f"Webots: Junta {self.current_joint + 1}: {current_pos - self.joint_step:.3f}")
                
            elif key >= ord('1') and key <= ord('6'):
                self.current_joint = key - ord('1')
                print(f"Webots: Controlando junta {self.current_joint + 1}")
        
        if self.node is not None:
            self.node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    controller = UR5eHybridController()
    controller.run()