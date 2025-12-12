import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from controller import Robot, Supervisor, Motor

# Importações de Cálculo
from ikpy.chain import Chain
import numpy as np

class UR5AutonomousGrasper(Node):

    def __init__(self):
        super().__init__('ur5_autonomous_grasper')
        
        # Inicializa o Supervisor (Necessário para o 'cheat' de pegar a posição do objeto)
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # 1. Setup da Câmera (Igual ao seu código de referência)
        self.configurar_camera()

        # 2. Setup dos Motores do Braço
        self.joints = [] # Lista para os motores do braço (usado no IK)
        self.configurar_motores_braco()

        # 3. Setup da Garra (Física e Força)
        self.finger_1 = None
        self.finger_2 = None
        self.configurar_garra()

        # 4. Setup da Cinemática Inversa (IKPy)
        self.chain = None
        self.carregar_kinematics()

        # 5. Timer do ROS (Substitui o loop while)
        self.timer = self.create_timer(self.timestep / 1000.0, self.step_webots)
        
        # Variáveis de Estado da Automação
        self.state = "SEARCH" 
        self.target_name = "BOLA" # Nome DEF do objeto no Webots
        self.target_node = None

        self.get_logger().info("Controlador Autônomo UR5 (Timer-based) Iniciado.")

    def configurar_camera(self):
        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            self.get_logger().info("Câmera 'camera_sensor' ativada!")
        else:
            self.get_logger().warn("AVISO: Câmera não encontrada!")

        self.camera_publisher = self.create_publisher(
            Image, 
            '/UR5e/camera_sensor/image_color', 
            10
        )

    def configurar_motores_braco(self):
        # Nomes exatos das juntas para garantir a ordem correta para o IKPy
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        for name in joint_names:
            motor = self.robot.getDevice(name)
            if motor:
                motor.setVelocity(1.0)
                self.joints.append(motor)
            else:
                self.get_logger().error(f"Motor {name} não encontrado!")

    def configurar_garra(self):
        # Tenta pegar pelos nomes padrão
        self.finger_1 = self.robot.getDevice('GARRA')
        self.finger_2 = self.robot.getDevice('GARRA')

        # Se encontrou, configura força e velocidade (igual ao seu código)
        if self.finger_1 and self.finger_2:
            max_force = 50.0 # N
            velocity = 1.0
            
            self.finger_1.setVelocity(velocity)
            self.finger_1.setAvailableForce(max_force)
            
            self.finger_2.setVelocity(velocity)
            self.finger_2.setAvailableForce(max_force)
            self.get_logger().info("Garra configurada com força máxima de 50N.")
        else:
            self.get_logger().warn("Motores da garra não encontrados!")

    def carregar_kinematics(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Caminho relativo inteligente: sobe um nível e entra em resource
        urdf_path = os.path.join(script_dir, '..', 'resource', 'robot_driver.urdf')
        
        self.get_logger().info(f"Carregando URDF de: {urdf_path}")

        try:
            self.chain = Chain.from_urdf_file(
                urdf_path,
                active_links_mask=[False, True, True, True, True, True, True]
            )
            self.get_logger().info("IKPy: Cadeia Cinemática carregada.")
        except Exception as e:
            self.get_logger().error(f"Erro CRÍTICO ao carregar URDF: {e}")
            # Fallback hardcoded se necessário
            try:
                path_fallback = "/trainee/workspace/trainee_ws/src/robot_vision/resource/robot_driver.urdf"
                self.chain = Chain.from_urdf_file(path_fallback, active_links_mask=[False, True, True, True, True, True, True])
            except:
                pass

    # --- Lógica de Controle ---

    def mover_garra(self, fechar=True):
        if not self.finger_1 or not self.finger_2:
            return
        
        pos_alvo = 0.85 if fechar else 0.0
        self.finger_1.setPosition(pos_alvo)
        self.finger_2.setPosition(pos_alvo)

    def mover_para_xyz(self, target_xyz):
        if not self.chain: return

        # 1. Pega posição atual (Seed) para ajudar o cálculo
        current_joints = [0] # Base fixa
        for motor in self.joints:
            current_joints.append(motor.getTargetPosition())
        
        # 2. Define a Matriz de Rotação "Olhando para Baixo"
        # Isso força os punhos (Wrists) a girarem para alinhar a garra verticalmente.
        # X alinhado, Y invertido, Z apontando para baixo (-1) no mundo.
        target_orientation_matrix = np.array([
            [1, 0,  0],
            [0, -1, 0],
            [0, 0, -1]
        ])

        # 3. Calcula IK usando mode="all" (Posição + Rotação Estrita)
        try:
            ik_solution = self.chain.inverse_kinematics(
                target_position=target_xyz,
                target_orientation=target_orientation_matrix, 
                orientation_mode="all", # <--- MUDANÇA IMPORTANTE AQUI
                initial_position=current_joints
            )
            
            # 4. Aplica aos motores
            joint_angles = ik_solution[1:7]
            
            if len(self.joints) == len(joint_angles):
                for i, motor in enumerate(self.joints):
                    motor.setPosition(joint_angles[i])
            
            # Debug: Mostra os ângulos calculados para ver se os punhos estão reagindo
            # self.get_logger().info(f"IK Solução (Punhos): {joint_angles[3]:.2f}, {joint_angles[4]:.2f}, {joint_angles[5]:.2f}")

        except Exception as e:
            self.get_logger().error(f"Erro no cálculo do IK: {e}")

    def ir_para_pose_inicial(self):
        # Posições em Radianos: [Base, Ombro, Cotovelo, W1, W2, W3]
        # Wrist_2 em -1.57 faz a garra apontar para baixo
        pose_olhando_baixo = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        self.get_logger().info("Movendo para pose inicial (Câmera para baixo)...")
        
        # Move cada motor
        for i, motor in enumerate(self.joints):
            # Verificação de segurança para não exceder índice
            if i < len(pose_olhando_baixo):
                motor.setPosition(pose_olhando_baixo[i])
        
        # Damos um tempo para ele chegar lá fisicamente na simulação
        # (Idealmente seria checar o sensor, mas um loop simples resolve agora)
        for _ in range(50): 
            self.robot.step(self.timestep)

    def get_cheat_target_position(self):
        if not self.target_node:
            self.target_node = self.robot.getFromDef(self.target_name)
        
        if self.target_node:
            return self.target_node.getPosition()
        return None

    # --- Loop Principal (Timer Callback) ---

    def step_webots(self):
        # 1. Avança a simulação
        if self.robot.step(self.timestep) == -1:
            rclpy.shutdown()
            return

        # 2. Publica Imagem
        self.publish_camera_image()

        # 3. Executa Máquina de Estados (Lógica Autônoma)
        self.executar_maquina_de_estados()

    def executar_maquina_de_estados(self):
        target_pos = self.get_cheat_target_position()
        
        if not target_pos:
            if self.state == "SEARCH":
                # Apenas um log throttled para não spammar
                self.get_logger().warn(f"Procurando objeto '{self.target_name}'...", throttle_duration_sec=5)
            return

        # Lógica da State Machine
        if self.state == "SEARCH":
            self.get_logger().info(f"Objeto detectado em: {target_pos}")
            self.mover_garra(fechar=False)
            self.state = "APPROACH"
        
        elif self.state == "APPROACH":
            # 20cm acima do objeto
            approach_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.20]
            self.mover_para_xyz(approach_pos)
            self.state = "LOWER"

        elif self.state == "LOWER":
            # Desce para pegar (compensando altura da garra)
            grasp_height = target_pos[2] + 0.14
            self.mover_para_xyz([target_pos[0], target_pos[1], grasp_height])
            
            # Checagem simples de tempo/posição poderia ser adicionada aqui
            # Por enquanto, assumimos que o movimento é rápido o suficiente
            # Usamos um contador simples ou lógica de delay se necessário.
            # Como estamos num timer, o ideal seria checar se o erro do motor é pequeno.
            # Para simplificar, passamos para o próximo estado:
            self.state = "WAIT_FOR_GRASP_POS"
            self.wait_counter = 0

        elif self.state == "WAIT_FOR_GRASP_POS":
            # Estado intermediário para esperar o braço chegar fisicamente
            self.wait_counter += 1
            if self.wait_counter > 20: # Espera ~600ms (dependendo do timestep)
                self.state = "GRASP"

        elif self.state == "GRASP":
            self.mover_garra(fechar=True)
            self.wait_counter = 0
            self.state = "WAIT_FOR_CLOSE"

        elif self.state == "WAIT_FOR_CLOSE":
            self.wait_counter += 1
            if self.wait_counter > 50: # Espera garra fechar
                self.state = "LIFT"

        elif self.state == "LIFT":
            lift_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.30]
            self.mover_para_xyz(lift_pos)
            self.state = "FINISHED"
        
        elif self.state == "FINISHED":
            pass

    def publish_camera_image(self):
        if self.camera:
            raw_image = self.camera.getImage()
            if raw_image:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link_optical'
                msg.height = self.camera.getHeight()
                msg.width = self.camera.getWidth()
                msg.encoding = 'bgra8'
                msg.step = 4 * self.camera.getWidth()
                msg.data = raw_image
                self.camera_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR5AutonomousGrasper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()