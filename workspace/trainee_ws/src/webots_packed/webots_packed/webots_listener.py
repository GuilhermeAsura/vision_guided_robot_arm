from controller import Robot, Motor  # Webots API
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

class WebotsSubscriber(Node):

    def __init__(self):
        super().__init__('webots_subscriber')  # nome do nó
        self.robot = Robot()  # inicializa webots

        self.timer = self.create_timer(0.032, self.step_webots) #inicializa o timer para rodar a simulação

        # recebendo os valores numericos do teclado
        self.subscription = self.create_subscription(
            Int32,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.last_cmd = -1  # inicializa o último comando recebido  
    
        # publica as infos das juntas
        self.publisher = self.create_publisher(
                Float64MultiArray,
                'ur5e/joint_targets',
                10
        )

        #  informações de juntas
        self.joint_default_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.0, 0.0]  # posiçõe iniciais das juntas: UR5e + end effector

        self.joints = [] # NOME DAS JUNTAS
        self.joint_names = [] # NOMES DAS JUNTAS
        self.joint_positions = [] # POSIÇÃO DAS JUNTAS

        self.joint_rotation_motor_min_max = [] # LIMITES DAS JUNTAS

        self.get_joints() # ESSA PORRA QUE SALVA AS JUNTAS, nome e posição

        # Junta selecionada
        self.selected = 0

        # Velocidade de movimento
        self.step = 0.1

        ## COLOCANDO AS JUNTAS NA POSIÇÃO INICIAL
        self.default_positions()

        self.get_logger().info("Ros2 iniciado e conectadi")
        self.get_logger().info("Use os valores numericos para selecionar a junta e as setas/WASD para move-la")

        #say list of joints
        for i, name in enumerate(self.joint_names):
            self.get_logger().info(f"Junta {i + 1}: {name}")

        for i, limits in enumerate(self.joint_rotation_motor_min_max):
            self.get_logger().info(f"Limites Junta {i + 1}: Min {limits[0]:.3f}, Max {limits[1]:.3f}")
        #for i, pos in enumerate(self.initial_positions):
        #    self.get_logger().info(f"Posição inicial Junta {i + 1}: {pos:.3f}")
        #for i in range(len(self.joints)):
         #   self.get_logger().info(f"Para selecionar a Junta {i + 1} pressione: {i + 1}")

    #função para rodar a simulação do webots
    def step_webots(self):
        self.robot.step(int(self.robot.getBasicTimeStep()))

    #isso existe apenas para ser execultado no init pq eu n sei como fazer sem deixar o codigo feio
    #default positions
    def default_positions(self):
        self.get_logger().info("RESETANDO PARA POSIÇÃO INICIAL")
        for i, joint in enumerate(self.joints):
            joint.setPosition(self.joint_default_positions[i])
            self.joint_positions[i] = self.joint_default_positions[i]
        self.publish_joints()

    #codigo refatorado do JAPA ;D
    def get_joints(self):
        n_devices = self.robot.getNumberOfDevices()

        for i in range(n_devices):
            dev = self.robot.getDeviceByIndex(i)
            print(dev.getName())

            # Adicionando o nome da junta a minha lista
            if isinstance(dev, Motor):
                self.joints.append(dev)
                self.joint_names.append(dev.getName())
                self.joint_positions.append(dev.getTargetPosition())

                #salvando os limites de cada junta
                self.joint_rotation_motor_min_max.append((dev.getMinPosition(), dev.getMaxPosition()))

    #função para publicar as posições das juntas
    def publish_joints(self):
        msg = Float64MultiArray() #msg criada
        msg.data = list(self.joint_positions) #atribuindo msg
        self.publisher.publish(msg) #publicando

    #processando os dados do teclado
    def keyboard_callback(self, msg):

        key = msg.data

        #RESETANDO PARA POSIÇÃO INICIAL
        if key == 201:
            self.default_positions()
            return

        #escolendo juntas
        if key <= 199 and key >= 100:
            val = key - 100 - 1 #o menos 1 é para a junta 1 ser o indice 0 na lista
            if 0 <= val < len(self.joints):
                self.selected = val
                self.get_logger().info(f"JUNTA {self.selected} EM OPERAÇÂO")
            else:
                self.get_logger().info(f"Junta {val} inválida. Escolha entre 1 e {len(self.joints)}.")
                return

        #Movendo a junta

        # W ou UP
        if key in (0, 4) and self.joint_positions[self.selected] + self.step <= self.joint_rotation_motor_min_max[self.selected][1]:
            self.joint_positions[self.selected] += self.step

        # S ou DOWN
        elif key in (1, 5) and self.joint_positions[self.selected] - self.step >= self.joint_rotation_motor_min_max[self.selected][0]:
            self.joint_positions[self.selected] -= self.step

        # teclas que não mexem juntas
        else:
            return

        #aplicando a nova posição com base na velocidade
        self.joints[self.selected].setPosition(self.joint_positions[self.selected])

        # publica estado atualizado
        self.publish_joints()

        self.get_logger().info(
            f"Junta {self.selected + 1}: {self.joint_positions[self.selected]:.3f}"
        )


#função main
def main(args=None):
    rclpy.init(args=args)
    node = WebotsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

##=== Main ===##
if __name__ == '__main__':
    main()