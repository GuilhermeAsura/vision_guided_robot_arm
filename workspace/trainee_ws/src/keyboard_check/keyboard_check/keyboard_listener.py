#importando tudo
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#criando o meu NO
class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener') #definindo o nome do meu no

        #criando o meu.. recebimento? esse trem que recebe o retorno do keyboard_input
        self.subscription = self.create_subscription(
            Int32,
            'keyboard_input',
            self.callback,
            10
        )
        self.get_logger().info("Aguardando as teclas") #msg para falar que iniciou

    #função para falar para o usuario qual tecla foi - dps mudar para enviar o callback para o webrobot
    def callback(self, msg):
        self.get_logger().info(f"Tecla digitada: {msg.data}")

#main
def main():
    rclpy.init() #ros2 iniciando
    node = KeyboardListener() #criando node
    rclpy.spin(node) #rodando ele
    rclpy.shutdown() #fechando

if __name__ == "__main__":
    main()