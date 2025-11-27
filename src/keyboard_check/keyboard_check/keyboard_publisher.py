# IMportanto as bilbiotecas
import rclpy
from rclpy.node import Node #classe do nosso no
from std_msgs.msg import Int32 #definindo o tipo de msg que sera publicado

#bibliotecas para fazer o terminal ficar "limpo" 
import sys
import termios
import tty
import select

#definindo nosso no
class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher') #definindo o nome do no
        self.publisher = self.create_publisher(Int32, 'keyboard_input', 10) #criando minha publicação para enviar um msg tipo Int32, no No do keyboard e definindo 10 na fila
        self.get_logger().info("Iniciando o teste para o teclado") # mandando uma msg para informar que começou

    #função para ler as teclas
    def read_key(self): 
        
        #nota pessoal - por algum motivo toda vez que eu n reseto o terminal para o modo padrao ele BUGA TODO
        old_settings = termios.tcgetattr(sys.stdin) #salvando o estado do terminal
        try:
            tty.setcbreak(sys.stdin.fileno()) #ativando o modo "etcbreak" para digitar no tirmal sem precisar dar enter e nem aparecer alguma tecla
            if select.select([sys.stdin], [], [], 0.01)[0]: #caso alguma tecla seja indenficiada roda o que ta em baixo
                
                #criando sequencia ASCI para ler as seta (sim, eu n gostei de fazer isso)
                key = sys.stdin.read(1) #lendo a tecla digitada

                if key != '\x1b': #Eu nem sabia que isso dava inicio a sequencia ASCI AAAA
                    return key #retorna a tecla digitada
                
                key += sys.stdin.read(1) #Python é engraçado, vc soma letras e ele concatena elas
                key += sys.stdin.read(1) #mo loucura isso

                if key == '\x1b[A': #seta para cima
                    return 'UP'
                elif key == '\x1b[B': #seta para baixo
                    return 'DOWN'
                elif key == '\x1b[C': #seta para direita
                    return 'RIGHT'
                elif key == '\x1b[D': #seta para esquerda
                    return 'LEFT'
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings) #voltando o terminal para o estado inicial antes de passalo para o modo "setcbreak"

    #função para rodar tudo
    def run(self):
        while rclpy.ok(): #criando um loop que perdura enquanto o ros2 estiver rodando

            key = self.read_key() #pegando a tecla que foi digitada
            
            if key is None:
                continue
            else:
                key = key.upper() # converte para maiúscula para não dar erro se apertar minúsculas

            # dicionário para mapear teclas → números
            #para talvez organizar algo toma ai a ideia da dupla (tipo, valor)

            # formatação - > 1° numero: tipo da tecla | 2° e 3° numero: qual tecla
            # 1° numero -> 0: movimentação  1: numeros  2:letras

            key_map = {
                'W': 0,
                'S': 1,
                'D': 2,
                'A': 3,
                
                'UP': 4,
                'DOWN': 5,
                'RIGHT': 6,
                'LEFT': 7,

                '1': 101,
                '2': 102,
                '3': 103,
                '4': 104,
                '5': 105,
                '6': 106,

                "R": 201,
                "T": 202
            }

            

            #caso alguma tecla tenha sido digitada
            if key in key_map:
                msg = Int32() #define o tipo da msg
                msg.data = key_map[key] #salva na tecla na msg
                self.publisher.publish(msg) #publica a msg
            else:
                msg = Int32() #define o tipo da msg
                msg.data = -1 #salva na tecla na msg
                self.publisher.publish(msg) #publica a msg

#criando o main
def main():
    rclpy.init() #iniciando o ros
    node = KeyboardPublisher() #publicando

    #sugestão do chat gpt para quando aperta CRLT+C n dar erro, mas n impediu de dar erro kkk
    #try:
    #    node.run()
    #except KeyboardInterrupt:
    #    pass
    node.run() #roando o loop
    rclpy.shutdown() #parando o ros

#main :D
if __name__ == "__main__":
    main()