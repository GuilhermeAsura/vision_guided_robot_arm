#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')

        # 1. Inicialização de Infraestrutura
        """
        O ROS transmite imagens como arrays de bytes serializados (sensor_msgs/Image). 
        O OpenCV espera matrizes NumPy. O CvBridge é o Middleware padrão que faz essa tradução.
        """
        self.bridge = CvBridge()

        # Subscriber: Escuta a câmera do Webots
        self.image_sub = self.create_subscription(
            Image,
            '/UR5e/camera_sensor/image_color',
            self.image_callback,
            10)

        # Publisher: Publica as coordenadas (x, y) normalizadas ou em pixels
        """
        Optei por publicar em geometry_msgs/msg/Point. Isso facilita a integração com o nó de Controle. 
        O nó de controle apenas assina /vision/object_coordinates e recebe $x, y$ limpos, sem precisar saber 
        o que é uma câmera ou processar imagens, mantendo o princípio de Responsabilidade Única (SOLID).
        """
        self.coord_pub = self.create_publisher(Point, '/vision/object_coordinates', 10)

        self.get_logger().info('Nó de Visão Computacional Inicializado.')

    def image_callback(self, msg):
        """
        Pipeline de Processamento de Imagem
        """
        try:
            # A. Conversão ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return

        # B. Processamento (Detecção de Círculo Vermelho)
        coordinates = self.process_image(cv_image)

        # C. Publicação
        if coordinates:
            cx, cy = coordinates

            # Cria a mensagem Point (z=0 pois é imagem 2D)
            point_msg = Point()
            point_msg.x = float(cx)
            point_msg.y = float(cy)
            point_msg.z = 0.0

            self.coord_pub.publish(point_msg)
            # Log para debug (remover em produção para performance)
            # self.get_logger().info(f'Objeto detectado em: X={cx}, Y={cy}')

            # Visualização (Opcional - abre janela no host)
            cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

        # Mostra o feed processado (apenas para debug local)
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def process_image(self, img):
        """
        Lógica Pura de Visão Computacional
        Retorna: (x, y) ou None
        """
        # 1. Pré-processamento: Blur para reduzir ruído
        blurred = cv2.GaussianBlur(img, (11, 11), 0)

        # 2. Conversão de Espaço de Cor: BGR -> HSV
        """
        HSV é muito mais robusto para detecção de cor sob iluminação variável, porque detectar cores no espaço RGB 
        é frágil. No RGB, uma sombra sobre o objeto vermelho altera drasticamente os valores R, G e B. 
        No HSV (Hue, Saturation, Value), a cor (Hue) é separada da intensidade luminosa (Value). 
        Isso permite que detectar "vermelho" mesmo se ele estiver numa sombra ou sob luz forte.
        """
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 3. Máscara para a cor VERMELHA
        # O vermelho no HSV envolve o 0 (vai de 170-180 e 0-10)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        full_mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

        # 4. Operações Morfológicas (Limpeza)
        # Remove pequenos ruídos brancos na máscara
        full_mask = cv2.erode(full_mask, None, iterations=2)
        full_mask = cv2.dilate(full_mask, None, iterations=2)

        # 5. Encontrar Contornos
        contours, _ = cv2.findContours(full_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Assume que o maior contorno é o objeto alvo
            c = max(contours, key=cv2.contourArea)

            # Calcula o momento para achar o centroide
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)

        return None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()