import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs

class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('depth_estimator_node')
        self.bridge = CvBridge()

        # Frame da câmera (deve ser o mesmo definido no Webots/URDF)
        self.CAMERA_FRAME = 'camera_link_optical' 
        # Frame de destino para Cinemática (ex: base do robô)
        self.TARGET_FRAME = 'base_link'           

        # Parâmetros Intrínsecos (Baseado no Webots)
        self.fov = 0.7854   # 45 graus
        self.img_w = 640
        self.ball_radius = 0.07 # Raio real da bola (metros)

        # Distância focal (pixels)
        self.f = self.img_w / (2 * math.tan(self.fov / 2))
        self.cx = 320 # Centro X da imagem
        self.cy = 240 # Centro Y da imagem

        # Publishers
        self.pub_cam_frame = self.create_publisher(PointStamped, '/vision/target_camera_frame', 10)
        self.pub_world_frame = self.create_publisher(PointStamped, '/vision/target_world_frame', 10)
        
        # Subscriber
        self.create_subscription(Image, '/UR5e/camera_sensor/image_color', self.image_callback, 10)

        # Buffer de Transformação (TF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Depth Estimator (Robot Extra) Iniciado!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        # 1. Detectar Bola (Processamento de Imagem)
        center, radius_px = self.detect_ball(cv_image)

        if center is not None and radius_px > 3:
            u, v = center
            
            # 2. Calcular Z (Profundidade)
            # Z = (f * R_real) / R_pixel
            z_cam = (self.f * self.ball_radius) / radius_px

            # 3. Calcular X e Y no frame da câmera
            x_cam = (u - self.cx) * z_cam / self.f
            y_cam = (v - self.cy) * z_cam / self.f

            # 4. Criar mensagem PointStamped (Frame da Câmera)
            point_cam = PointStamped()
            point_cam.header = msg.header
            # Se o header vier vazio do Webots, forçamos o nome
            if not point_cam.header.frame_id:
                point_cam.header.frame_id = self.CAMERA_FRAME
            
            point_cam.point.x = x_cam
            point_cam.point.y = y_cam
            point_cam.point.z = z_cam
            
            # Publica sempre (útil para Visual Servoing relativo)
            self.pub_cam_frame.publish(point_cam)

            # 5. Tentar Transformar para o Mundo/Base (Cinemática Inversa Global)
            try:
                # Procura a transformação mais recente disponível
                transform = self.tf_buffer.lookup_transform(
                    self.TARGET_FRAME,
                    point_cam.header.frame_id,
                    rclpy.time.Time()) # Time(0) pega a última disponível
                
                point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
                self.pub_world_frame.publish(point_world)
                
                # Debug no terminal
                self.get_logger().info(
                    f"ALVO MUNDO -> X: {point_world.point.x:.2f} | Y: {point_world.point.y:.2f} | Z: {point_world.point.z:.2f}", 
                    throttle_duration_sec=1.0)

            except Exception as e:
                # Se falhar o TF, avisamos mas não paramos o nó
                self.get_logger().warn(f"Sem TF disponível: {e}. Usando apenas coordenadas da câmera.", throttle_duration_sec=2.0)

            # Desenho para Debug
            cv2.circle(cv_image, (int(u), int(v)), int(radius_px), (0, 255, 0), 2)
            cv2.putText(cv_image, f"Z: {z_cam:.2f}m", (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        cv2.imshow("Robot Extra Vision", cv_image)
        cv2.waitKey(1)

    def detect_ball(self, img):
        # Lógica de detecção idêntica ao vision_node original que funcionava bem
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            return (x, y), radius
        return None, 0

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()