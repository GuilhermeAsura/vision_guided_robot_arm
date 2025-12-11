import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

# TF2 Imports
import tf2_ros
import tf2_geometry_msgs


class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('depth_estimator_node')
        self.bridge = CvBridge()

        # Parâmetros da câmera 
        self.fov = 0.7854  
        self.img_w = 640
        self.img_h = 480

        # Distância focal em pixels 
        self.f = self.img_w / (2 * math.tan(self.fov / 2))

        # Centro ótico da imagem
        self.cx = self.img_w / 2
        self.cy = self.img_h / 2

        # Raio real da bola no Webots
        self.ball_radius = 0.07 

        # Parâmetros de Filtro 
        self.last_z = 0.0
        self.filter_alpha = 0.4

        self.create_subscription(Image, '/UR5e/camera_sensor/image_color', self.image_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/vision/estimated_3d_coordinates', 10)  
        # Publicador do ponto 3D no mundo
        self.publisher_world = self.create_publisher(PointStamped, '/vision/estimated_3d_world', 10)

        # Configuração do TF2 (Para pegar a posição da câmera dinamicamente) 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"[Vision] Estimador de Profundidade Ativo.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Faixas do vermelho
        mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # Contornos (maior = bola)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((u, v), radius_px) = cv2.minEnclosingCircle(c)

            if radius_px > 3:
                # MÉTODO 1: Profundidade pelo raio
                z_radius = (self.ball_radius * self.f) / radius_px

                # MÉTODO 2: Profundidade pela área
                area_px = math.pi * (radius_px ** 2)
                k = self.ball_radius * self.f * math.sqrt(math.pi)
                z_area = k / math.sqrt(area_px)

                # Aplicação do filtro para suavização usando o método do raio
                z = z_radius
                if self.last_z == 0.0:
                    z_smooth = z
                else:
                    z_smooth = (self.filter_alpha * z) + ((1 - self.filter_alpha) * self.last_z)
                self.last_z = z_smooth

                # Validação
                self.get_logger().info(
                    f"[VALIDAÇÃO] Raio em px={radius_px:.2f} | Z(raio)={z_radius:.3f} | Z(area)={z_area:.3f} | Dif={abs(z_radius - z_area):.3f}",
                    throttle_duration_sec=0.5
                )

                # Coordenadas em relação ao centro da imagem (Câmera)
                x_cam = (u - self.cx) * z_smooth / self.f
                y_cam = (v - self.cy) * z_smooth / self.f

                # Estas coordenadas dizem exatamente onde a bola está em relação à sua mão
                self.get_logger().info(f"ALVO: {z_smooth:.2f}m em frente | Desvio X: {x_cam:.2f}m | Desvio Y: {y_cam:.2f}m", throttle_duration_sec=0.5)

                # Publicação no frame da câmera
                point_cam = PointStamped()
                point_cam.header = msg.header
                if not point_cam.header.frame_id:
                    point_cam.header.frame_id = "camera_link_optical"
                point_cam.point.x = x_cam
                point_cam.point.y = y_cam
                point_cam.point.z = z_smooth
                self.publisher.publish(point_cam)

                # Transformação para o frame do mundo
                try:
                    transform = self.tf_buffer.lookup_transform('world', point_cam.header.frame_id, rclpy.time.Time())

                    point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
                    point_world.header.frame_id = self.TARGET_FRAME
                    point_world.header.stamp = self.get_clock().now().to_msg()
                    # Publicar no mundo
                    self.publisher_world.publish(point_world)

                except Exception as e:
                    self.get_logger().warn(f"[TF] Falha ao transformar para o mundo: {e}", throttle_duration_sec=1.0)

                # Desenho
                cv2.circle(frame, (int(u), int(v)), int(radius_px), (0, 255, 0), 2)
                cv2.putText(frame, f"Z: {z_smooth:.2f}m", (int(u), int(v)+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("Estimador de Profundidade", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
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
