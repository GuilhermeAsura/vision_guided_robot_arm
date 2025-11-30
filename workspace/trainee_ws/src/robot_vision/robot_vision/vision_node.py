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
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/UR5e/camera_sensor/image_color', self.image_callback, 10)
        self.coord_pub = self.create_publisher(Point, '/vision/object_coordinates', 10)
        self.get_logger().info('Nó de Visão Computacional Iniciado.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Erro CvBridge: {e}')
            return

        coordinates = self.process_image(cv_image)

        if coordinates:
            cx, cy = coordinates
            point_msg = Point()
            point_msg.x = float(cx)
            point_msg.y = float(cy)
            point_msg.z = 0.0
            self.coord_pub.publish(point_msg)
            
            # Desenha círculo verde para debug
            cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def process_image(self, img):
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Intervalos de vermelho no HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        full_mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

        full_mask = cv2.erode(full_mask, None, iterations=2)
        full_mask = cv2.dilate(full_mask, None, iterations=2)

        contours, _ = cv2.findContours(full_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
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