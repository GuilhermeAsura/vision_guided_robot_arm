from controller import Robot, Keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

class UR5eHybridController:
    def __init__(self, ros_node=None):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            
        # Nomes das juntas (incluindo garra se necessário)
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joints = []
        for name in self.joint_names:
            dev = self.robot.getDevice(name)
            if dev: self.joints.append(dev)

        self.current_joint = 0
        self.joint_step = 0.05
        self.tracking_active = False
        
        # ROS2
        self.node = ros_node
        if self.node:
            self.camera_publisher = self.node.create_publisher(Image, '/UR5e/camera_sensor/image_color', 10)
            self.vision_subscriber = self.node.create_subscription(Point, '/vision/object_coordinates', self.vision_callback, 10)
            self.node.get_logger().info("Híbrido Iniciado. Pressione 'T' para seguir objeto.")

    def publish_camera_image(self):
        if self.node and self.camera:
            raw_image = self.camera.getImage()
            if raw_image:
                msg = Image()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link_optical'
                msg.height = self.camera.getHeight()
                msg.width = self.camera.getWidth()
                msg.encoding = 'bgra8' 
                msg.step = 4 * self.camera.getWidth()
                msg.data = raw_image
                self.camera_publisher.publish(msg)

    def move_single_joint(self, delta, joint_idx=None):
        idx = joint_idx if joint_idx is not None else self.current_joint
        if 0 <= idx < len(self.joints):
            current = self.joints[idx].getTargetPosition()
            # Limites simples para evitar quebra
            new_pos = current + delta
            self.joints[idx].setPosition(new_pos)

    def vision_callback(self, msg):
        if not self.tracking_active: return
        
        # Lógica de controle simples (Visual Servoing)
        img_center_x = self.camera.getWidth() / 2
        img_center_y = self.camera.getHeight() / 2
        error_x = img_center_x - msg.x
        error_y = img_center_y - msg.y
        
        kp = 0.001 # Ganho proporcional
        
        if abs(error_x) > 20:
            self.move_single_joint(kp * error_x, joint_idx=0) # Base gira
        if abs(error_y) > 20:
            self.move_single_joint(-kp * error_y, joint_idx=1) # Ombro sobe/desce

    def run(self):
        while self.robot.step(self.timestep) != -1:
            if self.node:
                self.publish_camera_image()
                rclpy.spin_once(self.node, timeout_sec=0)

            key = self.keyboard.getKey()
            if key == ord('T'):
                self.tracking_active = not self.tracking_active
                print(f"Tracking: {self.tracking_active}")
            elif key == Keyboard.UP:
                self.move_single_joint(self.joint_step)
            elif key == Keyboard.DOWN:
                self.move_single_joint(-self.joint_step)
            elif ord('1') <= key <= ord('6'):
                self.current_joint = key - ord('1')

def main(args=None):
    rclpy.init(args=args)
    node = Node('ur5e_hybrid_node')
    controller = UR5eHybridController(ros_node=node)
    controller.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()