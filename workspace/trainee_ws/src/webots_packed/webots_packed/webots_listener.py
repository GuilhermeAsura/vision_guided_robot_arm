from controller import Robot, Motor
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image # <--- NOVO: Import para imagem

class WebotsSubscriber(Node):

    def __init__(self):
        super().__init__('webots_subscriber')
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # camera setup
        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            self.get_logger().info("Câmera 'camera_sensor' ativada!")
        else:
            self.get_logger().warn("AVISO: Câmera não encontrada!")

        # camera publisher 
        self.camera_publisher = self.create_publisher(
            Image, 
            '/UR5e/camera_sensor/image_color', 
            10
        )

        self.timer = self.create_timer(self.timestep / 1000.0, self.step_webots)

        # keyboard subscriber -> keyboard_check
        self.subscription = self.create_subscription(
            Int32,
            'keyboard_input',
            self.keyboard_callback,
            10
        )

        # joint publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'ur5e/joint_targets',
            10
        )

        self.joint_default_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.0, 0.0]
        self.joints = []
        self.joint_names = []
        self.joint_positions = []
        self.joint_rotation_motor_min_max = []
        
        self.get_joints()
        self.selected = 0
        self.step = 0.1
        self.default_positions()

        self.get_logger().info("Controlador Manual + Câmera Iniciado.")

    def step_webots(self):
        # step Webots
        self.robot.step(self.timestep)
        
        # publish image
        self.publish_camera_image()

    def publish_camera_image(self):
        if self.camera:
            raw_image = self.camera.getImage()
            if raw_image:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link_optical'
                msg.height = self.camera.getHeight()
                msg.width = self.camera.getWidth()
                msg.encoding = 'bgra8' # -> webots retorns BGRA
                msg.step = 4 * self.camera.getWidth()
                msg.data = raw_image
                self.camera_publisher.publish(msg)

    def default_positions(self):
        for i, joint in enumerate(self.joints):
            joint.setPosition(self.joint_default_positions[i])
            self.joint_positions[i] = self.joint_default_positions[i]
        self.publish_joints()

    def get_joints(self):
        n_devices = self.robot.getNumberOfDevices()
        for i in range(n_devices):
            dev = self.robot.getDeviceByIndex(i)
            if isinstance(dev, Motor):
                self.joints.append(dev)
                self.joint_names.append(dev.getName())
                self.joint_positions.append(dev.getTargetPosition())
                self.joint_rotation_motor_min_max.append((dev.getMinPosition(), dev.getMaxPosition()))

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = list(self.joint_positions)
        self.publisher.publish(msg)

    def keyboard_callback(self, msg):
        key = msg.data
        if key == 201: # R
            self.default_positions()
            return
        
        # joints selection: 101-108
        if 101 <= key <= 108:
            val = key - 101
            if 0 <= val < len(self.joints):
                self.selected = val
                self.get_logger().info(f"Junta {self.selected + 1} selecionada")
            return

        # motion: W/S/UP/DOWN
        if key in (0, 4): # UP/W
            new_pos = self.joint_positions[self.selected] + self.step
            if new_pos <= self.joint_rotation_motor_min_max[self.selected][1]:
                self.joint_positions[self.selected] = new_pos
        elif key in (1, 5): # DOWN/S
            new_pos = self.joint_positions[self.selected] - self.step
            if new_pos >= self.joint_rotation_motor_min_max[self.selected][0]:
                self.joint_positions[self.selected] = new_pos
        else:
            return

        self.joints[self.selected].setPosition(self.joint_positions[self.selected])
        self.publish_joints()
        self.get_logger().info(f"Junta {self.selected+1}: {self.joint_positions[self.selected]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = WebotsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()