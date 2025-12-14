import os
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, JointState # [NOVO] Importando JointState
from std_msgs.msg import Float64MultiArray, Int32, Header # [NOVO] Importando Header
from controller import Robot
from ikpy.chain import Chain

class IKControllerNode(Node):
    def __init__(self):
        super().__init__('ik_controller_node')
        
        # webots setup
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.timer = self.create_timer(self.timestep/1000.0, self.step_webots)
        
        # hardware setup
        self.joints = []
        self.sensors = [] 
        self.joint_limits = [] 
        # Nomes das juntas conforme definido no URDF (IMPORTANTE)
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.setup_motors()
        self.setup_camera() 
        
        # ikpy setup
        self.chain = self.load_urdf()
        
        # ros comms
        self.vision_sub = self.create_subscription(
            PointStamped, '/vision/target_world_frame', self.vision_cb, 10)
        
        self.keyboard_sub = self.create_subscription(
            Int32, 'keyboard_input', self.keyboard_cb, 10)
            
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'ur5e/joint_targets', 10)
            
        # [NOVO] Publisher de Estado das Juntas para o robot_state_publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # state
        self.target_cam_xyz = None
        self.active = False
        self.state = "IDLE" 
        
        # manual control state
        self.selected_joint = 0
        self.step_size = 0.1
        
        self.get_logger().info("ik ready. 't': track, 'r': reset, '1-6': select, 'w/s': move")

    def setup_camera(self):
        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            self.cam_pub = self.create_publisher(Image, '/UR5e/camera_sensor/image_color', 10)
        else:
            self.get_logger().warn("camera_sensor not found")

    def setup_motors(self):
        for n in self.joint_names:
            m = self.robot.getDevice(n)
            
            # fallback for sensor naming convention in Webots
            s_name = n + '_sensor'
            s = self.robot.getDevice(s_name)
            
            if s: s.enable(self.timestep)
            else: self.get_logger().warn(f"sensor {s_name} not found")

            if m: 
                self.joints.append(m)
                self.sensors.append(s)
                self.joint_limits.append((m.getMinPosition(), m.getMaxPosition()))
            else:
                self.get_logger().error(f"motor {n} not found")

    def load_urdf(self):
        path = "/trainee/workspace/trainee_ws/src/robot_vision/robot_vision/resource/robot_driver.urdf"
        if not os.path.exists(path):
            self.get_logger().error(f"urdf not found at {path}")
            return None
        self.get_logger().info(f"urdf loaded: {path}")
        return Chain.from_urdf_file(path, active_links_mask=[False, True, True, True, True, True, True])

    def step_webots(self):
        if self.robot.step(self.timestep) == -1:
            rclpy.shutdown()
        
        self.publish_cam()
        self.publish_joint_states() # [NOVO] Publica o estado atual para o TF

        if self.active and self.state == "PLANNING":
            self.execute_logic()

    # [NOVO] Função essencial para o funcionamento do TF
    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names # Nomes devem bater com o URDF
        
        current_positions = []
        for i, s in enumerate(self.sensors):
            if s: 
                current_positions.append(s.getValue())
            else: 
                current_positions.append(self.joints[i].getTargetPosition())
        
        msg.position = current_positions
        self.joint_state_pub.publish(msg)

    def publish_cam(self):
        if hasattr(self, 'camera') and self.camera:
            raw = self.camera.getImage()
            if raw:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link_optical'
                msg.height = self.camera.getHeight()
                msg.width = self.camera.getWidth()
                msg.encoding = 'bgra8'
                msg.step = 4 * self.camera.getWidth()
                msg.data = raw
                self.cam_pub.publish(msg)

    def vision_cb(self, msg):
        self.target_cam_xyz = [msg.point.x, msg.point.y, msg.point.z]
        # Debug opcional
        # self.get_logger().info(f"Target Recebido: {self.target_cam_xyz}", throttle_duration_sec=2)

    def keyboard_cb(self, msg):
        k = msg.data
        if k == 202: # 't'
            self.active = True
            self.state = "PLANNING"
            self.get_logger().info("starting ik sequence...")
            return
        elif k == 201: # 'r'
            self.active = False
            self.state = "IDLE"
            self.reset_home()
            return

        if self.active: return

        if 101 <= k <= 106:
            idx = k - 101
            if idx < len(self.joints):
                self.selected_joint = idx
                self.get_logger().info(f"manual: joint {idx} selected")
            return

        if k in [0, 4]: self.manual_move(1)
        elif k in [1, 5]: self.manual_move(-1)

    def manual_move(self, direction):
        m = self.joints[self.selected_joint]
        curr = m.getTargetPosition()
        new_pos = curr + (direction * self.step_size)
        mn, mx = self.joint_limits[self.selected_joint]
        if mn != mx:
            if not (mn <= new_pos <= mx): return 
        m.setPosition(new_pos)

    def get_current_angles(self):
        # Retorna lista com 7 elementos (Base fixa + 6 juntas) para o IKPy
        angles = [0.0] 
        for i, s in enumerate(self.sensors):
            if s: angles.append(s.getValue())
            else: angles.append(self.joints[i].getTargetPosition()) 
        return angles

    def execute_logic(self):
        if not self.target_cam_xyz:
            self.get_logger().warn("Waiting for visual target . . .", throttle_duration_sec=2)
            return

        if not self.chain:
            self.get_logger().error("chain not loaded")
            self.active = False
            return

        current_joints = self.get_current_angles()
        target_xyz_world = self.target_cam_xyz 
        
        self.get_logger().info(f"Moving to Target (Base Frame): {target_xyz_world}")

        # IK calculation
        ik_sol = self.chain.inverse_kinematics(
            target_position=target_xyz_world,
            target_orientation=None,
            orientation_mode=None,
            initial_position=current_joints
        )

        self.move_to(ik_sol)
        self.state = "MOVED"
        self.active = False

    def move_to(self, joints):
        # apply ik solution (ignora o primeiro elemento que é a base fixa)
        target = joints[1:]
        
        msg = Float64MultiArray()
        msg.data = [float(x) for x in target]
        self.joint_target_pub.publish(msg)
        
        for i, m in enumerate(self.joints):
            if i < len(target): m.setPosition(target[i])

    def reset_home(self):
        home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        for i, m in enumerate(self.joints):
            m.setPosition(home[i])
        self.get_logger().info("reset complete")

def main(args=None):
    rclpy.init(args=args)
    node = IKControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()