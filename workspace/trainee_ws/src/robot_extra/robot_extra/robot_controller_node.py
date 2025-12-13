from controller import Robot, Motor
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PointStamped
import time

class WebotsSubscriber(Node):

    def __init__(self):
        super().__init__('webots_subscriber')
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # camera setup
        self.camera = self.robot.getDevice('camera_sensor')
        if self.camera:
            self.camera.enable(self.timestep)
            self.get_logger().info("camera active")

        self.camera_publisher = self.create_publisher(
            Image, '/UR5e/camera_sensor/image_color', 10)

        self.timer = self.create_timer(self.timestep / 1000.0, self.step_webots)

        self.subscription = self.create_subscription(
            Int32, 'keyboard_input', self.keyboard_callback, 10)
        
        self.vision_sub = self.create_subscription(
            PointStamped, '/vision/target_camera_frame', self.vision_callback, 10)
        
        self.tracking_active = False 
        self.last_vision_time = 0.0

        self.publisher = self.create_publisher(
            Float64MultiArray, 'ur5e/joint_targets', 10)

        self.joint_default_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.joints = []
        self.joint_positions = []
        self.joint_limits = []
        
        self.init_joints()
        self.selected = 0
        self.step = 0.1
        self.reset_robot()

        self.get_logger().info("controller ready. 't' to track, 'r' to reset.")
    
    def step_webots(self):
        self.robot.step(self.timestep)
        self.publish_camera_image()

        if self.tracking_active:
            # timeout: stop if object lost > 0.5s
            if (time.time() - self.last_vision_time) > 0.5:
                pass

    def publish_camera_image(self):
        if self.camera:
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
                self.camera_publisher.publish(msg)

    def vision_callback(self, msg):
        self.last_vision_time = time.time()
        
        if not self.tracking_active:
            return

        ex = msg.point.x 
        ey = msg.point.y 
        z_dist = msg.point.z

        # FILTER 1: IGNORE NOISE (Z jumps to 3m, 11m in logs)
        # We only accept realistic depth for a table-top task
        if z_dist > 2.0 or z_dist < 0.1: 
            return

        target_z = 0.30
        deadzone = 0.03 # increased slightly
        
        # TUNING GAINS
        kp_yaw = 1.2     # Base rotation
        kp_pitch = 0.8   # Shoulder height
        kp_zoom = 0.4    # Arm extension (Increased from 0.15)

        updated = False

        # --- AXIS 1: ALIGN YAW (Base) ---
        if abs(ex) > deadzone:
            self.joint_positions[0] += (ex * kp_yaw * 0.1)
            updated = True

        # --- AXIS 2: ALIGN PITCH (Shoulder) ---
        if abs(ey) > deadzone:
            self.joint_positions[1] -= (ey * kp_pitch * 0.1)
            updated = True

        # --- AXIS 3: APPROACH (Elbow) ---
        # Relaxed logic: approach if roughly aligned (was 0.15, now 0.30)
        # This prevents the robot from freezing Z when X/Y are fighting
        if abs(ex) < 0.30 and abs(ey) < 0.30:
            ez = z_dist - target_z
            
            if abs(ez) > 0.05:
                # Log action to terminal so user knows it's trying to move
                if ez > 0: 
                    print(f">> EXTENDING: {z_dist:.2f}m", end='\r')
                
                # Move Elbow (Joint 2)
                offset = ez * kp_zoom * 0.1
                self.joint_positions[2] -= offset 
                
                # Compensate Wrist 1 (Joint 3) to keep camera straight
                # If camera dips down, reduce this factor (e.g. 0.8 * offset)
                self.joint_positions[3] += (offset * 0.5)
                
                updated = True

        if updated:
            self.apply_joints()

    def reset_robot(self):
        self.joint_positions = list(self.joint_default_positions)
        self.apply_joints()

    def init_joints(self):
        n = self.robot.getNumberOfDevices()
        for i in range(n):
            d = self.robot.getDeviceByIndex(i)
            if isinstance(d, Motor):
                self.joints.append(d)
                self.joint_positions.append(d.getTargetPosition())
                self.joint_limits.append((d.getMinPosition(), d.getMaxPosition()))

    def apply_joints(self):
        msg = Float64MultiArray()
        msg.data = []
        for i, val in enumerate(self.joint_positions):
            if i < len(self.joint_limits):
                mn, mx = self.joint_limits[i]
                if mn != mx: 
                    val = max(mn, min(val, mx))
            
            self.joints[i].setPosition(val)
            self.joint_positions[i] = val 
            msg.data.append(val)
        
        self.publisher.publish(msg)

    def keyboard_callback(self, msg):
        k = msg.data
        if k == 202: # T
            self.tracking_active = not self.tracking_active
            s = "ON" if self.tracking_active else "OFF"
            self.get_logger().info(f"tracking {s}")
            return
        if k == 201: # R
            self.tracking_active = False 
            self.reset_robot()
            self.get_logger().info("reset done")
            return
        
        if self.tracking_active and k in (0,1,4,5): return

        if 101 <= k <= 108:
            idx = k - 101
            if idx < len(self.joints):
                self.selected = idx
                self.get_logger().info(f"joint {idx} selected")
            return

        if k in (0, 4): 
            self.joint_positions[self.selected] += self.step
        elif k in (1, 5): 
            self.joint_positions[self.selected] -= self.step
        else:
            return

        self.apply_joints()

def main(args=None):
    rclpy.init(args=args)
    node = WebotsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()