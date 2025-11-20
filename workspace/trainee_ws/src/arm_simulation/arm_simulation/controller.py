import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_sine_wave_controller')
        
        # topic name to publish the position command
        topic_name = '/Ur5e/shoulder_pan_joint/set_position'
        
        # create publisher -> msg type: float64 
        self.publisher_ = self.create_publisher(Float64, topic_name, 10)
        
        # timer (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Controlador iniciado! Publicando em: {topic_name}')
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # create message
        msg = Float64()
        
        # sinoidal motion: 1.0 rad -> pos = sin(tempo)
        msg.data = math.sin(elapsed)
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Sending command: {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()