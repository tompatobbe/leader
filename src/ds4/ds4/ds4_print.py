import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class DS4PrintNode(Node):
    def __init__(self):
        super().__init__('ds4_print_node')
        
        # Subscribe to joystick
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        
        # Publisher for Servo (Left Stick)
        self.servo_publisher = self.create_publisher(
            Float32,
            '/servo/angle',
            10)
            
        # Publisher for R2 (Trigger)
        self.r2_publisher = self.create_publisher(
            Float32,
            '/r2_value',
            10)
        
        self.get_logger().info("Node started. Press R2 to publish to /r2_value")

        # Settings
        self.min_servo_angle = 0
        self.max_servo_angle = 60
        self.current_angle = 30

    def listener_callback(self, msg):
        # ---------------------------------------------------------
        # 1. Handle R2 Trigger (Axis 5)
        # Based on your logs: Unpressed = 1.0, Pressed = -1.0
        # ---------------------------------------------------------
        if len(msg.axes) > 5:
            raw_r2 = msg.axes[5]
            
            # Mathematics to map [1.0 ... -1.0] to [0.0 ... 1.0]
            # 1.0 (unpressed) -> (1 - 1) / 2 = 0.0
            # -1.0 (pressed)  -> (1 - (-1)) / 2 = 1.0
            normalized_r2 = (1 - raw_r2) / 2.0
            
            # Publish
            r2_msg = Float32()
            r2_msg.data = normalized_r2
            self.r2_publisher.publish(r2_msg)

        # ---------------------------------------------------------
        # 2. Handle Servo (Left Stick - Axis 0)
        # ---------------------------------------------------------
        if len(msg.axes) > 0:
            axis_val = msg.axes[0]
            if abs(axis_val) > 0.05:
                angle_range = self.max_servo_angle - self.min_servo_angle
                self.current_angle = self.min_servo_angle + (angle_range * (axis_val + 1) / 2)
            else:
                self.current_angle = (self.min_servo_angle + self.max_servo_angle) / 2
            
            servo_msg = Float32()
            servo_msg.data = self.current_angle
            self.servo_publisher.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DS4PrintNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()