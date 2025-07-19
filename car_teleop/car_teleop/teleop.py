#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class PS3Teleop(Node):
    def __init__(self):
        super().__init__('ps3_teleop')

        self.throttle_pub = self.create_publisher(Float64, '/audibot/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/audibot/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/audibot/brake_cmd', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info("✅ PS3 teleop for AudiBot started.")

    def joy_callback(self, msg: Joy):
        # --- PS3 mapping (adjust if needed) ---
        # Left stick vertical (axes[1]): -1 forward, +1 backward
        # Right stick horizontal (axes[3]): -1 left, +1 right
        # Right trigger R2 (axes[5]): 1 released, -1 fully pressed

        throttle = max(msg.axes[1], 0.0)
        steering = msg.axes[3]*1.57
        if 1 - msg.axes[5] > 0:
            brake = 10000.0
        else:
            brake = 0.0

        self.throttle_pub.publish(Float64(data=throttle))
        self.steering_pub.publish(Float64(data=steering))
        self.brake_pub.publish(Float64(data=brake))

def main(args=None):
    rclpy.init(args=args)
    node = PS3Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
