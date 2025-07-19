#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from audibot_gazebo.msg import CarStatus
import math

class CarStatusNode(Node):
    def __init__(self):
        super().__init__('car_status_node')

        self.v_forward = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.a_y = 0.0

        self.sub_odom = self.create_subscription(Odometry, '/audibot/odom', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/audibot/imu', self.imu_callback, 50)
        self.pub_status = self.create_publisher(CarStatus, '/audibot/car_status', 10)
        self.get_logger().info("✅ CarStatusNode with IMU started.")

    def odom_callback(self, msg: Odometry):
        self.v_forward = msg.twist.twist.linear.x

        # Heading from odometry
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.publish_status()

    def imu_callback(self, msg: Imu):
        self.yaw_rate = msg.angular_velocity.z
        self.a_y = msg.linear_acceleration.y

    def publish_status(self):
        velocity = self.v_forward

        # Compute slide angle using IMU
        if abs(self.yaw_rate) > 0.001 and velocity > 0.1:
            v_y = self.a_y / self.yaw_rate
        else:
            v_y = 0.0

        slide_angle = math.degrees(math.atan2(v_y, velocity)) if velocity > 0.1 else 0.0
        is_drifting = abs(slide_angle) > 5.0 if velocity > 0.1 else False

        status = CarStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.velocity = velocity
        status.heading = math.degrees(self.yaw)
        status.slide_angle = slide_angle
        status.is_drifting = is_drifting

        self.pub_status.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = CarStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
