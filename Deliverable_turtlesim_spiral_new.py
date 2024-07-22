#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SpiralMotion(Node):

    def __init__(self):
        super().__init__('spiral_motion')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.get_logger().info("Turtle will start forming spiral shape")
        self.radius = 2.0
        self.pose_x = 0.0
        self.pose_y = 0.0

    def pose_callback(self, pose_msg):
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y

        if round(self.pose_y) == 9:
            self.get_logger().warn(f"About to reach the set limit at x: {self.pose_x}, y: {self.pose_y}")

    def publish_velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.radius

        if self.pose_x < 10.0 and self.pose_y < 10.0:
            vel_msg.angular.z = 22/7
            self.radius += 0.2
        else:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
            self.get_logger().info("Spiral rotation done!!!")
            self.timer.cancel()

        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    spiral_motion = SpiralMotion()
    rclpy.spin(spiral_motion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
