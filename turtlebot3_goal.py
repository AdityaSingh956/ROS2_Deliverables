#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import random
import math

class RandomTurtleBot3Mover(Node):

    def __init__(self):
        super().__init__('random_turtlebot3_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10))
        self.timer = self.create_timer(10, self.timer_callback)
        self.cmd = Twist()
        self.current_position = (0.0, 0.0)
        self.target_position = self.generate_random_goal()
        self.get_logger().info(f'New goal: {self.target_position}')

    def generate_random_goal(self):
        x = random.uniform(-2.0, 2.0)
        y = random.uniform(-2.0, 2.0)
        return (x, y)

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def euclidean_distance(self, goal_pos):
        return math.sqrt((goal_pos[0] - self.current_position[0]) ** 2 +
                         (goal_pos[1] - self.current_position[1]) ** 2)

    def timer_callback(self):
        distance_tolerance = 0.1
        self.target_position = self.generate_random_goal()
        self.get_logger().info(f'Moving to new goal: {self.target_position}')

        while rclpy.ok():
            distance = self.euclidean_distance(self.target_position)
            if distance <= distance_tolerance:
                break

            self.move_to_goal()
            rclpy.spin_once(self)

        self.stop_robot()

    def move_to_goal(self):
        angle_to_goal = math.atan2(self.target_position[1] - self.current_position[1],
                                   self.target_position[0] - self.current_position[0])
        distance = self.euclidean_distance(self.target_position)

        linear_speed = 0.2 * distance
        angular_speed = 1.0 * (angle_to_goal - self.get_yaw())

        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        self.publisher_.publish(self.cmd)

    def get_yaw(self):
        return 0.0  # Simplified for this example

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RandomTurtleBot3Mover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
