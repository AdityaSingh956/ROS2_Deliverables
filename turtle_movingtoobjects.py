#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
import random
import math


class CatchObjectsTurtle(Node):

    def __init__(self):
        super().__init__('catch_objects_turtle')
        self.client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10) # pose is turtle's current location
        self.timer = self.create_timer(10, self.generate_random_object)
        self.target_x = None
        self.target_y = None
        self.current_x = None
        self.current_y = None

    def generate_random_object(self): # Generates random location
        self.target_x = random.uniform(0.0, 11.0)
        self.target_y = random.uniform(0.0, 11.0)
        self.get_logger().info(f'New object at: x={self.target_x:.2f}, y={self.target_y:.2f}')

    def pose_callback(self, msg): # gets activated when we subscribe to pose
        self.current_x = msg.x # we use msg.x as we are taking in messages from pose
        self.current_y = msg.y
        if self.target_x is not None and self.target_y is not None:
            self.move_towards_object()

    def move_towards_object(self):
        angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        distance_to_target = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        cmd = Twist()
        if distance_to_target > 0.5:
            cmd.linear.x = 1.0
            cmd.angular.z = angle_to_target
        else:
            self.get_logger().info('Object caught!')
            self.target_x = None
            self.target_y = None
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CatchObjectsTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
