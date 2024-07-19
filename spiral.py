#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralCleanTurtle(Node):

    def __init__(self):
        super().__init__('spiral_clean_turtle')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_in_spiral)
        self.linear_velocity = 0.5
        self.angular_velocity = 4.0

    def move_in_spiral(self):
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity
        self.publisher.publish(cmd)
        
        self.linear_velocity += 0.01  # Gradually increase the linear velocity to form a spiral

        if self.linear_velocity > 3.0:  # Reset if the velocity gets too high
            self.linear_velocity = 0.5

def main(args=None):
    rclpy.init(args=args)
    node = SpiralCleanTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
