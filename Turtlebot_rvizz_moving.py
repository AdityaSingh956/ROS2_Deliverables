# The following script makes the turtlebot move to random location on either rvizz or gazebo simulation. The reason it is different from turtlesim simulation is because it utilizes different topics.
# The class uses self.publisher to publish to topic 'cmd_vel' of Twist message type provides the robot with directions. self.subscription subscribes to the topic 'odom' to access turtlebot's present time location
# The self.timer calls back to generate_random_goal method every 10 seconds so that a new goal is generated. self.subscriber calls back odom_callback method which takes coordintes and converts it to euler coordinates which are easier to understand.
# move_towards_goal method uses dist_to_target eqn and angle_to_goal eqn to estimate how far way it is from goal and parameters within the if/else conditionals optimizes its trajectory.

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import random
import math
from time import sleep

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')  # Name of the node
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10) # Odometry uses data from wheel encoders and inertial measurement units (IMUs) to estimate the robot's current position (x, y) and orientation (yaw) which is rotation around vertical axis.
        self.timer = self.create_timer(10, self.generate_random_goal) # calls generate_random_goal method 
        self.target_x = None
        self.target_y = None
        self.current_x = None
        self.current_y = None
        self.yaw = None

    def generate_random_goal(self):
        self.target_x = random.uniform(-5.0, 5.0)
        self.target_y = random.uniform(-5.0, 5.0)
        self.get_logger().info(f'New goal at: x={self.target_x:.2f}, y={self.target_y:.2f}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation  # is the yaw coordinate
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)  # not sure what this is doing
        if self.target_x is not None and self.target_y is not None:
            self.move_towards_goal()

    def move_towards_goal(self):
        angle_to_goal = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        distance_to_goal = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        
        cmd = Twist()
        if distance_to_goal > 0.1:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.4 * (angle_to_goal - self.yaw)
        else:
            self.get_logger().info('Goal reached!')
            self.target_x = None
            self.target_y = None
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)            # The code initializes and runs the ROS 2 node.
    node = TurtleBot3Controller()      # The main function starts by initializing ROS 2 with rclpy.init(), creates a SpiralMotion node, and keeps it running with rclpy.spin()
    rclpy.spin(node)                 # When done, rclpy.shutdown() cleans up.
    node.destroy_node()              # The script runs the main function if executed directly, ensuring the node initializes, runs, and shuts down properly.
    rclpy.shutdown()

if __name__ == '__main__':
    main()