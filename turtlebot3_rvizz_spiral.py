# # The following script makes the turtlebot spiral on either rvizz or gazebo simulation. The reason it is different from turtlesim simulation is because it utilizes different topics.
# The class uses self.publisher to publish to topic 'cmd_vel' of Twist message type provides the robot with directions. self.subscription subscribes to the topic 'odom' to access turtlebot's present time location
# The self.timer calls back to publish_velocity method every 10 seconds so that a new goal is generated. self.subscriber calls back pose_callback method which takes coordintes 
# publish_velocity method has parameters set with conditional statements which tells it when to end the spiral, how much incremental radius of spiral should be and what the angular velocity should be as well.
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SpiralMotion(Node):

    def __init__(self):
        super().__init__('spiral_motion')   # name of node 
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # publishes messages of Twist type. Twist is a specific message type defined in the geometry_msgs package. It contains two vectors: linear and angular, which represent linear and angular velocities in 3D space. This type is used to command robot movement by specifying how fast it should move and rotate. It publishes to topic name '/turtle1/cmd_vel'
        self.pose_subscriber = self.create_subscription(Pose, '/pose', self.pose_callback, 10)    # This creates a subscription to Pose type messages. It essentially takes the turtle's current location. It subscribes to the following topic: '/turtle1/pose'
        self.timer = self.create_timer(1.0, self.publish_velocity)     # This creates a timer that gets triggered every 1 second calls the method self.publish_velocity
        self.get_logger().info("Turtle will start forming spiral shape")
        self.radius = 2.0   # This is the initial radius of the turtle
        self.pose_x = 0.0  
        self.pose_y = 0.0

    def pose_callback(self, pose_msg):
        self.pose_x = pose_msg.x    # assign the current coordinates of the TurtleSim turtle (received from the Pose message) to the instance variables self.pose_x and self.pose_y. 
        self.pose_y = pose_msg.y    # This allows the node to keep track of the turtle's current position, which is used for movement calculations in the publish_velocity method.

        if round(self.pose_y) == 9: # If it reaches this particular coordinate then gives the below written instruction
            self.get_logger().warn(f"About to reach the set limit at x: {self.pose_x}, y: {self.pose_y}")

    def publish_velocity(self):
        vel_msg = Twist()    # The line vel_msg = Twist() creates a new instance of the Twist message
        vel_msg.linear.x = self.radius   

        if self.pose_x < 7.0 and self.pose_y < 7.0:
            vel_msg.angular.z = 22/7
            self.radius += 0.2
        else:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
            self.get_logger().info("Spiral rotation done!!!")
            self.timer.cancel()

        self.velocity_publisher.publish(vel_msg)

def main(args=None):        # The code initializes and runs the ROS 2 node. The main function starts by initializing ROS 2 with rclpy.init(), creates a SpiralMotion node, and keeps it running with rclpy.spin()
    rclpy.init(args=args)   # When done, rclpy.shutdown() cleans up. The script runs the main function if executed directly, ensuring the node initializes, runs, and shuts down properly.
    spiral_motion = SpiralMotion()
    rclpy.spin(spiral_motion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
