#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class StateSubscriber(Node):

    def __init__(self):
        super().__init__('state_subscriber_node')
        self.subscription = self.create_subscription(Float64MultiArray,'state_pub',self.listener_callback,10)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'state_6_pub', 10)  # The topic to publish to

    def listener_callback(self, msg):
        # Extract the 7-dimensional state
        state_7d = msg.data
        # Remove the yaw component (assumed to be the last element)
        state_6d = state_7d[:-1]

        # Create a new message with the 6-dimensional state
        new_msg = Float64MultiArray()
        new_msg.data = state_6d

        # Publish the 6-dimensional state
        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Publishing: {state_6d}')

def main(args=None):
    rclpy.init(args=args)
    my_pub = StateSubscriber()
    print('State 6D Publisher Node Running...')

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print('Terminating Node...')
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

