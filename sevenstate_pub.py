#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher_node')
        self.pub = self.create_publisher(Float64MultiArray, 'state_pub', 10)
        self.timer = self.create_timer(1, self.publish_state_pub)
        self.counter = 0

    def publish_state_pub(self):
        msg = Float64MultiArray()
        state_7d = [random.uniform(-100, 100) for _ in range(7)]
        msg.data = state_7d
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {state_7d}')

def main(args=None):
    rclpy.init(args=args)
    my_pub = StatePublisher()
    print('State 7D Publisher Node Running...')

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print('Terminating Node...')
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

