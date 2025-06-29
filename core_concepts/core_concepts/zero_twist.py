#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ZeroTwist(Node):
    def __init__(self):
        super().__init__('zero_twist')

        self.subscription = self.create_subscription(
            String,
            'is_stopped',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, 'twist', 10)
        self.should_publish = False

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz check

    def listener_callback(self, msg):
        if msg.data == 'true':
            self.should_publish = True
            self.get_logger().info("Received 'true': will publish zero twist")
        else:
            self.should_publish = False
            self.get_logger().info("Received 'false': will NOT publish")

    def timer_callback(self):
        if self.should_publish:
            twist_msg = Twist()
            # All fields are zero by default
            self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZeroTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
