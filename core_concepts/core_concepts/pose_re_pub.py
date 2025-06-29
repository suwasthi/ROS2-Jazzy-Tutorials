#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class PoseRePub(Node):
    def __init__(self):
        super().__init__('pose_re_pub')
        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose_with_covariance_stamped',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Pose, 'pose', 10)

    def callback(self, msg):
        pose_msg = Pose()
        pose_msg.position = msg.pose.pose.position
        pose_msg.orientation = msg.pose.pose.orientation
        self.publisher.publish(pose_msg)
        self.get_logger().info('Published pose message.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseRePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
