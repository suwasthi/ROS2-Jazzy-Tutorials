#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import os

class TwistFromDatabase(Node):
    def __init__(self):
        super().__init__('twist_from_database')

        self.publisher_ = self.create_publisher(Twist, 'twist_from_database', 20)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Prepare to read CSV
        self.csv_file_path = os.path.join(os.path.dirname(__file__), 'values.csv')
        self.csv_lines = []
        self.current_line = 0

        # Read all lines at startup
        try:
            with open(self.csv_file_path, 'r') as f:
                reader = csv.reader(f)
                self.csv_lines = list(reader)
            self.get_logger().info(f"Loaded {len(self.csv_lines)} lines from values.csv")
        except Exception as e:
            self.get_logger().error(f"Failed to read values.csv: {e}")

    def timer_callback(self):
        if not self.csv_lines:
            return  # No data to publish

        # Wrap around if reach end
        if self.current_line >= len(self.csv_lines):
            self.current_line = 0

        line = self.csv_lines[self.current_line]
        self.current_line += 1

        # Parse floats, ignore whitespace
        try:
            values = [float(v.strip()) for v in line]
            if len(values) != 6:
                self.get_logger().warn(f"Line {self.current_line} malformed, expected 6 values, got {len(values)}")
                return
        except ValueError:
            self.get_logger().warn(f"Line {self.current_line} contains non-float values")
            return

        twist_msg = Twist()
        twist_msg.linear.x = values[0]
        twist_msg.linear.y = values[1]
        twist_msg.linear.z = values[2]
        twist_msg.angular.x = values[3]
        twist_msg.angular.y = values[4]
        twist_msg.angular.z = values[5]

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Published twist from line {self.current_line}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistFromDatabase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
