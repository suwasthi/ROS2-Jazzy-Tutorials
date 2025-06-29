#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_service import SetParametersResult  # <--- correct import

class ParamReader(Node):
    def __init__(self):
        super().__init__('param_reader')

        self.declare_parameter('robot_name', '')

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        if robot_name:
            self.get_logger().info(f"robot_name parameter at startup: {robot_name}")
        else:
            self.get_logger().warn("Parameter 'robot_name' is not set or empty.")

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'robot_name':
                if param.type_ == Parameter.Type.STRING:
                    self.get_logger().info(f"robot_name parameter set to: {param.value}")
                else:
                    self.get_logger().warn(f"robot_name parameter has wrong type: {param.type_}")
        return SetParametersResult(successful=True)  # <--- return this, imported from parameter_service

def main(args=None):
    rclpy.init(args=args)
    node = ParamReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
