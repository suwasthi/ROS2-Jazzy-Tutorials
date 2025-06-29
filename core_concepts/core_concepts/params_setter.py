#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamsSetter(Node):
    def __init__(self):
        super().__init__('params_setter')

        # Define param and value variables here
        param = 'my_global_param'
        value = 'Hello_ROS2'

        # Declare the parameter first (with default value)
        self.declare_parameter(param, '')

        # Set the global parameter to the desired value
        self.set_parameters([Parameter(param, Parameter.Type.STRING, value)])

        self.get_logger().info(f"Set parameter '{param}' to '{value}'")

def main(args=None):
    rclpy.init(args=args)
    node = ParamsSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
