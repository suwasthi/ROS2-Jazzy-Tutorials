#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node as RclpyNode

# Custom function to create one-time Twist publisher
def publish_twist_once():
    rclpy.init()
    tmp_node = RclpyNode('tmp_twist_publisher')
    pub = tmp_node.create_publisher(Twist, 'twist', 10)

    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    pub.publish(msg)
    tmp_node.get_logger().info("Published one Twist message with all zeros to topic 'twist'")

    rclpy.shutdown()

def generate_launch_description():
    # Create nodes
    chatter_node = Node(
        package='tests',
        executable='chatter',
        name='test_chatter',
        output='screen',
        remappings=[('/chatter', '/test/chatter')]
    )

    listener_node = Node(
        package='tests',
        executable='listener',
        name='listener',
        output='screen',
    )

    # TimerAction used to delay the custom publisher slightly (optional)
    publish_action = TimerAction(
        period=1.0,
        actions=[],
    )

    # Actually run the twist publisher once after launch starts
    publish_twist_once()

    return LaunchDescription([
        chatter_node,
        listener_node,
        publish_action,
    ])
