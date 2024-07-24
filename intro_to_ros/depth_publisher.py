#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from intro_to_ros import pressure_sens

class DepthPublisher(Node):
    def __init__(self):
        super().__init__("depth_publisher")
        qos = qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            Altitude,
            "/bluerov2/depth",
            qos
        )
        self.loop = self.create_timer(1, self.depth_callback)
        self.get_logger().info("Depth Publisher Node started")

    def depth_callback(self):
        msg = pressure_sens.rov_depth
        self.publisher.publish(msg)
        self.get_logger().info(f"Depth:{pressure_sens.rov_depth}")
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()
    try:
        rclpy.spin(node)
   
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
