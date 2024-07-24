#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np

class Pressure(Node):
    def __init__(self):
        super().__init__("Status_Presure")
        self.Static_Pressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.callback_static,
            QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        )
        self.Diff_Pressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/diff_pressure",
            self.callback_diff,
            QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        )
        self.get_logger().info("starting subscriber node")
    def callback_static(self, msg):
        self.Static_P = msg.fluid_pressure
        self.get_logger().info(f"Static Pressure: {self.Static_P}")
    def callback_diff(self, msg):
        self.Diff_P = msg.fluid_pressure
        self.get_logger().info(f"Static Pressure: {self.Diff_P}")
def main(args=None):
    rclpy.init(args=args)
    node = Pressure()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()
    

# ros2 topic pub /mavros/imu/static_pressure sensor_msgs/msg/FluidPressure