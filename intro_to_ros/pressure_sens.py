#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Pressure(Node):
    def __init__(self):
        self.rov_depth = 0.0
        super().__init__("ROV_Pressure")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        self.rovPressure = self.create_subscription(
            FluidPressure,
            "/bluerov2/pressure",
            self.callback_rov_pressure,
            qos
        )
        self.get_logger().info("starting subscriber node")

        self.publisher = self.create_publisher(
            Altitude,
            "/bluerov2/depth",
            qos
        )
        self.loop = self.create_timer(0.25, self.publish_depth)
        self.get_logger().info("Depth Publisher Node started")
        
    def callback_rov_pressure(self, msg):
        self.rov_pressure = msg.fluid_pressure
        self.get_logger().info(f"Static Pressure: {self.rov_pressure}")
        self.rov_depth = (self.rov_pressure-101325)/9.81/1000
    
    def publish_depth(self):
        msg = Altitude()
        msg.local = self.rov_depth
        self.publisher.publish(msg)
        self.get_logger().info(f"Depth:{self.rov_depth}")

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