#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import time

start_time = int(time.time())
class Batt(Node):
    def __init__(self):
        super().__init__("Battery")
        self.subscriber = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        )
        self.get_logger().info("Starting Battery subscriber node")

    def callback(self, msg):
        data = msg.voltage
        self.get_logger().info(f"Battery Voltage: {data}")
    def check_volatage(self, msg):
        ela_time = int(time.time()) - start_time
        if ela_time <= 0:
            start_time = int(time.time())
            data = msg.voltage
            if data < 10:
                self.get_logger().info(f"Battery fell below safety")
            else:
                self.get_logger().info(f"Battery is still good")

class Imu_time(Node):
    def __init__(self):
        super().__init__("IMU")
        self.subscriber = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        )
        self.get_logger().info("Starting IMU subscriber node")

    def callback(self, msg):
        data = msg.header.stamp.sec
        self.get_logger().info(f"IMU Time: {data}")
  


def main(args=None):
    rclpy.init(args=args)
    batt_node = Batt()
    imu_node = Imu_time()
    executor = MultiThreadedExecutor()
    executor.add_node(batt_node)
    executor.add_node(imu_node)
    

    try:
        executor.spin() 
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
