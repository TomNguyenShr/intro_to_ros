#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, FluidPressure
import sensor_msgs.msg
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import sys
class BlueRov2Sensors(Node):
    def __init__(self):
        """initializes and subscribes to battery, IMU, and pressure topics"""
        super().__init__("bluerov2sensors")
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.battery = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.callbackBattery,
            qos_profile
        )
        self.get_logger().info("starting battery sensor node")
        self.get_logger().info(sensor_msgs.msg.__file__)


        self.IMU = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.callbackImu,
            qos_profile
        )
        self.get_logger().info("starting IMU data node")


        self.staticpressure = self.create_subscription(
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.callbackStaticPressure,
            qos_profile
        )
        self.get_logger().info("starting static pressure node")

        self.battery_timer = self.create_timer(5.0, self.voltagecheck)
        #creates timer to check voltage every 5 seconds

    def callbackBattery(self, msg):
        """Callback function for the battery. Saves and updates battery voltage to battery"""
        self.battery = msg
        self.get_logger().info(f"\nBattery Voltage: {self.battery.voltage}")

    def callbackImu(self, msg):
        """Callback function for the IMU. Saves and updates IMU data to Imu"""
        self.Imu = msg
        self.get_logger().info(f"\nImu linear acceleration: {self.Imu.linear_acceleration}")

    def callbackStaticPressure(self,msg):
        """Callback function for static pressure. Saves and updates static pressure in Pa and depth in meters"""
        self.staticpressure = msg
        self.get_logger().info(f"\nStatic pressure: {msg.fluid_pressure}")
        # Depth is calculated by: (pressure in Pa - atmospheric pressure in Pa) / gravity in m/s^2 / density of water in kg/m^3 
        self.get_logger().info(f"\nDepth:{(msg.fluid_pressure-101325)/9.81/1000}")
    
    def voltagecheck(self):
        """Checks if battery voltage is safe"""
        if self.battery.voltage < 12:
            self.get_logger().info("battery voltage is too low")
        else:
            self.get_logger().info("battery voltage is safe")


def main(args=None):
    rclpy.init(args=args)
    node = BlueRov2Sensors()
   # print(node.battery.header.stamp)

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