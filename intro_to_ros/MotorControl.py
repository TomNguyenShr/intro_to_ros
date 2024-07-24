#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class RCOverridePublisher(Node):
    def __init__(self, pwm, channel):
        super().__init__("rc_override_publisher")
        qos = qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            qos
        )
        self.end_time2 = time.time()+60
        self.pwm = pwm
        self.channel = channel
        self.end_time1 = time.time()+100
        self.timer = self.create_timer(1.0, self.control_actuator)
        self.get_logger().info("RC Override Publisher Node started")

    def control_actuator(self):
        msg = OverrideRCIn()
        msg.channels = [65535 for _ in range(18)]
        msg.channels[self.channel] = self.pwm
       
        # Set RC channel values; adjust these as needed
        self.publisher.publish(msg)
        self.get_logger().info(f"{msg} for  {time.time()}")
        
def main(args=None):
    rclpy.init(args=args)
    node = RCOverridePublisher(1800, 2)
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
