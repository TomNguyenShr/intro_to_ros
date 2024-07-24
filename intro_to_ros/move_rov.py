#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class BlueROV2Control(Node):
    """
    ROS2 Jazzy node class for publishing RC Override commands to a BlueROV2 via MAVROS.
    """

    def __init__(self):
        """
        Initialize the current instance of the BlueROV2Control
        """
        super().__init__("bluerov2_rc_override")
        self.control_msg = OverrideRCIn()
        self.control_pub = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            10
        )

    def set_rc_channel(self, channel, pwm):
        """
        Publish a pwm value to a specific channel.
        """
        self.control_msg.channels = [OverrideRCIn.CHAN_NOCHANGE for _ in range(18)]
        self.control_msg.channels[channel - 1] = max(1100, min(pwm, 1900))
        self.control_pub.publish(self.control_msg)

    def set_rc_channels(self, channels, pwms):
        """
        Publish multiple pwm values to specific channels.
        """

        self.control_msg.channels = [OverrideRCIn.CHAN_NOCHANGE for _ in range(18)]
        for channel, pwm in zip(channels, pwms):
            self.control_msg.channels[channel - 1] = max(1100, min(pwm, 1900))
        self.control_pub.publish(self.control_msg)

    def set_rc_channels_to_neutral(self):
        """
        Publish the neutral pwm value to all channels.
        """

        self.control_msg.channels = [1500 for _ in range(18)]
        self.control_pub.publish(self.control_msg)

    def run_node(self, node):
        rclpy.spin(node)
        """
        Sample method to implement control logic.
        """
def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2Control()
    try:
        node.run_node(node)
   
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()