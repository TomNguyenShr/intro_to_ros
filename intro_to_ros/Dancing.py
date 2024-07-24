import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn


class DanceNode(Node):
    _move_forward = False
    _move_back = False
    _move_left = False
    _move_right = False
    _move_up = False
    _move_down = False
    _turn_left = False
    _turn_right = False
    _stop = False
    _step_counter = 0.0
    _run_counter = 0
    _light = False

    def __init__(self):
        super().__init__("dancing_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.loop = self.create_timer(0.1, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 8
        self.command_pub.publish(neutral)

    def _loop(self):
        if self._step_counter > 60:
            self.destroy_node()
            return
        self._dance_moves()

        # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 11
        if self._light:
            commands.channels[8] = 1900
            commands.channels[9] = 1900
        else:
            commands.channels[8] = 1000
            commands.channels[9] = 1000
        if self._move_forward:
            commands.channels[4] = 1700
        elif self._move_back:
            commands.channels[4] = 1300
        elif not self._move_forward and not self._move_back:
            commands.channels[4] = 1500
        if self._move_left:
            commands.channels[5] = 1300
        elif self._move_right:
            commands.channels[5] = 1700
        elif not self._move_left and not self._move_right:
            commands.channels[5] = 1500
        if self._move_up:
            commands.channels[2] = 1700
        elif self._move_down:
            commands.channels[2] = 1300
        elif not self._move_up and not self._move_down:
            commands.channels[2] = 1500
        if self._turn_left:
            commands.channels[3] = 1200
        elif self._turn_right:
            commands.channels[3] = 1800
        elif not self._turn_left and not self._turn_right:
            commands.channels[3] = 1500
        if self._stop:
            commands.channels[2] = 1500
            commands.channels[3] = 1500
            commands.channels[4] = 1500
            commands.channels[5] = 1500


        self.command_pub.publish(commands)

    def _dance_moves(self):
        self.get_logger().info(f"{self._step_counter}")
        if self._step_counter < 60: #1.1
            # FORWARD 1.1 sec
            self._move_down = True
        elif self._step_counter < 2.2:
            # RIGHT 1.1 sec
            self._move_forward = False
            self._move_right = True
        elif self._step_counter < 3.3:
            # BACK 1.1 sec
            self._move_right = False
            self._move_back = True
        elif self._step_counter < 4.4:
            # LEFT 1.1 sec
            self._move_back = False
            self._move_left = True
        elif self._step_counter < 4.5:
            # RIGHT TO STOP DRIFT 0.1 sec
            self._move_left = False
            self._move_right = True
        elif self._step_counter < 5.6:
            # MOVE FORWARD 1 sec
            self._move_right = False
            self._move_forward = True
        elif self._step_counter < 6.7:
            #MOVE BACK 1.1 sec
            self._move_forward = False
            self._move_back = True
        elif self._step_counter < 8.9:
            # SPIN RIGHT 2.2 sec
            self._light = True
            self._turn_right = True

        if self._step_counter > 60: #8.9
            self._turn_right = False
            self._step_counter = 0
            self._run_counter += 1
            self._light = False
        
        if self._run_counter == 2:
            self.move_forward = False
            self.stop = True
            return None

        self._step_counter += 0.1

    def destroy_node(self):
        
        return super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    danceNode = DanceNode()

    try:
        rclpy.spin(danceNode)
    except KeyboardInterrupt:
        pass
    finally:
        danceNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()