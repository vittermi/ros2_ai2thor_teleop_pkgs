import sys
import termios
import tty
import select
import time
import os

import rclpy
from rclpy.node import Node
from ai2thor_msgs.msg import TeleopCmd


KEY_BINDINGS = {
    'w': ('MOVE_AHEAD',     'step_m'),
    's': ('MOVE_BACK',      'step_m'),
    'a': ('STRAFE_LEFT',    'step_m'),
    'd': ('STRAFE_RIGHT',   'step_m'),
    'q': ('ROTATE_LEFT',    'step_deg'),
    'e': ('ROTATE_RIGHT',   'step_deg'),
    'r': ('LOOK_UP',        'step_deg'),
    'f': ('LOOK_DOWN',      'step_deg'),
    'x': ('STOP',           None),
}

HELP = """
AI2-THOR Teleop — commands:

 Movement:       w/s = forward/back     a/d = strafe left/right
 Rotation:       q/e = rotate left/right
 Look:           r/f = look up/down
 Stop:           x
 Quit:           CTRL + C or k

Params (ROS): topic, step_m (m), step_deg (deg), repeat_hz
"""

def getch_blocking():
    """
    Block until a single keypress is read from stdin (like turtlesim keyLoop).
    Returns a single-character string.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    try:
        return c.decode('utf-8')
    except UnicodeDecodeError:
        return ''
    


class TeleopKey(Node):
    def __init__(self):
        super().__init__('ai2thor_teleop_key')

        self.declare_parameter('topic', '/ai2thor/teleop_cmd')
        self.declare_parameter('step_m', 0.25)
        self.declare_parameter('step_deg', 10.0)
        self.declare_parameter('repeat_hz', 10.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.step_m = float(self.get_parameter('step_m').get_parameter_value().double_value)
        self.step_deg = float(self.get_parameter('step_deg').get_parameter_value().double_value)
        self.repeat_hz = float(self.get_parameter('repeat_hz').get_parameter_value().double_value)

        self.publisher = self.create_publisher(TeleopCmd, self.topic, 10)

        self.last_key = None
        self.last_sent_time = 0.0
        self.repeat_period = 1.0 / max(self.repeat_hz, 0.1)

        self.get_logger().info(HELP)
        self.get_logger().info(
            f'Publishing TeleopCmd on "{self.topic}"  step_m={self.step_m}  step_deg={self.step_deg}  repeat_hz={self.repeat_hz}'
        )

        # self.timer = self.create_timer(self.repeat_period, self._repeat_tick)

    # this does not 
    def _publish_action(self, action, magnitude):
        msg = TeleopCmd()
        msg.action = action
        msg.magnitude = float(magnitude) if magnitude is not None else 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.action)
        self.last_sent_time = time.time()

    def key_loop(self):
        self.get_logger().info("Reading from keyboard. Press 'k' or Ctrl-C to quit.")
        try:
            while rclpy.ok():
                key = getch_blocking().lower()  # your existing function
                if key == 'k' or key == '\x03':  # 'k' or Ctrl-C byte decoded
                    break
                if key in KEY_BINDINGS:
                    action, mag_param = KEY_BINDINGS[key]
                    magnitude = getattr(self, mag_param) if mag_param else 0.0
                    self._publish_action(action, magnitude)
        finally:
            # nothing else to do here; terminal is restored by getch_blocking()
            pass


    def on_shutdown(self):
        try:
            self._publish_action('STOP', 0.0)
        except Exception:
            pass


def main():
    rclpy.init()
    node = TeleopKey()
    try:
        node.key_loop()  # <-- run the blocking loop here
    except KeyboardInterrupt:
        # optional: no logging here; see step 4
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
