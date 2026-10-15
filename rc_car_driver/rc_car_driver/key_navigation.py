"""Keyboard teleop for testing: arrow keys -> /ackermann_cmd.

Publishes ``ackermann_msgs/AckermannDriveStamped`` so the same low-level
``vehicle_driver`` used for autonomy can be exercised by hand.

Controls
--------
    Up / Down     : speed      +/- (m/s)
    Left / Right  : steering   +/- (deg)   (left = positive, ROS convention)
    Space         : stop (speed=0, steering=0)
    q  /  Ctrl-C  : quit (sends a final neutral command)

Steering is entered and displayed in degrees for convenience; it is converted
to radians (ROS convention) when published on /ackermann_cmd.

No extra dependencies: reads stdin in cbreak mode via termios/tty and parses
the arrow-key escape sequences (ESC '[' 'A'/'B'/'C'/'D'). The terminal cannot
report key release, so this is increment-on-press; use Space to stop.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class KeyNavigation(Node):

    def __init__(self):
        super().__init__('key_navigation')
        self.declare_parameter('speed_step', 0.2)
        self.declare_parameter('steering_step_deg', 5.0)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('max_steering_angle_deg', 25.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.speed_step = self.get_parameter('speed_step').value
        self.steering_step = self.get_parameter('steering_step_deg').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steering_angle_deg').value

        self.speed = 0.0
        self.steering = 0.0

        self.pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self._print_help()

    def _print_help(self):
        sys.stdout.write(
            '\r\n'
            'key_navigation: arrows=drive  Space=stop  q=quit\r\n'
            '  Up/Down = speed, Left/Right = steering (left +)\r\n\r\n')
        sys.stdout.flush()

    def _read_keys(self):
        """Drain whatever is buffered on stdin and apply each key."""
        buf = b''
        while select.select([sys.stdin], [], [], 0.0)[0]:
            buf += os.read(sys.stdin.fileno(), 16)
            if len(buf) > 256:
                break
        i = 0
        quit_requested = False
        while i < len(buf):
            b = buf[i]
            if b == 0x1b and buf[i + 1:i + 2] == b'[':  # arrow escape seq
                code = buf[i + 2:i + 3]
                if code == b'A':      # Up
                    self.speed = _clamp(self.speed + self.speed_step,
                                        -self.max_speed, self.max_speed)
                elif code == b'B':    # Down
                    self.speed = _clamp(self.speed - self.speed_step,
                                        -self.max_speed, self.max_speed)
                elif code == b'D':    # Left
                    self.steering = _clamp(self.steering + self.steering_step,
                                           -self.max_steer, self.max_steer)
                elif code == b'C':    # Right
                    self.steering = _clamp(self.steering - self.steering_step,
                                           -self.max_steer, self.max_steer)
                i += 3
                continue
            ch = bytes([b])
            if ch == b' ':
                self.speed = 0.0
                self.steering = 0.0
            elif ch in (b'q', b'\x03'):  # q or Ctrl-C
                quit_requested = True
            i += 1
        return quit_requested

    def _tick(self):
        if self._read_keys():
            raise KeyboardInterrupt
        self._publish(self.speed, self.steering)
        sys.stdout.write(
            f'\r speed={self.speed:+.2f} m/s  steering={self.steering:+.1f} deg   ')
        sys.stdout.flush()

    def _publish(self, speed, steering_deg):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        # steering is tracked in degrees; /ackermann_cmd is radians.
        msg.drive.steering_angle = float(math.radians(steering_deg))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyNavigation()
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        # final neutral command so the car stops promptly
        try:
            node._publish(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.write('\r\nkey_navigation stopped.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
