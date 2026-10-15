"""Low-level vehicle driver node.

Subscribes to ``ackermann_msgs/AckermannDriveStamped`` on ``/ackermann_cmd``
and drives a steering servo (PWM ch0 / GPIO12) and an ESC throttle
(PWM ch1 / GPIO13) through the Raspberry Pi hardware PWM peripheral.

Design notes
------------
* Actuator-native interface: this node only converts a *steering angle* and a
  *speed* into pulse widths. Velocity-space (v, w) commands are intentionally
  NOT handled here -- add a separate ``twist_to_ackermann`` node upstream if
  needed. This keeps the safety-critical bottom layer simple and free of the
  v->0 steering singularity.
* Safety watchdog: if no command arrives within ``watchdog_timeout_s`` the
  outputs are forced to neutral (throttle stop + steering centre).
* ESC arming: on startup neutral is held for ``arm_duration_s``.
"""

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from rc_car_driver.pwm_actuator import PwmActuator


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class VehicleDriver(Node):

    def __init__(self):
        super().__init__('vehicle_driver')

        # --- common PWM params ---
        self.declare_parameter('pwm_frequency', 50.0)
        self.declare_parameter('pwm_chip', 0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.declare_parameter('arm_duration_s', 2.0)

        # --- steering params (PWM channel 0 / GPIO12) ---
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('steering_neutral_us', 1587.0)  # straight ahead (chassis-calibrated)
        self.declare_parameter('steering_min_us', 1075.0)      # right full lock
        self.declare_parameter('steering_max_us', 1947.0)      # left full lock
        self.declare_parameter('max_steering_angle_deg', 25.0)
        self.declare_parameter('steering_direction', 1)  # +1 / -1 to invert

        # --- throttle params (PWM channel 1 / GPIO13) ---
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('throttle_neutral_us', 1500.0)
        self.declare_parameter('throttle_min_us', 1000.0)
        self.declare_parameter('throttle_max_us', 2000.0)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('throttle_direction', -1)  # this ESC (TBLE-04S) drives forward on low-pulse side

        g = self.get_parameter
        freq = g('pwm_frequency').value
        chip = g('pwm_chip').value

        self.steering_neutral_us = g('steering_neutral_us').value
        self.steering_min_us = g('steering_min_us').value
        self.steering_max_us = g('steering_max_us').value
        # parameter is in degrees for convenience; /ackermann_cmd is radians.
        self.max_steering_angle_rad = math.radians(g('max_steering_angle_deg').value)
        self.steering_direction = 1 if g('steering_direction').value >= 0 else -1

        self.throttle_neutral_us = g('throttle_neutral_us').value
        self.throttle_min_us = g('throttle_min_us').value
        self.throttle_max_us = g('throttle_max_us').value
        self.max_speed_mps = g('max_speed_mps').value
        self.throttle_direction = 1 if g('throttle_direction').value >= 0 else -1

        self.watchdog_timeout_s = g('watchdog_timeout_s').value

        # --- actuators ---
        self.steering = PwmActuator(
            channel=g('steering_channel').value, frequency_hz=freq,
            min_us=self.steering_min_us, max_us=self.steering_max_us, chip=chip)
        self.throttle = PwmActuator(
            channel=g('throttle_channel').value, frequency_hz=freq,
            min_us=self.throttle_min_us, max_us=self.throttle_max_us, chip=chip)

        # latest command (defaults = stopped & centred)
        self._steering_angle = 0.0
        self._speed = 0.0
        # initialise "last command" far in the past so the watchdog holds
        # neutral until a real command arrives.
        self._last_cmd_time = self.get_clock().now() - rclpy.duration.Duration(
            seconds=2.0 * self.watchdog_timeout_s + 1.0)

        # --- ESC arming: hold neutral on both channels ---
        arm_s = g('arm_duration_s').value
        self.steering.set_pulse_us(self.steering_neutral_us)
        self.throttle.set_pulse_us(self.throttle_neutral_us)
        self.get_logger().info(
            f'Arming: holding neutral for {arm_s:.1f}s '
            f'(steer={self.steering_neutral_us:.0f}us, '
            f'throttle={self.throttle_neutral_us:.0f}us)')
        if arm_s > 0.0:
            self._sleep(arm_s)

        # --- ROS I/O ---
        self.sub = self.create_subscription(
            AckermannDriveStamped, 'ackermann_cmd', self._on_cmd, 10)
        period = 1.0 / float(g('control_rate_hz').value)
        self.timer = self.create_timer(period, self._on_timer)
        self._warned_watchdog = False
        self.get_logger().info('vehicle_driver ready, listening on /ackermann_cmd')

    def _sleep(self, seconds):
        # blocking sleep that still keeps rclpy time advancing
        end = self.get_clock().now() + rclpy.duration.Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _on_cmd(self, msg):
        self._steering_angle = float(msg.drive.steering_angle)
        self._speed = float(msg.drive.speed)
        self._last_cmd_time = self.get_clock().now()

    def _map(self, value, value_max, neutral_us, min_us, max_us, direction):
        """Map a signed command to a pulse width around neutral, clamped."""
        if value_max <= 0.0:
            return neutral_us
        norm = _clamp(value / value_max, -1.0, 1.0) * direction
        if norm >= 0.0:
            return neutral_us + norm * (max_us - neutral_us)
        return neutral_us + norm * (neutral_us - min_us)

    def _on_timer(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.watchdog_timeout_s:
            # Stale command -> fail safe to neutral.
            self.steering.set_pulse_us(self.steering_neutral_us)
            self.throttle.set_pulse_us(self.throttle_neutral_us)
            if not self._warned_watchdog:
                self.get_logger().warn(
                    f'Watchdog: no command for >{self.watchdog_timeout_s:.2f}s, '
                    'holding neutral.')
                self._warned_watchdog = True
            return
        self._warned_watchdog = False

        steer_us = self._map(
            self._steering_angle, self.max_steering_angle_rad,
            self.steering_neutral_us, self.steering_min_us,
            self.steering_max_us, self.steering_direction)
        throttle_us = self._map(
            self._speed, self.max_speed_mps,
            self.throttle_neutral_us, self.throttle_min_us,
            self.throttle_max_us, self.throttle_direction)
        self.steering.set_pulse_us(steer_us)
        self.throttle.set_pulse_us(throttle_us)

    def shutdown(self):
        """Return to neutral, then release the PWM outputs."""
        try:
            self.steering.set_pulse_us(self.steering_neutral_us)
            self.throttle.set_pulse_us(self.throttle_neutral_us)
        except Exception:  # noqa: BLE001 - best effort on shutdown
            pass
        for act in (self.steering, self.throttle):
            try:
                act.stop()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
