"""Thin wrapper around the Raspberry Pi hardware PWM peripheral.

Hides the sysfs / duty-cycle arithmetic behind a pulse-width (microsecond)
interface, which is the natural unit for hobby RC servos and ESCs
(typically 1000-2000 us pulses at 50 Hz, 1500 us = neutral).

Backed by ``rpi-hardware-pwm`` (true SoC hardware PWM via /sys/class/pwm).
On a Raspberry Pi 4 the relevant sysfs chip is ``pwmchip0`` (chip=0);
on a Pi 5 it is ``pwmchip2`` (chip=2) -- pass ``chip`` accordingly.
"""

from rpi_hardware_pwm import HardwarePWM


class PwmActuator:
    """A single hardware-PWM channel driving a servo or ESC.

    Parameters
    ----------
    channel:
        Hardware PWM channel index. With ``dtoverlay=pwm-2chan,pin=12,...,pin2=13``
        channel 0 -> GPIO12 and channel 1 -> GPIO13.
    frequency_hz:
        PWM carrier frequency. RC servos/ESCs expect ~50 Hz.
    min_us, max_us:
        Hard safety clamp for the output pulse width (microseconds). Commands
        outside this range are clamped, never sent raw to the hardware.
    chip:
        sysfs pwmchip number (Pi4=0, Pi5=2).
    """

    def __init__(self, channel, frequency_hz=50.0, min_us=1000.0,
                 max_us=2000.0, chip=0):
        if min_us > max_us:
            raise ValueError('min_us must be <= max_us')
        self.channel = channel
        self.frequency_hz = float(frequency_hz)
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self._period_us = 1_000_000.0 / self.frequency_hz
        self._pwm = HardwarePWM(pwm_channel=channel, hz=self.frequency_hz,
                                chip=chip)
        # Start with no output (0% duty) so nothing twitches before the first
        # explicit command / arming sequence.
        self._pwm.start(0.0)
        self._last_us = 0.0

    def set_pulse_us(self, pulse_us):
        """Set the output pulse width in microseconds (clamped to [min,max])."""
        pulse_us = max(self.min_us, min(self.max_us, float(pulse_us)))
        duty_percent = pulse_us / self._period_us * 100.0
        self._pwm.change_duty_cycle(duty_percent)
        self._last_us = pulse_us
        return pulse_us

    @property
    def last_us(self):
        """The last pulse width actually commanded (after clamping)."""
        return self._last_us

    def stop(self):
        """Stop emitting pulses (servo/ESC goes limp). Safe to call twice."""
        try:
            self._pwm.change_duty_cycle(0.0)
        finally:
            self._pwm.stop()
