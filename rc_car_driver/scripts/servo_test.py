#!/usr/bin/env python3
"""Phase-0 standalone servo test (no ROS required).

Drives one hardware-PWM channel to verify wiring / overlay / library / channel
before involving ROS. Run AFTER the system setup in install.md and AFTER wiring
the servo (signal -> GPIO12, common GND, V+ from an external 5-6V supply).

Examples
--------
    # smooth back-and-forth sweep on channel 0 (GPIO12), Ctrl-C to stop
    python3 servo_test.py --sweep

    # send a single position then hold (e.g. one extreme), 1700us
    python3 servo_test.py --once --pulse 1700

    # use channel 1 (GPIO13) with custom range
    python3 servo_test.py --channel 1 --min 1100 --max 1900 --sweep
"""

import argparse
import os
import sys
import time

# Allow running straight from source (before `colcon build`/sourcing).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rc_car_driver.pwm_actuator import PwmActuator  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description='Standalone hardware-PWM servo test.')
    p.add_argument('--channel', type=int, default=0,
                   help='PWM channel (0=GPIO12, 1=GPIO13). Default 0.')
    p.add_argument('--chip', type=int, default=0,
                   help='sysfs pwmchip number (Pi4=0, Pi5=2). Default 0.')
    p.add_argument('--freq', type=float, default=50.0, help='PWM Hz. Default 50.')
    p.add_argument('--min', dest='min_us', type=float, default=1000.0,
                   help='Min pulse width us. Default 1000.')
    p.add_argument('--max', dest='max_us', type=float, default=2000.0,
                   help='Max pulse width us. Default 2000.')
    p.add_argument('--neutral', type=float, default=1500.0,
                   help='Neutral pulse width us. Default 1500.')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--sweep', action='store_true',
                      help='Continuously sweep min<->max (default mode).')
    mode.add_argument('--once', action='store_true',
                      help='Go to --pulse once and hold until Ctrl-C.')
    p.add_argument('--pulse', type=float, default=1500.0,
                   help='Target pulse us for --once. Default 1500 (neutral).')
    p.add_argument('--period', type=float, default=3.0,
                   help='Seconds for one min->max sweep. Default 3.0.')
    return p.parse_args()


def sweep(act, min_us, max_us, neutral_us, period_s):
    """Smoothly move neutral -> max -> min -> neutral, repeating."""
    step_hz = 50.0
    dt = 1.0 / step_hz
    half = max(0.1, period_s / 2.0)
    waypoints = [neutral_us, max_us, min_us, neutral_us]
    print('Sweeping (Ctrl-C to stop and return to neutral)...')
    while True:
        for a, b in zip(waypoints, waypoints[1:]):
            n = max(1, int(half / dt))
            for i in range(n + 1):
                us = a + (b - a) * (i / n)
                act.set_pulse_us(us)
                print(f'  pulse = {us:7.1f} us', end='\r', flush=True)
                time.sleep(dt)


def main():
    args = parse_args()
    act = PwmActuator(channel=args.channel, frequency_hz=args.freq,
                      min_us=args.min_us, max_us=args.max_us, chip=args.chip)
    print(f'channel={args.channel} (GPIO{12 if args.channel == 0 else 13}) '
          f'chip={args.chip} freq={args.freq}Hz '
          f'range=[{args.min_us:.0f},{args.max_us:.0f}]us')
    try:
        if args.once:
            us = act.set_pulse_us(args.pulse)
            print(f'Holding {us:.1f} us. Ctrl-C to stop.')
            while True:
                time.sleep(0.5)
        else:
            sweep(act, args.min_us, args.max_us, args.neutral, args.period)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        # Return to neutral briefly, then release.
        try:
            act.set_pulse_us(args.neutral)
            time.sleep(0.3)
        finally:
            act.stop()
            print('Stopped (output disabled).')


if __name__ == '__main__':
    main()
