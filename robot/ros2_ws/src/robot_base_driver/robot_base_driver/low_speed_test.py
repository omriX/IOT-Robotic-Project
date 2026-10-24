#!/usr/bin/env python3
# low_speed_test.py
#
# Low-speed starts from standstill: sends a cmd_vel step, records the /odom
# twist, and reports how quickly and how smoothly the robot starts moving.
# Used to compare friction feedforward / startup kick settings
# (ff_static_pwm, ff_pwm_per_tick, kick_pwm, kick_ms) with live ros2 param set.

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_RATE_HZ = 20.0
ODOM_STALE_S = 1.0
SETTLE_S = 1.5
STALL_FRACTION = 0.1

DEFAULT_LINEAR = [0.02, 0.03, 0.04, 0.06]
DEFAULT_ANGULAR = [0.3, 0.5, 0.8]


def step_metrics(samples, target, cmd_start):
    """samples: [(t, value)] taken after cmd_start. Returns start-up metrics, or None."""
    if not samples or target == 0:
        return None
    sign = math.copysign(1.0, target)
    size = abs(target)
    values = [(t - cmd_start, sign * v) for t, v in samples]
    rise = next((t for t, v in values if v >= 0.5 * size), None)
    peak = max(v for _, v in values)
    moving = [v for t, v in values if rise is not None and t >= rise]
    stalls = sum(1 for v in moving if v < STALL_FRACTION * size)
    mean = sum(moving) / len(moving) if moving else 0.0
    return {
        'rise_s': rise,
        'peak_ratio': peak / size,
        'mean_ratio': mean / size,
        'stalls': stalls,
        'samples': len(moving),
    }


class LowSpeedTest(Node):
    def __init__(self):
        super().__init__('low_speed_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_time = 0.0
        self.samples = None
        self.angular = False
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()
        if self.samples is not None:
            twist = msg.twist.twist
            self.samples.append((self.odom_time, twist.angular.z if self.angular else twist.linear.x))

    def wait_for_odom(self, timeout_s=5.0):
        end = time.monotonic() + timeout_s
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError('no /odom received -- is the agent connected?')

    def send(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def settle(self):
        self.cmd_pub.publish(Twist())
        end = time.monotonic() + SETTLE_S
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_step(self, target, angular, duration_s):
        self.angular = angular
        self.samples = []
        start = time.monotonic()
        while time.monotonic() - start < duration_s:
            if angular:
                self.send(0.0, target)
            else:
                self.send(target, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / CMD_RATE_HZ)
            if time.monotonic() - self.odom_time > ODOM_STALE_S:
                raise RuntimeError('/odom went stale mid-step')
        samples, self.samples = self.samples, None
        self.settle()
        return step_metrics(samples, target, start)


def print_row(label, m):
    if m is None:
        print(f'{label:>14}  no samples')
        return
    rise = 'never' if m['rise_s'] is None else f"{m['rise_s'] * 1000:.0f} ms"
    print(f"{label:>14}  rise {rise:>8}  peak {m['peak_ratio']:4.2f}x  "
          f"mean {m['mean_ratio']:4.2f}x  stalls {m['stalls']}/{m['samples']}")


def main():
    parser = argparse.ArgumentParser(description='Low-speed start-from-rest test using /odom.')
    parser.add_argument('--linear', type=float, nargs='*', default=DEFAULT_LINEAR,
                        help='linear speeds in m/s (default 0.02 0.03 0.04 0.06)')
    parser.add_argument('--angular', type=float, nargs='*', default=DEFAULT_ANGULAR,
                        help='in-place turn speeds in rad/s (default 0.3 0.5 0.8)')
    parser.add_argument('--duration', type=float, default=2.0, help='seconds per step (default 2.0)')
    parser.add_argument('--alternate', action='store_true',
                        help='alternate the direction of each step so the robot stays in place')
    args = parser.parse_args()

    rclpy.init()
    node = LowSpeedTest()
    try:
        node.wait_for_odom()
        print('*** clear floor space, wheels on the ground, Ctrl-C stops the robot at any time ***')
        if input('ready? [y/N] ').strip().lower() not in ('y', 'yes'):
            return
        node.settle()
        sign = 1.0
        for angular, speeds, unit in ((False, args.linear, 'm/s'), (True, args.angular, 'rad/s')):
            for speed in speeds:
                for _ in range(2 if args.alternate else 1):
                    target = sign * speed
                    print_row(f'{target:+.2f} {unit}', node.run_step(target, angular, args.duration))
                    if args.alternate:
                        sign = -sign
        print('rise: time to 50% of target. peak/mean: relative to target. '
              'stalls: samples under 10% of target after the robot started moving.')
    except KeyboardInterrupt:
        print('\nstopped.')
    except RuntimeError as e:
        print(f'\naborted: {e}')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
