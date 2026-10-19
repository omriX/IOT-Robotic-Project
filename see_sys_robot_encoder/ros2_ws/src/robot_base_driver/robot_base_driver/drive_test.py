#!/usr/bin/env python3
# drive_test.py
#
# Drives full test courses using /odom feedback: a closed square, a
# forward/back + 90 deg + forward/back pattern, and a half circle.
# Each segment stops when odom says it is done, so the calibrated
# wheel_radius_m / wheel_base_m decide how far the robot really goes.

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_RATE_HZ = 20.0
ODOM_STALE_S = 1.0
SETTLE_S = 0.8
DIST_TOLERANCE_M = 0.005
ANGLE_TOLERANCE_RAD = math.radians(0.5)
MIN_LINEAR_MPS = 0.04
MIN_ANGULAR_RPS = 0.35
SLOWDOWN_M = 0.10
SLOWDOWN_RAD = math.radians(20)


class SegmentError(RuntimeError):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_time = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()

    def wait_for_odom(self, timeout_s=5.0):
        end = time.monotonic() + timeout_s
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise SegmentError('no /odom received -- is the agent connected?')

    def pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, 2.0 * math.atan2(q.z, q.w)

    def send(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def settle(self):
        self.stop()
        end = time.monotonic() + SETTLE_S
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _step(self, deadline):
        rclpy.spin_once(self, timeout_sec=1.0 / CMD_RATE_HZ)
        now = time.monotonic()
        if now - self.odom_time > ODOM_STALE_S:
            raise SegmentError('/odom went stale mid-segment')
        if now > deadline:
            raise SegmentError('segment timed out -- check the wheels and motors')

    def drive_distance(self, distance, speed):
        x0, y0, _ = self.pose()
        sign = math.copysign(1.0, distance)
        deadline = time.monotonic() + abs(distance) / speed * 2.0 + 3.0
        while True:
            x, y, _ = self.pose()
            remaining = abs(distance) - math.hypot(x - x0, y - y0)
            if remaining <= DIST_TOLERANCE_M:
                break
            v = max(MIN_LINEAR_MPS, speed * min(1.0, remaining / SLOWDOWN_M))
            self.send(sign * v, 0.0)
            self._step(deadline)
        self.settle()

    def reset_heading_goal(self):
        self.heading_goal = self.pose()[2]

    def rotate(self, angle, angular_speed):
        # Aim at an absolute heading so small per-turn errors don't add up over a course.
        self.heading_goal = wrap(self.heading_goal + angle)
        sign = math.copysign(1.0, angle)
        deadline = time.monotonic() + abs(angle) / angular_speed * 2.0 + 3.0
        while True:
            _, _, theta = self.pose()
            remaining = sign * wrap(self.heading_goal - theta)
            if remaining <= ANGLE_TOLERANCE_RAD:
                break
            w = max(MIN_ANGULAR_RPS, angular_speed * min(1.0, remaining / SLOWDOWN_RAD))
            self.send(0.0, sign * w)
            self._step(deadline)
        self.settle()

    def arc(self, radius, sweep, speed):
        _, _, prev = self.pose()
        turned = 0.0
        sign = math.copysign(1.0, sweep)
        deadline = time.monotonic() + radius * abs(sweep) / speed * 2.0 + 3.0
        while True:
            _, _, theta = self.pose()
            turned += wrap(theta - prev)
            prev = theta
            remaining = abs(sweep) - sign * turned
            if remaining <= ANGLE_TOLERANCE_RAD:
                break
            v = max(MIN_LINEAR_MPS, speed * min(1.0, remaining / SLOWDOWN_RAD))
            self.send(v, sign * v / radius)
            self._step(deadline)
        self.settle()


def square(node, args):
    for side in range(4):
        print(f'  side {side + 1}/4: forward {args.side:.2f} m, then turn 90 deg')
        node.drive_distance(args.side, args.speed)
        node.rotate(math.radians(90), args.angular_speed)
    return 4 * args.side


def out_and_back(node, args):
    for leg in range(2):
        print(f'  leg {leg + 1}/2: forward {args.distance:.2f} m, back {args.distance:.2f} m')
        node.drive_distance(args.distance, args.speed)
        node.drive_distance(-args.distance, args.speed)
        if leg == 0:
            print('  turn 90 deg')
            node.rotate(math.radians(90), args.angular_speed)
    return 4 * args.distance


def half_circle(node, args):
    print(f'  half circle, radius {args.radius:.2f} m')
    node.arc(args.radius, math.pi, args.speed)
    return math.pi * args.radius


TESTS = {
    'square': (square, 'square path, should finish on the start spot'),
    'out_back': (out_and_back, 'forward/back, turn 90 deg, forward/back'),
    'half_circle': (half_circle, 'half circle, should finish 2*radius to the left, facing back'),
}


def confirm(prompt):
    return input(prompt).strip().lower() in ('y', 'yes')


def run_test(node, name, args):
    fn, description = TESTS[name]
    print(f'\n=== {name}: {description} ===')
    if not confirm('robot on its start mark? [y/N] '):
        return
    node.settle()
    node.reset_heading_goal()
    x0, y0, th0 = node.pose()
    path = fn(node, args)
    x1, y1, th1 = node.pose()
    dx, dy = x1 - x0, y1 - y0
    # Express the end point in the start pose's frame so it matches the floor marks.
    fwd = dx * math.cos(th0) + dy * math.sin(th0)
    left = -dx * math.sin(th0) + dy * math.cos(th0)
    print(f'done. path length {path:.2f} m')
    print(f'odom end vs start: forward {fwd:+.3f} m, left {left:+.3f} m, '
          f'heading {math.degrees(wrap(th1 - th0)):+.1f} deg')
    if name != 'half_circle':
        print(f'odom closing error: {math.hypot(dx, dy):.3f} m '
              f'({math.hypot(dx, dy) / path * 100:.1f}% of path)')
    print('measure the real end position against your tape marks and compare.')


def main():
    parser = argparse.ArgumentParser(description='Drive test courses using /odom feedback.')
    parser.add_argument('test', choices=list(TESTS) + ['all'])
    parser.add_argument('--side', type=float, default=1.0, help='square side in m (default 1.0)')
    parser.add_argument('--distance', type=float, default=1.0, help='out_back leg in m (default 1.0)')
    parser.add_argument('--radius', type=float, default=0.5, help='half circle radius in m (default 0.5)')
    parser.add_argument('--speed', type=float, default=0.1, help='linear speed in m/s (default 0.1)')
    parser.add_argument('--angular-speed', type=float, default=0.6,
                        help='in-place turn speed in rad/s (default 0.6)')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest()
    try:
        node.wait_for_odom()
        print('*** clear floor space, wheels on the ground, Ctrl-C stops the robot at any time ***')
        for name in (TESTS if args.test == 'all' else [args.test]):
            run_test(node, name, args)
    except KeyboardInterrupt:
        print('\nstopped.')
    except SegmentError as e:
        print(f'\naborted: {e}')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
