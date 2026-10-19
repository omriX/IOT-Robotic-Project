#!/usr/bin/env python3
# drive_test.py
#
# Drives full test courses using /odom feedback: a closed square, a
# forward/back + 90 deg + forward/back pattern, a half circle, and a
# half circle that drives straight back to the start.
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
DIST_TOLERANCE_M = 0.01
ANGLE_TOLERANCE_RAD = math.radians(1.0)
MIN_LINEAR_MPS = 0.04
MIN_ANGULAR_RPS = 0.5
SLOWDOWN_M = 0.15
SLOWDOWN_RAD = math.radians(30)
HEADING_HOLD_GAIN = 2.0
HEADING_HOLD_MAX_RPS = 0.4
LINE_HOLD_GAIN = 3.0
RADIUS_HOLD_GAIN = 8.0
MAX_CORRECTIONS = 3


class SegmentError(RuntimeError):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DriveTest(Node):
    def __init__(self, lead_s):
        super().__init__('drive_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lead_s = lead_s
        self.odom = None
        self.odom_time = 0.0
        self.odom_seq = 0
        self.heading_goal = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()
        self.odom_seq += 1

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

    def reset_heading_goal(self):
        self.heading_goal = self.pose()[2]

    def _spin_checked(self, deadline):
        rclpy.spin_once(self, timeout_sec=1.0 / CMD_RATE_HZ)
        now = time.monotonic()
        if now - self.odom_time > ODOM_STALE_S:
            raise SegmentError('/odom went stale mid-segment')
        if now > deadline:
            raise SegmentError('segment timed out -- check the wheels and motors')

    def _run(self, error_fn, command_fn, tolerance, max_speed, min_speed, slowdown, expected_s):
        # The robot keeps moving for a while after a stop is sent (Wi-Fi latency, wheel
        # deceleration, odom period), so the main pass stops when the error is predicted
        # to reach zero lead_s from now.
        error = error_fn()
        sign = math.copysign(1.0, error)
        deadline = time.monotonic() + expected_s * 2.0 + 3.0
        last_seq, last_t, last_err, rate = self.odom_seq, time.monotonic(), error, 0.0
        while sign * error - max(rate, 0.0) * self.lead_s > tolerance:
            speed = max(min_speed, min(max_speed, max_speed * sign * error / slowdown))
            command_fn(sign * speed)
            self._spin_checked(deadline)
            error = error_fn()
            if self.odom_seq != last_seq:
                now = time.monotonic()
                if now > last_t:
                    rate = (last_err - error) / (now - last_t) * sign
                last_seq, last_t, last_err = self.odom_seq, now, error
        self.settle()

        # Feedback from a standstill overshoots by the same latency, so corrections are
        # timed pulses instead: with delay plus a smooth speed-up and slow-down, a pulse
        # moves the robot about speed * duration.
        for _ in range(MAX_CORRECTIONS):
            error = error_fn()
            if abs(error) <= tolerance:
                return
            end = time.monotonic() + abs(error) / min_speed
            deadline = end + 3.0
            while time.monotonic() < end:
                command_fn(math.copysign(min_speed, error))
                self._spin_checked(deadline)
            self.settle()

    def drive_distance(self, distance, speed):
        x0, y0, _ = self.pose()
        h = self.heading_goal

        def error():
            x, y, _ = self.pose()
            return distance - ((x - x0) * math.cos(h) + (y - y0) * math.sin(h))

        def command(v):
            x, y, theta = self.pose()
            left_of_line = -(x - x0) * math.sin(h) + (y - y0) * math.cos(h)
            hold = (HEADING_HOLD_GAIN * wrap(h - theta)
                    - LINE_HOLD_GAIN * left_of_line * math.copysign(1.0, v))
            self.send(v, max(-HEADING_HOLD_MAX_RPS, min(HEADING_HOLD_MAX_RPS, hold)))

        self._run(error, command, DIST_TOLERANCE_M, speed, MIN_LINEAR_MPS, SLOWDOWN_M,
                  abs(distance) / speed)

    def rotate(self, angle, angular_speed):
        # Aim at an absolute heading so small per-turn errors don't add up over a course.
        self.heading_goal = wrap(self.heading_goal + angle)
        goal = self.heading_goal
        self._run(lambda: wrap(goal - self.pose()[2]), lambda w: self.send(0.0, w),
                  ANGLE_TOLERANCE_RAD, angular_speed, MIN_ANGULAR_RPS, SLOWDOWN_RAD,
                  abs(angle) / angular_speed)

    def arc(self, radius, sweep, speed):
        x0, y0, th0 = self.pose()
        turn = math.copysign(1.0, sweep)
        cx = x0 - turn * radius * math.sin(th0)
        cy = y0 + turn * radius * math.cos(th0)
        state = {'prev': th0, 'turned': 0.0}

        def error():
            theta = self.pose()[2]
            state['turned'] += wrap(theta - state['prev'])
            state['prev'] = theta
            return sweep - state['turned']

        def command(w):
            # Steer back onto the circle when a weaker wheel widens or tightens the arc.
            x, y, _ = self.pose()
            outside = math.hypot(x - cx, y - cy) - radius
            v = w * radius
            self.send(v, w + turn * RADIUS_HOLD_GAIN * outside * abs(v) / radius)

        # Error is in radians here, so the speed limits are angular rates along the arc.
        self._run(error, command, ANGLE_TOLERANCE_RAD,
                  speed / radius, MIN_LINEAR_MPS / radius, SLOWDOWN_RAD,
                  radius * abs(sweep) / speed)
        self.heading_goal = wrap(self.heading_goal + sweep)


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


def half_circle_return(node, args):
    print(f'  half circle, radius {args.radius:.2f} m')
    node.arc(args.radius, math.pi, args.speed)
    print(f'  turn 90 deg, forward {2 * args.radius:.2f} m back to start, turn 90 deg')
    node.rotate(math.radians(90), args.angular_speed)
    node.drive_distance(2 * args.radius, args.speed)
    node.rotate(math.radians(90), args.angular_speed)
    return (math.pi + 2) * args.radius


TESTS = {
    'square': (square, 'square path, should finish on the start spot'),
    'out_back': (out_and_back, 'forward/back, turn 90 deg, forward/back'),
    'half_circle': (half_circle, 'half circle, should finish 2*radius to the left, facing back'),
    'half_circle_return': (half_circle_return,
                           'half circle, then straight back, should finish on the start spot'),
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
    parser.add_argument('--radius', type=float, default=0.5,
                        help='half circle radius in m, both half circle tests (default 0.5)')
    parser.add_argument('--speed', type=float, default=0.1, help='linear speed in m/s (default 0.1)')
    parser.add_argument('--angular-speed', type=float, default=0.8,
                        help='in-place turn speed in rad/s (default 0.8)')
    parser.add_argument('--lead', type=float, default=0.4,
                        help='seconds the robot keeps moving after a stop is sent (default 0.4)')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest(args.lead)
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
