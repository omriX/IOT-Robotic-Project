#!/usr/bin/env python3
# straight / rotate: correct wheel_radius_m / wheel_base_m from odom vs. a tape-measured value.
# drive / turn: odom-terminated moves; tune-drive / tune-turn learn the stop margin.
# See docs/live_calibration.md.

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

TARGET_NODE = 'robot_base'
CMD_RATE_HZ = 20.0
ODOM_STALE_S = 0.5
# Robot counts as stopped once odom twist stays below these for SETTLE_HOLD_S.
SETTLE_LINEAR_MPS = 0.005
SETTLE_ANGULAR_RPS = 0.02
SETTLE_HOLD_S = 0.3
SETTLE_TIMEOUT_S = 3.0


def yaw_from_quaternion(z, w):
    """Yaw of a planar (z/w-only) quaternion, as the firmware publishes it."""
    return 2.0 * math.atan2(z, w)


def angle_delta(theta0, theta1):
    """Signed rotation from theta0 to theta1, wrapped to [-pi, pi)."""
    return (theta1 - theta0 + math.pi) % (2.0 * math.pi) - math.pi


def corrected_wheel_radius(current_radius, reported_m, actual_m):
    # Odom distance scales linearly with the wheel radius.
    return current_radius * (actual_m / reported_m)


def corrected_wheel_base(current_base, reported_rad, actual_rad):
    # Odom rotation scales with 1 / wheel base, so the ratio is inverted.
    return current_base * (reported_rad / actual_rad)


def progress_along_heading(x0, y0, theta0, x, y):
    """Signed distance travelled along the starting heading (negative = backward)."""
    return (x - x0) * math.cos(theta0) + (y - y0) * math.sin(theta0)


def lateral_offset(x0, y0, theta0, x, y):
    """Signed sideways drift from the starting line (positive = left)."""
    return -(x - x0) * math.sin(theta0) + (y - y0) * math.cos(theta0)


def approach_speed(remaining, cruise, slow_zone, min_fraction):
    """Cruise until slow_zone before the stop point, then ramp down linearly to min_fraction * cruise."""
    if remaining >= slow_zone or slow_zone <= 0:
        return cruise
    return cruise * max(min_fraction, remaining / slow_zone)


def next_stop_margin(margin, overshoot, max_margin):
    # Overshoot is how far past the target the robot coasted; stop that much earlier next time.
    return min(max(margin + overshoot, 0.0), max_margin)


class AutoCalibrate(Node):
    def __init__(self):
        super().__init__('auto_calibrate')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_time = None
        # Odom yaw wraps at +-pi; this keeps counting so long turns measure correctly.
        self.unwrapped_yaw = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.get_param_client = self.create_client(GetParameters, f'/{TARGET_NODE}/get_parameters')
        self.set_param_client = self.create_client(SetParameters, f'/{TARGET_NODE}/set_parameters')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.z, q.w)
        if self.odom is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += angle_delta(self.last_yaw, yaw)
        self.last_yaw = yaw
        self.odom = msg
        self.odom_time = time.monotonic()

    def check_odom_fresh(self):
        age = time.monotonic() - self.odom_time
        if age > ODOM_STALE_S:
            self.cmd_pub.publish(Twist())
            raise RuntimeError(f'/odom stale for {age:.2f}s mid-move, stopped')

    def wait_for_odom(self, timeout_s=5.0):
        # Wait for a message newer than this call, so pose() isn't a stale queued sample.
        start = time.monotonic()
        end = start + timeout_s
        while (self.odom_time is None or self.odom_time < start) and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom_time is None or self.odom_time < start:
            raise RuntimeError('no /odom received -- is the agent connected?')

    def pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, yaw_from_quaternion(q.z, q.w)

    def get_param(self, name):
        if not self.get_param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'/{TARGET_NODE}/get_parameters service not available')
        future = self.get_param_client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError(f'get_parameters({name}) timed out')
        return future.result().values[0].double_value

    def set_param(self, name, value):
        if not self.set_param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'/{TARGET_NODE}/set_parameters service not available')
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        future = self.set_param_client.call_async(
            SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError(f'set_parameters({name}) timed out')
        result = future.result().results[0]
        if not result.successful:
            raise RuntimeError(f'set_parameters({name}) rejected: {result.reason}')

    def drive(self, linear_x, angular_z, duration_s):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        period = 1.0 / CMD_RATE_HZ
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)
        self.cmd_pub.publish(Twist())
        # let the PID/watchdog settle and one more odom sample arrive
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def settle(self):
        """Keep commanding zero until odom twist reads stopped. Returns False on timeout."""
        stop = Twist()
        still_since = None
        end = time.monotonic() + SETTLE_TIMEOUT_S
        while time.monotonic() < end:
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=1.0 / CMD_RATE_HZ)
            twist = self.odom.twist.twist
            still = abs(twist.linear.x) < SETTLE_LINEAR_MPS and abs(twist.angular.z) < SETTLE_ANGULAR_RPS
            if not still:
                still_since = None
            elif still_since is None:
                still_since = time.monotonic()
            elif time.monotonic() - still_since >= SETTLE_HOLD_S:
                return True
        return False

    def closed_loop_move(self, angular, target, cruise, stop_margin, slow_zone, min_fraction):
        """Drive (angular=False, target in m) or turn (angular=True, target in rad) until odom
        progress reaches target - stop_margin, then settle. target's sign picks the direction.
        Returns the settled progress and, for drives, the sideways drift and heading change."""
        self.wait_for_odom()
        x0, y0, theta0 = self.pose()
        yaw0 = self.unwrapped_yaw
        direction = math.copysign(1.0, target)
        stop_at = abs(target) - stop_margin

        def progress():
            if angular:
                return direction * (self.unwrapped_yaw - yaw0)
            x, y, _ = self.pose()
            return direction * progress_along_heading(x0, y0, theta0, x, y)

        timeout = time.monotonic() + 3.0 * abs(target) / cruise + 5.0
        twist = Twist()
        while (remaining := stop_at - progress()) > 0:
            if time.monotonic() > timeout:
                self.cmd_pub.publish(Twist())
                raise RuntimeError('move did not reach its target in time, stopped')
            self.check_odom_fresh()
            speed = direction * approach_speed(remaining, cruise, slow_zone, min_fraction)
            if angular:
                twist.angular.z = speed
            else:
                twist.linear.x = speed
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=1.0 / CMD_RATE_HZ)

        if not self.settle():
            print('warning: odom twist did not settle to zero; reading pose anyway')
        x, y, theta = self.pose()
        return progress(), lateral_offset(x0, y0, theta0, x, y), angle_delta(theta0, theta)


def confirm(prompt):
    return input(prompt).strip().lower() in ('y', 'yes')


def read_float(prompt):
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print('enter a number')


def run_straight(node, args):
    node.wait_for_odom()
    current_radius = node.get_param('wheel_radius_m')
    x0, y0, _ = node.pose()

    duration = args.distance / args.speed
    print(f'driving forward {args.distance:.3f} m at {args.speed:.3f} m/s (~{duration:.1f}s)...')
    node.drive(args.speed, 0.0, duration)

    x1, y1, _ = node.pose()
    reported = math.hypot(x1 - x0, y1 - y0)
    print(f'odom-reported distance: {reported:.4f} m')
    if reported == 0:
        print('reported distance is 0 -- check /odom, aborting.')
        return

    actual = read_float('measured actual distance (m), from tape: ')
    new_radius = corrected_wheel_radius(current_radius, reported, actual)
    print(f'current wheel_radius_m = {current_radius:.6f}')
    print(f'new wheel_radius_m     = {new_radius:.6f}')

    if confirm('apply live via ros2 param set? [y/N] '):
        node.set_param('wheel_radius_m', new_radius)
        print('applied. run again to confirm reported now tracks actual.')


def run_rotate(node, args):
    node.wait_for_odom()
    current_base = node.get_param('wheel_base_m')
    _, _, theta0 = node.pose()

    target = math.radians(args.angle)
    duration = abs(target) / args.angular_speed
    speed = math.copysign(args.angular_speed, target)
    print(f'rotating {args.angle:.1f} deg at {math.degrees(args.angular_speed):.1f} deg/s (~{duration:.1f}s)...')
    node.drive(0.0, speed, duration)

    _, _, theta1 = node.pose()
    reported = angle_delta(theta0, theta1)
    print(f'odom-reported rotation: {math.degrees(reported):.2f} deg ({reported:.4f} rad)')
    if reported == 0:
        print('reported rotation is 0 -- check /odom, aborting.')
        return

    print('measure the actual angle turned against your tape lines (a protractor,')
    actual_deg = read_float('or the angle between your two marks): ')
    actual = math.radians(actual_deg)
    new_base = corrected_wheel_base(current_base, reported, actual)
    print(f'current wheel_base_m = {current_base:.6f}')
    print(f'new wheel_base_m     = {new_base:.6f}')

    if confirm('apply live via ros2 param set? [y/N] '):
        node.set_param('wheel_base_m', new_base)
        print('applied. run again to confirm reported now tracks actual.')


def move_spec(args, angular):
    """(target, cruise, stop_margin, slow_zone) in m or rad, plus a formatter that
    prints m as mm and rad as deg (pass sign='+' for errors)."""
    if angular:
        return (math.radians(args.angle), args.angular_speed, math.radians(args.stop_margin),
                math.radians(args.slow_zone),
                lambda rad, sign='': f'{math.degrees(rad):{sign}.2f} deg')
    return (args.distance, args.speed, args.stop_margin, args.slow_zone,
            lambda m, sign='': f'{m * 1000:{sign}.1f} mm')


def print_move(target, progress, drift, heading, fmt, angular):
    line = f'odom: {fmt(progress)} (target {fmt(abs(target))}), error {fmt(progress - abs(target), "+")}'
    if not angular:
        line += f', drift {drift * 1000:+.1f} mm, heading {math.degrees(heading):+.2f} deg'
    print(line)


def run_move(node, args, angular):
    target, cruise, margin, slow_zone, fmt = move_spec(args, angular)
    progress, drift, heading = node.closed_loop_move(
        angular, target, cruise, margin, slow_zone, args.min_speed_fraction)
    print_move(target, progress, drift, heading, fmt, angular)


def run_tune(node, args, angular):
    target, cruise, margin, slow_zone, fmt = move_spec(args, angular)
    tolerance = math.radians(args.tolerance) if angular else args.tolerance
    max_margin = abs(target) / 2.0
    in_tolerance = 0
    for run in range(1, args.max_runs + 1):
        # Alternate direction so the robot ends up back near where it started.
        signed_target = target if run % 2 else -target
        print(f'run {run}: stop margin {fmt(margin)}, {"forward" if signed_target > 0 else "back"}')
        progress, drift, heading = node.closed_loop_move(
            angular, signed_target, cruise, margin, slow_zone, args.min_speed_fraction)
        print_move(target, progress, drift, heading, fmt, angular)

        overshoot = progress - abs(target)
        in_tolerance = in_tolerance + 1 if abs(overshoot) <= tolerance else 0
        if in_tolerance >= args.confirm_runs:
            print(f'converged: {in_tolerance} runs in a row within {fmt(tolerance)}')
            break
        margin = next_stop_margin(margin, overshoot, max_margin)
        time.sleep(args.pause)
    else:
        print(f'did not converge in {args.max_runs} runs')

    flag = math.degrees(margin) if angular else margin
    print(f'learned stop margin: {fmt(margin)} (reuse with --stop-margin {flag:.4f})')


def add_move_args(p, angular):
    if angular:
        p.add_argument('--angle', type=float, default=90.0, help='rotation in degrees, + = left (default 90)')
        p.add_argument('--angular-speed', type=float, default=0.5, help='cruise speed in rad/s (default 0.5)')
        p.add_argument('--stop-margin', type=float, default=0.0,
                       help='stop this many degrees early to allow for coasting (default 0)')
        p.add_argument('--slow-zone', type=float, default=20.0,
                       help='start slowing this many degrees before the stop point (default 20)')
    else:
        p.add_argument('--distance', type=float, default=0.5, help='distance in m, - = backward (default 0.5)')
        p.add_argument('--speed', type=float, default=0.1, help='cruise speed in m/s (default 0.1)')
        p.add_argument('--stop-margin', type=float, default=0.0,
                       help='stop this many m early to allow for coasting (default 0)')
        p.add_argument('--slow-zone', type=float, default=0.1,
                       help='start slowing this many m before the stop point (default 0.1)')
    p.add_argument('--min-speed-fraction', type=float, default=0.3,
                   help='slowest approach speed, as a fraction of cruise (default 0.3)')


def add_tune_args(p, angular):
    add_move_args(p, angular)
    p.add_argument('--tolerance', type=float, default=1.0 if angular else 0.005,
                   help='accepted |error|, deg or m (default 1 deg / 0.005 m)')
    p.add_argument('--max-runs', type=int, default=10, help='give up after this many moves (default 10)')
    p.add_argument('--confirm-runs', type=int, default=2,
                   help='consecutive in-tolerance runs needed to finish (default 2)')
    p.add_argument('--pause', type=float, default=1.0, help='seconds between runs (default 1)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)

    add_move_args(sub.add_parser('drive', help='drive straight until odom reaches the distance'), angular=False)
    add_move_args(sub.add_parser('turn', help='turn in place until odom reaches the angle'), angular=True)
    add_tune_args(sub.add_parser('tune-drive', help='repeat drives, learning the stop margin'), angular=False)
    add_tune_args(sub.add_parser('tune-turn', help='repeat turns, learning the stop margin'), angular=True)

    p_straight = sub.add_parser('straight', help='wheel_radius_m via a straight-line drive')
    p_straight.add_argument('--distance', type=float, default=1.0, help='commanded distance in m (default 1.0)')
    p_straight.add_argument('--speed', type=float, default=0.1, help='commanded linear speed in m/s (default 0.1)')

    p_rotate = sub.add_parser('rotate', help='wheel_base_m via an in-place rotation')
    p_rotate.add_argument('--angle', type=float, default=90.0, help='commanded rotation in degrees (default 90)')
    p_rotate.add_argument('--angular-speed', type=float, default=0.5,
                           help='commanded angular speed in rad/s (default 0.5)')

    args = parser.parse_args()

    rclpy.init()
    node = AutoCalibrate()
    try:
        print('*** clear floor space, wheels on the ground, Ctrl-C stops the robot at any time ***')
        if not confirm('ready? [y/N] '):
            return
        if args.mode == 'straight':
            run_straight(node, args)
        elif args.mode == 'rotate':
            run_rotate(node, args)
        elif args.mode in ('drive', 'turn'):
            run_move(node, args, angular=args.mode == 'turn')
        else:
            run_tune(node, args, angular=args.mode == 'tune-turn')
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
        print('\nstopped.')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
