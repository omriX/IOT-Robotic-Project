#!/usr/bin/env python3
# auto_calibrate.py
#
# Automates the "command it, read odom" half of docs/live_calibration_log.md
# Parts 1 and 2: drives a straight line or an in-place rotation, reads the
# odom-reported distance/angle, and computes the corrected wheel_radius_m /
# wheel_base_m. You still have to physically measure the actual distance or
# angle with a tape measure -- there's no ground-truth sensor to automate
# that part away.

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


class AutoCalibrate(Node):
    def __init__(self):
        super().__init__('auto_calibrate')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.get_param_client = self.create_client(GetParameters, f'/{TARGET_NODE}/get_parameters')
        self.set_param_client = self.create_client(SetParameters, f'/{TARGET_NODE}/set_parameters')

    def _on_odom(self, msg):
        self.odom = msg

    def wait_for_odom(self, timeout_s=5.0):
        end = time.monotonic() + timeout_s
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError('no /odom received -- is the agent connected?')

    def pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        return p.position.x, p.position.y, theta

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
    new_radius = current_radius * (actual / reported)
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
    reported = theta1 - theta0
    print(f'odom-reported rotation: {math.degrees(reported):.2f} deg ({reported:.4f} rad)')
    if reported == 0:
        print('reported rotation is 0 -- check /odom, aborting.')
        return

    print('measure the actual angle turned against your tape lines (a protractor,')
    actual_deg = read_float('or the angle between your two marks): ')
    actual = math.radians(actual_deg)
    new_base = current_base * (reported / actual)
    print(f'current wheel_base_m = {current_base:.6f}')
    print(f'new wheel_base_m     = {new_base:.6f}')

    if confirm('apply live via ros2 param set? [y/N] '):
        node.set_param('wheel_base_m', new_base)
        print('applied. run again to confirm reported now tracks actual.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)

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
        else:
            run_rotate(node, args)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
        print('\nstopped.')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
