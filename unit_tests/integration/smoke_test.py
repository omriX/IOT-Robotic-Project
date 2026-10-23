#!/usr/bin/env python3
# smoke_test.py -- end-to-end check of a running robot through the micro-ROS agent.
#
# Needs the agent running and the robot connected (state:AGENT_CONNECTED). Run
# inside the ros_tools container:
#   python3 /repo/unit_tests/integration/smoke_test.py           # robot can stay still
#   python3 /repo/unit_tests/integration/smoke_test.py --drive   # also drives ~14 cm forward
#
# Prints PASS / WARN / FAIL per check; exits 1 if anything failed.

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8

TARGET_NODE = 'robot_base'
PARAMS = ['log_level', 'cmd_vel_timeout_ms', 'odom_rate_hz', 'wheel_radius_m',
          'wheel_base_m', 'max_linear_mps', 'max_angular_rps']
LOW_BATTERY_V = 6.6
MIN_ODOM_HZ = 15.0
DRIVE_SPEED_MPS = 0.07
DRIVE_TIME_S = 2.0


class Smoke(Node):
    def __init__(self):
        super().__init__('smoke_test')
        self.odom = []
        self.battery = None
        self.status = None
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.append((time.monotonic(), m)), 50)
        self.create_subscription(BatteryState, '/battery_state', lambda m: setattr(self, 'battery', m), 10)
        self.create_subscription(UInt8, '/robot_base/status', lambda m: setattr(self, 'status', m), 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.params = self.create_client(GetParameters, f'/{TARGET_NODE}/get_parameters')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout_s):
        end = time.monotonic() + timeout_s
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()


results = []


def report(level, name, detail):
    results.append(level)
    print(f'[{level}] {name}: {detail}')


def check_odom(node):
    if not node.wait_for(lambda: node.odom, 5.0):
        report('FAIL', '/odom', 'nothing received in 5 s -- is the agent running and the robot connected?')
        return False
    node.odom.clear()
    node.spin_for(3.0)
    hz = len(node.odom) / 3.0
    report('PASS' if hz >= MIN_ODOM_HZ else 'WARN', '/odom rate', f'{hz:.1f} Hz (expect about 20)')
    return True


def check_battery(node):
    if not node.wait_for(lambda: node.battery is not None, 3.0):
        report('FAIL', '/battery_state', 'nothing received in 3 s')
        return
    v = node.battery.voltage
    report('PASS' if v >= LOW_BATTERY_V else 'WARN', '/battery_state',
           f'{v:.2f} V' + ('' if v >= LOW_BATTERY_V else f' -- below {LOW_BATTERY_V} V, charge it'))


def check_status(node):
    if not node.wait_for(lambda: node.status is not None, 3.0):
        report('FAIL', '/robot_base/status', 'nothing received in 3 s')
        return
    if node.status.data == 0:
        report('PASS', 'self-test', 'passed at boot')
    else:
        report('FAIL', 'self-test', 'FAILED at boot, robot refuses /cmd_vel -- reset it, see WebSerial for deltas')


def check_params(node):
    if not node.params.wait_for_service(timeout_sec=5.0):
        report('FAIL', 'parameters', f'/{TARGET_NODE}/get_parameters not available')
        return
    future = node.params.call_async(GetParameters.Request(names=PARAMS))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        report('FAIL', 'parameters', 'get_parameters timed out')
        return
    values = []
    for name, v in zip(PARAMS, future.result().values):
        values.append(f'{name}={v.integer_value if v.type == 2 else round(v.double_value, 6)}')
    report('PASS', 'parameters', ', '.join(values))


def pose(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)


def check_drive(node):
    if input(f'Drive forward {DRIVE_SPEED_MPS * DRIVE_TIME_S * 100:.0f} cm? Clear floor ahead. [y/N] ').strip().lower() != 'y':
        report('WARN', 'drive', 'skipped by operator')
        return
    node.spin_for(0.3)
    x0, y0, th0 = pose(node.odom[-1][1])
    twist = Twist()
    twist.linear.x = DRIVE_SPEED_MPS
    try:
        end = time.monotonic() + DRIVE_TIME_S
        while time.monotonic() < end:
            node.cmd_pub.publish(twist)
            node.spin_for(0.1)
    finally:
        node.cmd_pub.publish(Twist())
    node.spin_for(1.5)

    x1, y1, th1 = pose(node.odom[-1][1])
    expected = DRIVE_SPEED_MPS * DRIVE_TIME_S
    dist = math.hypot(x1 - x0, y1 - y0)
    turn = math.degrees((th1 - th0 + math.pi) % (2 * math.pi) - math.pi)
    speed = abs(node.odom[-1][1].twist.twist.linear.x)

    ok_dist = 0.5 * expected <= dist <= 1.5 * expected
    report('PASS' if ok_dist else 'FAIL', 'drive distance', f'{dist * 100:.1f} cm (commanded {expected * 100:.0f} cm)')
    report('PASS' if abs(turn) <= 10 else 'WARN', 'drive heading', f'{turn:+.1f} deg drift')
    report('PASS' if speed < 0.01 else 'FAIL', 'stop after drive', f'{speed:.3f} m/s 1.5 s after the last command')


def main():
    parser = argparse.ArgumentParser(description='Robot end-to-end smoke test.')
    parser.add_argument('--drive', action='store_true', help='also drive a short distance forward')
    args = parser.parse_args()

    rclpy.init()
    node = Smoke()
    try:
        if check_odom(node):
            check_battery(node)
            check_status(node)
            check_params(node)
            if args.drive:
                check_drive(node)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

    failed = results.count('FAIL')
    print(f'\n{results.count("PASS")} passed, {results.count("WARN")} warnings, {failed} failed')
    return 1 if failed or not results else 0


if __name__ == '__main__':
    sys.exit(main())
