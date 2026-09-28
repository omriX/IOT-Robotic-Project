#!/usr/bin/env python3
# driver_node.py
#
# Host-side driver for robot_base: relays /robot_base/log to /rosout,
# broadcasts odom->base_link TF from /odom, publishes /diagnostics, and
# stops the robot if /odom goes stale.

import time

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8
from rcl_interfaces.msg import Log
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

ODOM_STALE_S = 1.0
STATUS_STALE_S = 2.0
LOW_BATTERY_V = 6.6


class RobotBaseDriver(Node):
    def __init__(self):
        super().__init__('robot_base_driver')

        self.last_odom_time = None
        self.last_status_time = None
        self.faulted = False
        self.battery_voltage = float('nan')
        self.battery_present = False
        self.odom_watchdog_tripped = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Log, '/robot_base/log', self.on_log, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
        self.create_subscription(UInt8, '/robot_base/status', self.on_status, 10)

        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info('robot_base_driver started')

    def on_log(self, msg):
        # rcl_interfaces/Log levels (10/20/30/40/50) match
        # rclpy.logging.LoggingSeverity's values directly.
        try:
            severity = LoggingSeverity(msg.level)
        except ValueError:
            severity = LoggingSeverity.INFO
        self.get_logger().log(f'[robot_base] {msg.msg}', severity)

    def on_odom(self, msg):
        self.last_odom_time = time.monotonic()

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def on_battery(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_present = msg.present

    def on_status(self, msg):
        self.last_status_time = time.monotonic()
        self.faulted = msg.data != 0

    def publish_diagnostics(self):
        now = time.monotonic()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(self._odom_status(now))
        array.status.append(self._connectivity_status(now))
        array.status.append(self._fault_status())
        array.status.append(self._battery_status())
        self.diag_pub.publish(array)

    def _odom_status(self, now):
        status = DiagnosticStatus(name='robot_base: odom')
        if self.last_odom_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'no odom received yet'
            return status

        age = now - self.last_odom_time
        if age <= ODOM_STALE_S:
            status.level = DiagnosticStatus.OK
            status.message = f'age {age:.2f}s'
            self.odom_watchdog_tripped = False
            return status

        status.level = DiagnosticStatus.ERROR
        status.message = f'odom stale ({age:.1f}s) -- publishing zero cmd_vel'
        self.cmd_vel_pub.publish(Twist())
        if not self.odom_watchdog_tripped:
            self.get_logger().error(f'odom stale ({age:.1f}s), stopping robot')
            self.odom_watchdog_tripped = True
        return status

    def _connectivity_status(self, now):
        status = DiagnosticStatus(name='robot_base: agent connectivity')
        stale = self.last_status_time is None or (now - self.last_status_time) > STATUS_STALE_S
        status.level = DiagnosticStatus.ERROR if stale else DiagnosticStatus.OK
        status.message = 'no /robot_base/status received recently' if stale else 'connected'
        return status

    def _fault_status(self):
        status = DiagnosticStatus(name='robot_base: self-test')
        status.level = DiagnosticStatus.ERROR if self.faulted else DiagnosticStatus.OK
        status.message = 'faulted -- cmd_vel refused' if self.faulted else 'passed'
        return status

    def _battery_status(self):
        status = DiagnosticStatus(name='robot_base: battery')
        status.values.append(KeyValue(key='voltage', value=f'{self.battery_voltage:.2f}'))
        if not self.battery_present:
            status.level = DiagnosticStatus.WARN
            status.message = 'not present'
        elif self.battery_voltage < LOW_BATTERY_V:
            status.level = DiagnosticStatus.WARN
            status.message = 'low battery'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'ok'
        return status


def main():
    rclpy.init()
    node = RobotBaseDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
