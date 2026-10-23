import math

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8
from std_srvs.srv import Empty

from robot_base_driver.driver_node import (
    LOW_BATTERY_V, ODOM_STALE_S, PATH_MAX_POSES, STATUS_STALE_S, RobotBaseDriver)


class Recorder:
    def __init__(self):
        self.calls = []

    def publish(self, msg):
        self.calls.append(msg)

    sendTransform = publish


@pytest.fixture
def node():
    n = RobotBaseDriver()
    n.cmd_vel_pub = Recorder()
    n.tf_broadcaster = Recorder()
    n.path_pub = Recorder()
    yield n
    n.destroy_node()


def test_odom_warns_before_first_message(node):
    assert node._odom_status(now=0.0).level == DiagnosticStatus.WARN


def test_fresh_odom_is_ok_and_does_not_stop_robot(node):
    node.on_odom(Odometry())
    assert node._odom_status(node.last_odom_time + 0.1).level == DiagnosticStatus.OK
    assert node.cmd_vel_pub.calls == []


def test_stale_odom_stops_robot(node):
    node.on_odom(Odometry())
    status = node._odom_status(node.last_odom_time + ODOM_STALE_S + 0.5)
    assert status.level == DiagnosticStatus.ERROR
    assert node.cmd_vel_pub.calls == [Twist()]
    assert node.odom_watchdog_tripped


def test_odom_recovery_clears_watchdog(node):
    node.on_odom(Odometry())
    node._odom_status(node.last_odom_time + ODOM_STALE_S + 0.5)
    node.on_odom(Odometry())
    node._odom_status(node.last_odom_time)
    assert not node.odom_watchdog_tripped


def test_odom_becomes_tf(node):
    msg = Odometry()
    msg.pose.pose.position.x = 1.5
    msg.pose.pose.position.y = -0.25
    msg.pose.pose.orientation.z = 0.6
    msg.pose.pose.orientation.w = 0.8
    node.on_odom(msg)
    (tf,) = node.tf_broadcaster.calls
    assert (tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_link')
    assert tf.transform.translation.x == 1.5
    assert tf.transform.translation.y == -0.25
    assert tf.transform.rotation == msg.pose.pose.orientation


def test_connectivity(node):
    assert node._connectivity_status(now=0.0).level == DiagnosticStatus.ERROR
    node.on_status(UInt8(data=0))
    assert node._connectivity_status(node.last_status_time).level == DiagnosticStatus.OK
    assert node._connectivity_status(node.last_status_time + STATUS_STALE_S + 0.5).level == DiagnosticStatus.ERROR


def test_fault_flag(node):
    node.on_status(UInt8(data=0))
    assert node._fault_status().level == DiagnosticStatus.OK
    node.on_status(UInt8(data=1))
    assert node._fault_status().level == DiagnosticStatus.ERROR


@pytest.mark.parametrize('present, voltage, level', [
    (False, 7.5, DiagnosticStatus.WARN),
    (True, LOW_BATTERY_V - 0.1, DiagnosticStatus.WARN),
    (True, 7.5, DiagnosticStatus.OK),
])
def test_battery(node, present, voltage, level):
    node.on_battery(BatteryState(present=present, voltage=voltage))
    assert node._battery_status().level == level


def test_log_relay_accepts_mixed_levels(node):
    # rclpy Humble pins one severity per call site; mixed levels used to crash.
    for level in (10, 20, 30, 40, 50, 20, 99):
        node.on_log(Log(level=level, msg='hello'))


def odom_at(x, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def path_xs(node):
    return [round(p.pose.position.x, 3) for p in node.path.poses]


def test_path_skips_tiny_moves(node):
    for x in (0.0, 0.002, 0.004, 0.012, 0.015, 0.03):
        node.on_odom(odom_at(x))
    assert path_xs(node) == [0.0, 0.012, 0.03]


def test_path_records_turn_in_place(node):
    node.on_odom(odom_at(0.0, yaw=0.0))
    node.on_odom(odom_at(0.0, yaw=math.radians(1)))
    node.on_odom(odom_at(0.0, yaw=math.radians(10)))
    assert len(node.path.poses) == 2


def test_path_turn_across_pi_is_small(node):
    node.on_odom(odom_at(0.0, yaw=math.radians(179)))
    node.on_odom(odom_at(0.0, yaw=math.radians(-179)))
    assert len(node.path.poses) == 1


def test_path_restarts_after_odom_jump(node):
    for x in (1.00, 1.02, 1.04):
        node.on_odom(odom_at(x))
    node.on_odom(odom_at(0.0))  # ESP32 rebooted, odom back at the origin
    assert path_xs(node) == [0.0]


def test_path_is_capped(node):
    for i in range(PATH_MAX_POSES + 10):
        node.on_odom(odom_at(i * 0.02))
    assert len(node.path.poses) == PATH_MAX_POSES
    assert node.path.poses[0].pose.position.x == pytest.approx(10 * 0.02)


def test_path_published_only_when_changed(node):
    node.on_odom(odom_at(0.0))
    node.publish_path()
    node.publish_path()
    (path,) = node.path_pub.calls
    assert path.header.frame_id == 'odom'
    assert path.poses[0].header.frame_id == 'odom'


def test_reset_path_service(node):
    for x in (0.0, 0.05, 0.1):
        node.on_odom(odom_at(x))
    node.on_reset_path(Empty.Request(), Empty.Response())
    assert node.path.poses == []
    node.publish_path()
    assert node.path_pub.calls[-1].poses == []
