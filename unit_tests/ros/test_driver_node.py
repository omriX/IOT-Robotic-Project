import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8

from robot_base_driver.driver_node import LOW_BATTERY_V, ODOM_STALE_S, STATUS_STALE_S, RobotBaseDriver


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
