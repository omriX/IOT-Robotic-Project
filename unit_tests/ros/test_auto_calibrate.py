import math

import pytest

import argparse

from robot_base_driver.auto_calibrate import (
    add_tune_args, angle_delta, approach_speed, corrected_wheel_base, corrected_wheel_radius,
    lateral_offset, next_stop_margin, progress_along_heading, run_tune, yaw_from_quaternion)


def test_radius_matches_recorded_calibration():
    # 17 Oct 2026: odom said 1.0419 m, tape said 1.04 m (docs/calibration_log_1.md).
    # The log rounds the odom distance, hence the 2e-6 tolerance.
    assert corrected_wheel_radius(0.022, 1.0419, 1.04) == pytest.approx(0.021961, abs=2e-6)


def test_radius_grows_when_odom_under_reports():
    assert corrected_wheel_radius(0.022, 0.9, 1.0) > 0.022


def test_base_shrinks_when_odom_under_rotates():
    # Odom saw less rotation than really happened -> wheel base too large.
    assert corrected_wheel_base(0.116, math.radians(80), math.radians(90)) < 0.116


def test_base_unchanged_when_exact():
    assert corrected_wheel_base(0.116, 1.2, 1.2) == pytest.approx(0.116)


@pytest.mark.parametrize('yaw', [0.0, 0.5, -1.0, math.pi / 2, -math.pi / 2, 3.0])
def test_yaw_round_trip(yaw):
    assert yaw_from_quaternion(math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


@pytest.mark.parametrize('theta0, theta1, expected', [
    (0.0, math.pi / 2, math.pi / 2),
    (0.0, -math.pi / 2, -math.pi / 2),
    # Crossing +-pi: a +90 deg turn starting at 135 deg ends at -135 deg.
    (math.radians(135), math.radians(-135), math.radians(90)),
    (math.radians(-135), math.radians(135), math.radians(-90)),
])
def test_angle_delta_wraps(theta0, theta1, expected):
    assert angle_delta(theta0, theta1) == pytest.approx(expected)


def test_progress_along_rotated_heading():
    # Facing +y; 0.3 m forward and 0.01 m to the right (-x).
    theta0 = math.pi / 2
    assert progress_along_heading(1.0, 2.0, theta0, 0.99, 2.3) == pytest.approx(0.3)
    assert lateral_offset(1.0, 2.0, theta0, 0.99, 2.3) == pytest.approx(0.01)


def test_progress_is_negative_backward():
    assert progress_along_heading(0.0, 0.0, 0.0, -0.2, 0.0) == pytest.approx(-0.2)


@pytest.mark.parametrize('remaining, expected', [(0.5, 0.1), (0.1, 0.1), (0.05, 0.05), (0.001, 0.03)])
def test_approach_speed_ramps_to_floor(remaining, expected):
    assert approach_speed(remaining, cruise=0.1, slow_zone=0.1, min_fraction=0.3) == pytest.approx(expected)


def test_stop_margin_clamped():
    assert next_stop_margin(0.01, 0.02, max_margin=0.25) == pytest.approx(0.03)
    assert next_stop_margin(0.01, -0.05, max_margin=0.25) == 0.0
    assert next_stop_margin(0.2, 0.1, max_margin=0.25) == 0.25


class CoastingRobot:
    """Lands a fixed coast past wherever it was told to stop."""

    def __init__(self, coast):
        self.coast = coast
        self.margins = []

    def closed_loop_move(self, angular, target, cruise, stop_margin, slow_zone, min_fraction):
        self.margins.append(stop_margin)
        return abs(target) - stop_margin + self.coast, 0.0, 0.0


def tune_args(*argv):
    parser = argparse.ArgumentParser()
    add_tune_args(parser, angular=False)
    return parser.parse_args(['--pause', '0', *argv])


def test_tune_learns_the_coast(capsys):
    robot = CoastingRobot(coast=0.02)
    run_tune(robot, tune_args(), angular=False)
    # Run 1 overshoots by the coast; run 2 stops early by it and lands; run 3 confirms.
    assert robot.margins == pytest.approx([0.0, 0.02, 0.02])
    assert 'converged' in capsys.readouterr().out


def test_tune_gives_up_after_max_runs(capsys):
    class Noisy(CoastingRobot):
        def closed_loop_move(self, *a):
            self.coast = -self.coast  # flips between +-2 cm every run
            return super().closed_loop_move(*a)

    robot = Noisy(coast=0.02)
    run_tune(robot, tune_args('--max-runs', '4'), angular=False)
    assert len(robot.margins) == 4
    assert 'did not converge' in capsys.readouterr().out
