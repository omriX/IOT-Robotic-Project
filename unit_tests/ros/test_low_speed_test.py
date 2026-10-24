import pytest

from robot_base_driver.low_speed_test import step_metrics


def ramp(values, start=10.0, dt=0.05):
    return [(start + (i + 1) * dt, v) for i, v in enumerate(values)]


def test_smooth_start():
    m = step_metrics(ramp([0.0, 0.03, 0.04, 0.04, 0.04]), 0.04, 10.0)
    assert m['rise_s'] == pytest.approx(0.10)
    assert m['peak_ratio'] == pytest.approx(1.0)
    assert m['stalls'] == 0


def test_stick_slip_counts_stalls_and_overshoot():
    m = step_metrics(ramp([0.0, 0.0, 0.09, 0.0, 0.0, 0.08, 0.0]), 0.04, 10.0)
    assert m['peak_ratio'] == pytest.approx(2.25)
    assert m['stalls'] == 3


def test_reverse_target_uses_its_sign():
    m = step_metrics(ramp([0.0, -0.5, -0.5]), -0.5, 10.0)
    assert m['rise_s'] == pytest.approx(0.10)
    assert m['mean_ratio'] == pytest.approx(1.0)


def test_never_moves():
    m = step_metrics(ramp([0.0, 0.0, 0.0]), 0.03, 10.0)
    assert m['rise_s'] is None
    assert m['samples'] == 0


def test_no_samples():
    assert step_metrics([], 0.03, 10.0) is None
