"""Unit tests for the ROS-independent yaw-rate slip detector (loaded by file path)."""
import importlib.util
import os

import numpy as np
import pytest

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'ros2_amr_nmpc_telemetry', 'slip_detector.py')
_SPEC = importlib.util.spec_from_file_location('slip_detector_under_test', _PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
YawRateSlipDetector = _MODULE.YawRateSlipDetector

# Baseline sample: wheel 0.0, gyro 0.005 -> residual -0.005 (gyro bias 0.005).
# Slip sample: wheel 0.2, gyro 0.1 -> residual +0.1.


def feed(det, wheel, gyro, n):
    return [det.update(wheel, gyro) for _ in range(n)]


def test_constant_baseline_never_alarms():
    det = YawRateSlipDetector()
    assert not any(feed(det, 0.0, 0.005, 600))


def test_step_is_detected_quickly():
    det = YawRateSlipDetector()
    feed(det, 0.0, 0.005, 200)
    hits = feed(det, 0.2, 0.1, 30)
    assert hits[-1] is True
    assert hits.index(True) <= 6


def test_alarm_clears_after_signal_returns():
    det = YawRateSlipDetector()
    feed(det, 0.0, 0.005, 200)
    feed(det, 0.2, 0.1, 30)
    back = feed(det, 0.0, 0.005, 30)
    assert back[-1] is False


def test_center_is_frozen_during_alarm():
    det = YawRateSlipDetector()
    feed(det, 0.0, 0.005, 200)
    feed(det, 0.2, 0.1, 100)
    assert abs(det.center - (-0.005)) < 0.002


def test_non_finite_samples_are_ignored():
    det = YawRateSlipDetector()
    feed(det, 0.0, 0.005, 150)
    before = det.samples
    state = det.alarm
    assert det.update(float('nan'), 0.0) == state
    assert det.update(0.0, float('inf')) == state
    assert det.samples == before


def test_confirm_samples_delays_alarm():
    fast = YawRateSlipDetector(confirm_samples=1)
    slow = YawRateSlipDetector(confirm_samples=10)
    for det in (fast, slow):
        feed(det, 0.0, 0.005, 200)
    fast_hits = feed(fast, 0.2, 0.1, 40)
    slow_hits = feed(slow, 0.2, 0.1, 40)
    assert fast_hits.index(True) < slow_hits.index(True)
    assert 10 <= slow_hits.index(True) <= 16


def test_reset_forgets_history():
    det = YawRateSlipDetector()
    feed(det, 0.0, 0.005, 150)
    feed(det, 0.2, 0.1, 30)
    det.reset()
    assert det.samples == 0
    assert det.alarm is False
    assert det.center == 0.0


def test_no_alarm_storm_on_nominal_noise():
    rng = np.random.default_rng(0)
    det = YawRateSlipDetector()
    alarms = []
    for _ in range(3000):
        wheel = rng.normal(0.0, 0.03)
        gyro = 0.005 + rng.normal(0.0, 0.01)
        alarms.append(det.update(wheel, gyro))
    assert np.mean(alarms) < 0.03


@pytest.mark.parametrize('kwargs', [
    {'window': 0},
    {'window': 2.5},
    {'threshold': -1.0},
    {'threshold': float('nan')},
    {'warmup_samples': 5},
    {'center_alpha': 0.0},
    {'center_alpha': 1.5},
    {'confirm_samples': 0},
])
def test_invalid_parameters_raise(kwargs):
    with pytest.raises(ValueError):
        YawRateSlipDetector(**kwargs)
