"""Unit tests for the ROS-independent slip-aware digital twin."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry.digital_twin import AMRDigitalTwin  # noqa: E402

DT = 0.02
BIAS = 0.005


def feed(twin, start, n, wheel_v, wheel_w, gyro_w):
    out = None
    for i in range(n):
        out = twin.update(start + i * DT, wheel_v, wheel_w, gyro_w)
    return out, start + n * DT


def test_straight_motion_integrates_pose_and_learns_bias():
    twin = AMRDigitalTwin()
    out, _ = feed(twin, 0.0, 100, 1.0, 0.0, BIAS)
    assert out['x'] == pytest.approx(99 * DT * 1.0, abs=1e-9)
    assert out['y'] == pytest.approx(0.0, abs=1e-9)
    assert out['yaw'] == pytest.approx(0.0, abs=1e-9)
    assert out['gyro_bias'] == pytest.approx(BIAS, abs=1e-9)
    assert out['slip'] is False and out['traction'] == 1.0


def test_slip_is_detected_and_traction_is_estimated():
    twin = AMRDigitalTwin()
    _, t = feed(twin, 0.0, 200, 0.8, 0.5, 0.5 + BIAS)
    out, t = feed(twin, t, 60, 0.8, 0.5, 0.65 * 0.5 + BIAS)
    assert out['slip'] is True
    assert out['slip_events'] == 1
    assert out['traction'] == pytest.approx(0.65, abs=0.01)
    assert out['slip_time'] > 0.5


def test_traction_returns_to_one_after_slip_ends():
    twin = AMRDigitalTwin()
    _, t = feed(twin, 0.0, 200, 0.8, 0.5, 0.5 + BIAS)
    _, t = feed(twin, t, 60, 0.8, 0.5, 0.65 * 0.5 + BIAS)
    out, _ = feed(twin, t, 100, 0.8, 0.5, 0.5 + BIAS)
    assert out['slip'] is False
    assert out['traction'] == 1.0
    assert out['slip_events'] == 1


def test_no_alarm_storm_on_nominal_noise():
    rng = np.random.default_rng(1)
    twin = AMRDigitalTwin()
    alarms = []
    for i in range(3000):
        wheel_w = 0.3 + rng.normal(0.0, 0.03)
        gyro_w = 0.3 + BIAS + rng.normal(0.0, 0.01)
        alarms.append(twin.update(i * DT, 0.5, wheel_w, gyro_w)['slip'])
    assert np.mean(alarms) < 0.01


def test_non_finite_samples_are_ignored():
    twin = AMRDigitalTwin()
    good, t = feed(twin, 0.0, 150, 1.0, 0.0, BIAS)
    for bad in ((float('nan'), 0.0, BIAS), (1.0, float('inf'), BIAS), (1.0, 0.0, float('nan'))):
        out = twin.update(t, *bad)
        assert out['valid'] is False
        assert out['x'] == good['x'] and out['yaw'] == good['yaw']
    assert twin.update(float('nan'), 1.0, 0.0, BIAS)['valid'] is False


def test_large_time_gap_is_not_integrated():
    twin = AMRDigitalTwin()
    good, t = feed(twin, 0.0, 150, 1.0, 0.0, BIAS)
    out = twin.update(t + 5.0, 1.0, 0.0, BIAS)
    assert out['x'] == pytest.approx(good['x'], abs=1e-12)


def test_reset_forgets_everything():
    twin = AMRDigitalTwin()
    _, t = feed(twin, 0.0, 200, 0.8, 0.5, 0.5 + BIAS)
    feed(twin, t, 60, 0.8, 0.5, 0.65 * 0.5 + BIAS)
    twin.reset()
    out = twin.telemetry(0.0)
    assert (out['x'], out['y'], out['yaw']) == (0.0, 0.0, 0.0)
    assert out['slip'] is False and out['slip_events'] == 0 and out['traction'] == 1.0


@pytest.mark.parametrize('kwargs', [
    {'window': 0},
    {'window': 2.5},
    {'encoder_std': 0.0},
    {'encoder_std': float('nan')},
    {'s_min': 0.0},
    {'s_min': 1.5},
])
def test_invalid_parameters_raise(kwargs):
    with pytest.raises(ValueError):
        AMRDigitalTwin(**kwargs)

@pytest.mark.parametrize("sample_count", [1, 150, 260])
@pytest.mark.parametrize("timestamp_offset", [0.0, -0.10])
def test_replayed_timestamps_preserve_state_and_next_update(
        sample_count, timestamp_offset):
    import copy

    def state(twin):
        twin_state = {
            key: value for key, value in twin.__dict__.items()
            if key != "detector"
        }
        return copy.deepcopy((twin_state, twin.detector.__dict__))

    reference = AMRDigitalTwin()
    for i in range(sample_count):
        gyro_w = 0.5 + BIAS if i < 200 else 0.65 * 0.5 + BIAS
        reference.update(i * DT, 0.8, 0.5, gyro_w)

    if sample_count == 260:
        assert reference.slip is True

    candidate = copy.deepcopy(reference)
    before = state(candidate)
    rejected = candidate.update(
        candidate._t_prev + timestamp_offset,
        3.0, -2.0, 2.0,
    )

    assert rejected["valid"] is False
    assert state(candidate) == before

    next_time = sample_count * DT
    gyro_w = 0.5 + BIAS if sample_count < 200 else 0.65 * 0.5 + BIAS
    expected_output = reference.update(next_time, 0.8, 0.5, gyro_w)
    actual_output = candidate.update(next_time, 0.8, 0.5, gyro_w)

    assert actual_output == expected_output
    assert state(candidate) == state(reference)
