import math
import numpy as np
import pytest
from ros2_amr_nmpc_telemetry.simulation import DifferentialDrivePlant, run_trial
from ros2_amr_nmpc_telemetry.freshness import TimestampGate


def test_timestamp_replay_delay_and_future():
    gate = TimestampGate()
    assert not gate.accept(0, 1_000_000_000)
    assert gate.accept(1_000_000_000, 1_100_000_000)
    assert not gate.accept(1_000_000_000, 1_100_000_000)
    assert not gate.accept(900_000_000, 1_100_000_000)
    assert not gate.accept(1_100_000_000, 2_000_000_000)
    assert not gate.accept(3_000_000_000, 2_000_000_000)
    assert gate.accept(2_000_000_000, 2_000_000_000)


def test_watchdog_stops_base_without_controller():
    plant = DifferentialDrivePlant()
    plant.command(1, 0, 0)
    for k in range(100): plant.step(k*0.01, 0.01)
    assert plant.watchdog
    assert abs(plant.velocity[0]) < 0.002
    assert not plant.command(math.nan, 0, 2)


def test_slip_not_visible_to_encoder_but_visible_to_imu():
    plant = DifferentialDrivePlant()
    for k in range(150):
        t=k*0.01; plant.command(1,0.5,t); plant.step(t,0.01,traction=0.5)
    encoder, imu = plant.sensors()
    assert encoder[0] > 0.9
    assert plant.velocity[0] == pytest.approx(0.5, abs=0.01)
    assert imu == pytest.approx(0.25, abs=0.05)


def test_trial_reproducibility_and_limits():
    a, ma = run_trial(seed=23, duration=2)
    b, mb = run_trial(seed=23, duration=2)
    np.testing.assert_array_equal(a, b)
    assert ma == mb
    assert np.abs(a[:,9]).max() <= 1.5
    assert np.abs(a[:,10]).max() <= 2


@pytest.mark.parametrize('scenario', ['imu_dropout','odom_dropout','odom_delay','nonfinite'])
def test_sensor_faults_stop_and_recover(scenario):
    data, metrics = run_trial(scenario=scenario, seed=42, duration=18)
    assert 0.4 <= metrics['stop_detection_sec'] <= 1.05
    active = (data[:,0] > 13.1) & (data[:,0] < 16)
    assert np.all(data[active,9:11] == 0)
    assert np.all(data[active,13] == 1)
    assert np.any(np.abs(data[data[:,0] > 17,9]) > 0.01)


def test_dropped_commands_activate_independent_watchdog():
    data, metrics = run_trial(scenario='command_dropout', duration=18)
    assert metrics['stop_detection_sec'] <= 0.21
    assert np.all(data[(data[:,0] > 12.3) & (data[:,0] < 16),14] == 1)
    assert np.abs(data[(data[:,0] > 13.5) & (data[:,0] < 16),11]).max() < 0.002
