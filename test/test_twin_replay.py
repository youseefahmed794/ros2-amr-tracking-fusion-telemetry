"""Check replay timestamps using the real short-duration simulator."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_replay():
    spec = importlib.util.spec_from_file_location(
        "amr_twin_replay_under_test",
        ROOT / "scripts" / "digital_twin_replay.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("scenario", ["nominal", "slip"])
def test_replay_compares_states_at_sensor_time(monkeypatch, scenario):
    replay = load_replay()
    monkeypatch.setattr(replay, "DURATION", 0.2)
    monkeypatch.setattr(replay, "FAULT", (0.06, 0.14))

    captured_truth = []
    captured_data = []
    original_sensors = replay.sim.DifferentialDrivePlant.sensors
    original_run = replay.sim.run_trial

    def observing_sensors(self):
        captured_truth.append(self.pose[:2].copy())
        return original_sensors(self)

    def observing_run(*args, **kwargs):
        data, metrics = original_run(*args, **kwargs)
        captured_data.append(data.copy())
        return data, metrics

    monkeypatch.setattr(
        replay.sim.DifferentialDrivePlant, "sensors", observing_sensors
    )
    monkeypatch.setattr(replay.sim, "run_trial", observing_run)
    rows, _ = replay.replay(scenario)

    assert replay.sim.DifferentialDrivePlant.sensors is observing_sensors
    assert len(rows) == len(captured_truth) == 10
    assert len(captured_data) == 1
    data = captured_data[0]
    assert data.shape[0] == 20

    # Ensure this test can distinguish sensor-time truth from post-step truth.
    assert any(
        np.linalg.norm(truth - data[2 * i, 1:3]) > 1e-8
        for i, truth in enumerate(captured_truth)
    )

    for i, row in enumerate(rows):
        t = i * replay.SENSOR_DT
        assert row["t"] == pytest.approx(t, abs=1e-12)
        np.testing.assert_allclose(
            row["true"], captured_truth[i], rtol=0.0, atol=1e-12
        )
        expected_ekf = (0.0, 0.0) if i == 0 else data[2 * i - 1, 4:6]
        if i:
            assert data[2 * i - 1, 0] == pytest.approx(t, abs=1e-12)
        np.testing.assert_allclose(
            row["ekf"], expected_ekf, rtol=0.0, atol=1e-12
        )
        np.testing.assert_allclose(
            row["ref"], replay.sim.reference(t)[:2],
            rtol=0.0, atol=1e-12,
        )


def test_replay_restores_sensor_method_when_trial_raises(monkeypatch):
    replay = load_replay()
    original_sensors = replay.sim.DifferentialDrivePlant.sensors

    def failing_run(*args, **kwargs):
        raise RuntimeError("deliberate regression-test failure")

    monkeypatch.setattr(replay.sim, "run_trial", failing_run)
    with pytest.raises(RuntimeError, match="deliberate regression-test failure"):
        replay.replay("nominal")
    assert replay.sim.DifferentialDrivePlant.sensors is original_sensors
