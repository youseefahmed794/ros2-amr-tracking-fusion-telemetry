import math
import numpy as np
import pytest
from ros2_amr_nmpc_telemetry.core import Tracker, VelocityEKF, reference


def test_reference_derivatives():
    for t in np.linspace(0, 30, 30):
        a, b = reference(t-1e-5), reference(t+1e-5)
        r = reference(t)
        assert math.hypot((b[0]-a[0])/2e-5, (b[1]-a[1])/2e-5) == pytest.approx(r[3], abs=1e-8)
        delta = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        assert delta/2e-5 == pytest.approx(r[4], abs=1e-8)


def test_filter_velocity_fusion_and_covariance():
    f = VelocityEKF()
    rng = np.random.default_rng(42)
    for _ in range(1000):
        f.update([1+rng.normal(0, 0.02), 0.2], (3, 4), f.R_odom)
        f.update_imu(0.2)
        f.predict(0.02)
        assert np.linalg.eigvalsh(f.P).min() >= -1e-12
        assert np.allclose(f.P, f.P.T)
    assert f.x[3, 0] == pytest.approx(1, abs=0.03)
    assert f.x[4, 0] == pytest.approx(0.2, abs=0.001)
    assert -math.pi <= f.x[2, 0] <= math.pi
    before = f.x.copy()
    assert not f.update([float('nan')], (4,), f.R_imu)
    np.testing.assert_array_equal(before, f.x)


def test_limits_and_fail_closed():
    tracker = Tracker()
    previous = 0
    for t in np.arange(0, 20, 0.02):
        v, w = tracker.step((-100, 100, 0), t, 0.02)
        assert abs(v) <= 1.5 and abs(w) <= 2
        assert abs(v-previous) <= 2.5*0.02+1e-12
        assert abs(tracker.integral) <= 0.4
        previous = v
    assert tracker.step((0, 0, 0), 20, -1) == (0, 0)
    assert tracker.integral == tracker.previous_v == 0
    assert tracker.step((float('nan'), 0, 0), 0, 0.02) == (0, 0)


@pytest.mark.parametrize('dt', [0, -1, 1, float('nan')])
def test_bad_prediction_interval(dt):
    with pytest.raises(ValueError):
        VelocityEKF().predict(dt)
