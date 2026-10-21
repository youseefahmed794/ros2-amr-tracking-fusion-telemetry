"""Bias observability, covariance and explicit wheel-reference limitations."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from ros2_amr_nmpc_telemetry.core import VelocityEKF, normalize_angle


@pytest.mark.parametrize('bias', [-0.02, 0.005, 0.03])
def test_unknown_signed_bias_converges_with_unbiased_wheels(bias):
    f = VelocityEKF()
    rng = np.random.default_rng(302)
    for k in range(3000):
        # Vary rate: calibration does not require stationary robot or truth pose.
        omega = 0.3*np.sin(k*0.02)
        f.update([0.7, omega+rng.normal(0,0.03)], (3,4), f.R_odom)
        f.update_imu(omega+bias+rng.normal(0,0.01))
        f.predict(0.02)
        if k%100==0:
            assert np.linalg.eigvalsh(f.P).min()>=-1e-12
            assert np.allclose(f.P,f.P.T)
    assert f.x[5,0] == pytest.approx(bias,abs=0.002)
    assert f.x[4,0] == pytest.approx(omega,abs=0.02)


def test_gyro_alone_does_not_observe_rate_bias_difference():
    f = VelocityEKF(initial_bias_variance=0.1)
    unobservable = np.array([0,0,0,0,1,-1])
    initial = unobservable@f.P@unobservable
    for _ in range(300):
        f.update_imu(0.21)
    assert unobservable@f.P@unobservable == pytest.approx(initial,abs=1e-12)
    assert f.x[4,0] == pytest.approx(f.x[5,0])
    assert f.P[5,5]>0.04


def test_wheel_rate_bias_is_indistinguishable_from_gyro_bias():
    f = VelocityEKF()
    for _ in range(1500):
        # True rate .2, wheel rate has +.01 error, gyro has +.005 bias.
        f.update([0.7,0.21], (3,4), f.R_odom)
        f.update_imu(0.205)
        f.predict(0.02)
    assert f.x[5,0] == pytest.approx(-0.005,abs=0.0001)
    assert f.x[4,0] == pytest.approx(0.21,abs=0.0001)


def test_no_absolute_heading_correction_from_bias_estimation():
    a,b = VelocityEKF(),VelocityEKF()
    b.x[2,0] = 1.0
    for _ in range(500):
        for f in (a,b):
            f.update([0.7,0.2],(3,4),f.R_odom)
            f.update_imu(0.205)
            f.predict(0.02)
    assert normalize_angle(b.x[2,0]-a.x[2,0]) == pytest.approx(1.0,abs=1e-12)


def test_disabled_bias_reproduces_archived_five_state_filter():
    source = Path(__file__).resolve().parents[1]/'docs/release/gyro-bias/baseline-source/core.py'
    spec = importlib.util.spec_from_file_location('archived_bias_baseline',source)
    archived = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(archived)
    old,new = archived.VelocityEKF(),VelocityEKF(estimate_gyro_bias=False)
    rng = np.random.default_rng(404)
    for _ in range(500):
        wheel = rng.normal([0.5,0.2],[0.03,0.03])
        gyro = 0.205+rng.normal(0,0.01)
        old.update(wheel,(3,4),old.R_odom)
        old.update([gyro],(4,),old.R_imu)
        old.predict(0.02)
        new.update(wheel,(3,4),new.R_odom)
        new.update_imu(gyro)
        new.predict(0.02)
    np.testing.assert_allclose(new.x[:5],old.x,atol=1e-12,rtol=0)
    np.testing.assert_allclose(new.P[:5,:5],old.P,atol=1e-12,rtol=0)
    assert new.x[5,0] == new.P[5,5] == 0


def test_invalid_gyro_does_not_change_bias_or_covariance():
    f = VelocityEKF()
    before_x,before_p = f.x.copy(),f.P.copy()
    assert not f.update_imu(float('nan'))
    np.testing.assert_array_equal(f.x,before_x)
    np.testing.assert_array_equal(f.P,before_p)


def test_rate_updates_preserve_integrated_pose_and_its_covariance_block():
    f = VelocityEKF()
    f.update([0.7,0.2],(3,4),f.R_odom)
    f.update_imu(0.205)
    f.predict(0.02)
    pose,covariance = f.x[:3].copy(),f.P[:3,:3].copy()
    f.update([0.8,0.3],(3,4),f.R_odom)
    f.update_imu(0.31)
    np.testing.assert_array_equal(f.x[:3],pose)
    np.testing.assert_allclose(f.P[:3,:3],covariance,atol=1e-15,rtol=0)
    assert np.linalg.eigvalsh(f.P).min()>=-1e-12
    assert f.x[5,0] != 0
