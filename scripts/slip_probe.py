#!/usr/bin/env python3
"""Observability probe (measurement only, changes nothing).

Question: does the disagreement between wheel yaw rate and raw gyro yaw rate
reveal the simulated traction loss (scenario 'slip': traction 0.65 during 12-16 s)?

The simulator, estimator and controller are not modified. The plant's sensors()
method is wrapped only to record raw samples while sim.run_trial() runs unchanged.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402

SENSOR_DT = 0.02  # sensors() is called on every second 0.01 s tick
WINDOW = 25  # samples, 0.5 s
FAULT = (12.0, 16.0)
CALIBRATION_SEEDS = (201, 202, 203)
EVALUATION_SEEDS = (301, 302, 303, 304, 305)


def run_recorded(scenario, seed):
    records = []
    original = sim.DifferentialDrivePlant.sensors

    def recording_sensors(self):
        wheel, imu = original(self)
        records.append((float(wheel[0]), float(wheel[1]), float(imu),
                        float(self.velocity[0]), float(self.velocity[1])))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = recording_sensors
    try:
        _, metrics = sim.run_trial('lyapunov_integral', scenario, seed, 30.0)
    finally:
        sim.DifferentialDrivePlant.sensors = original
    arr = np.asarray(records)
    t = SENSOR_DT * np.arange(len(arr))
    return t, arr, metrics


def windowed_mean(x, n):
    c = np.cumsum(np.insert(x, 0, 0.0))
    out = np.full(len(x), np.nan)
    out[n - 1:] = (c[n:] - c[:-n]) / n
    return out


def residual_stats(arr):
    d = arr[:, 1] - arr[:, 2]  # wheel yaw rate minus raw gyro rate
    return d, windowed_mean(d, WINDOW)


def main():
    print('simulation module:', sim.__file__)
    print('numpy:', np.__version__)
    print(f'window = {WINDOW} samples = {WINDOW * SENSOR_DT:.2f} s, fault window = {FAULT} s')

    calibration = []
    for seed in CALIBRATION_SEEDS:
        t, arr, _ = run_recorded('nominal', seed)
        _, m = residual_stats(arr)
        calibration.append(m[t >= 1.0])
    calibration = np.concatenate(calibration)
    center = float(np.median(calibration))
    sigma = float(np.std(calibration))
    threshold = float(np.percentile(np.abs(calibration - center), 99.9))
    print(f'calibration (nominal, seeds {CALIBRATION_SEEDS}): '
          f'center = {center:.4f} rad/s, std = {sigma:.4f}, threshold = {threshold:.4f} rad/s')

    profile = None
    for scenario in ('nominal', 'slip'):
        print(f'\n=== scenario: {scenario} (evaluation seeds {EVALUATION_SEEDS}) ===')
        for seed in EVALUATION_SEEDS:
            t, arr, metrics = run_recorded(scenario, seed)
            _, m = residual_stats(arr)
            with np.errstate(invalid='ignore'):
                alarm = np.abs(m - center) > threshold
            fault = (t >= FAULT[0]) & (t < FAULT[1])
            pre = (t >= 1.0) & (t < FAULT[0] - 0.5)
            true_w = arr[:, 4]
            gap = np.abs(arr[:, 1] - true_w)  # hidden truth: wheel yaw rate minus true yaw rate
            hits = np.flatnonzero(alarm & fault)
            latency = f'{t[hits[0]] - FAULT[0]:.2f}s' if len(hits) else 'none'
            print(f'seed {seed}: true_rmse={metrics["true_tracking_rmse_m"]:.3f} m  '
                  f'est_rmse={metrics["estimated_tracking_rmse_m"]:.3f} m | '
                  f'in fault window: mean|true_w|={np.mean(np.abs(true_w[fault])):.3f}  '
                  f'mean wheel-vs-true gap={np.mean(gap[fault]):.3f}  '
                  f'alarm fraction={np.mean(alarm[fault]):.2f}  first alarm after {latency} | '
                  f'before fault: gap={np.mean(gap[pre]):.3f}  false-alarm fraction={np.mean(alarm[pre]):.3f}')
            if scenario == 'slip' and profile is None:
                profile = (t, m, alarm, true_w)

    t, m, alarm, true_w = profile
    print(f'\nPer-second profile, scenario slip, seed {EVALUATION_SEEDS[0]}')
    print('  t0   mean|true_w|   mean|m-center|   alarm_fraction')
    for a in range(9, 19):
        sel = (t >= a) & (t < a + 1)
        print(f'{a:4d}   {np.mean(np.abs(true_w[sel])):11.3f}   '
              f'{np.mean(np.abs(m[sel] - center)):13.4f}   {np.mean(alarm[sel]):13.2f}')
    print('\nUnits: rad/s. One sample = 0.02 s. Alarm fractions are per 50 Hz sample.')


if __name__ == '__main__':
    main()
