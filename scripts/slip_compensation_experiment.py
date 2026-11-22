#!/usr/bin/env python3
"""Pre-registered experiment: slip-compensated wheel odometry (simulation only, model-dependent).

The simulator, estimator and controller files are unchanged. sensors() is wrapped
only to hand the EKF a corrected wheel reading while the slip detector is in alarm:
the wheel (v, yaw rate) reading is multiplied by a traction factor s estimated as
the regression slope of (gyro - bias) on the wheel yaw rate over the last 50 samples
(1 s at 50 Hz), shrunk toward s = 1 by a ridge term set from the encoder noise.

MODEL DEPENDENCE: this works only because the simulator scales the linear AND the
yaw velocity by the same traction factor. Real wheel slip does not have to do that.
A positive result is a statement about this simulator model, not about real slip.

Pre-registered, no parameter is tuned: window 50 samples, ridge = window * encoder_std^2,
s clipped to [0.3, 1.0], detector threshold 0.030 with confirm_samples 1.
Compared paired by seed: baseline (no compensation) vs compensated, 30 s runs,
fault window 12-16 s from simulation.py (traction 0.65 for slip, load factor 0.7 for load).
"""
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from ros2_amr_nmpc_telemetry.slip_detector import YawRateSlipDetector  # noqa: E402

THRESHOLD = 0.030
CONFIRM = 1
WINDOW = 50
ENCODER_STD = 0.03
PRIOR = WINDOW * ENCODER_STD ** 2
S_MIN = 0.3
SEEDS = tuple(range(901, 911))
DURATION = 30.0

CRITERIA = (
    'B1 slip: mean true tracking RMSE compensated / baseline <= 0.50',
    'B2 slip: mean localization RMSE (true vs estimated pose) compensated / baseline <= 0.50',
    'B3 nominal: mean true tracking RMSE compensated - baseline <= 0.010 m',
    'B4 load: mean true tracking RMSE compensated - baseline <= 0.010 m',
    'B5 nominal: mean fraction of samples with compensation active <= 0.005',
)


class SlipCompensator:
    def __init__(self):
        self.detector = YawRateSlipDetector(threshold=THRESHOLD, confirm_samples=CONFIRM)
        self.cross = deque(maxlen=WINDOW)
        self.power = deque(maxlen=WINDOW)
        self.active = 0
        self.samples = 0

    def factor(self, wheel_rate, gyro_rate):
        alarm = self.detector.update(wheel_rate, gyro_rate)
        corrected_gyro = gyro_rate + self.detector.center
        self.cross.append(wheel_rate * corrected_gyro)
        self.power.append(wheel_rate * wheel_rate)
        self.samples += 1
        if not alarm:
            return 1.0
        self.active += 1
        s = (sum(self.cross) + PRIOR) / (sum(self.power) + PRIOR)
        return float(min(1.0, max(S_MIN, s)))


def run(scenario, seed, compensate):
    compensator = SlipCompensator() if compensate else None
    original = sim.DifferentialDrivePlant.sensors

    def wrapped(self):
        wheel, imu = original(self)
        if compensator is not None:
            wheel = wheel * compensator.factor(float(wheel[1]), float(imu))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = wrapped
    try:
        _, metrics = sim.run_trial('lyapunov_integral', scenario, seed, DURATION)
    finally:
        sim.DifferentialDrivePlant.sensors = original
    active = compensator.active / compensator.samples if compensator else 0.0
    return metrics, active


def verdict(name, ok):
    print('[%s] %s' % ('PASS' if ok else 'FAIL', name), flush=True)


def main():
    print('simulation module:', sim.__file__)
    print('seeds:', SEEDS)
    print('config: threshold %.3f confirm %d window %d prior %.4f s_min %.1f'
          % (THRESHOLD, CONFIRM, WINDOW, PRIOR, S_MIN))
    print('criteria (fixed before running):')
    for line in CRITERIA:
        print('  ' + line)

    summary = {}
    for scenario in ('nominal', 'load', 'slip'):
        base = {'true': [], 'est': [], 'loc': []}
        comp = {'true': [], 'est': [], 'loc': []}
        active = []
        for seed in SEEDS:
            m0, _ = run(scenario, seed, False)
            m1, a = run(scenario, seed, True)
            for store, m in ((base, m0), (comp, m1)):
                store['true'].append(m['true_tracking_rmse_m'])
                store['est'].append(m['estimated_tracking_rmse_m'])
                store['loc'].append(m['localization_rmse_m'])
            active.append(a)
        summary[scenario] = (base, comp, float(np.mean(active)))
        print('\n##### %s (mean over %d seeds)' % (scenario, len(SEEDS)))
        print('                         baseline   compensated')
        print('true tracking RMSE [m]   %8.3f   %11.3f' % (np.mean(base['true']), np.mean(comp['true'])))
        print('est. tracking RMSE [m]   %8.3f   %11.3f' % (np.mean(base['est']), np.mean(comp['est'])))
        print('localization RMSE [m]    %8.3f   %11.3f' % (np.mean(base['loc']), np.mean(comp['loc'])))
        print('compensation active fraction: %.5f' % np.mean(active))
        if scenario == 'slip':
            print('per-seed true RMSE baseline:    ' + ' '.join('%.3f' % x for x in base['true']))
            print('per-seed true RMSE compensated: ' + ' '.join('%.3f' % x for x in comp['true']), flush=True)

    b, c, _ = summary['slip']
    n0, n1, n_active = summary['nominal']
    l0, l1, _ = summary['load']
    print('\n##### verdicts (computed by this script)')
    verdict(CRITERIA[0], np.mean(c['true']) / np.mean(b['true']) <= 0.50)
    verdict(CRITERIA[1], np.mean(c['loc']) / np.mean(b['loc']) <= 0.50)
    verdict(CRITERIA[2], np.mean(n1['true']) - np.mean(n0['true']) <= 0.010)
    verdict(CRITERIA[3], np.mean(l1['true']) - np.mean(l0['true']) <= 0.010)
    verdict(CRITERIA[4], n_active <= 0.005)
    print('slip true RMSE ratio %.3f | slip localization ratio %.3f | nominal delta %.4f m | load delta %.4f m'
          % (np.mean(c['true']) / np.mean(b['true']), np.mean(c['loc']) / np.mean(b['loc']),
             np.mean(n1['true']) - np.mean(n0['true']), np.mean(l1['true']) - np.mean(l0['true'])))


if __name__ == '__main__':
    main()
