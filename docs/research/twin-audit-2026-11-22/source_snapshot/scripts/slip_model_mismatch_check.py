#!/usr/bin/env python3
"""Pre-registered check: what happens to the slip compensation when the slip model is NOT the one it assumes?

The compensator (scripts/slip_compensation_experiment.py) assumes that traction loss scales the
linear AND the yaw velocity by the same factor. Here the plant's step() is replaced by an exact
copy in which the two factors differ; nothing else changes. Slip scenario only, 30 s runs,
fault window 12-16 s, traction 0.65, fresh seeds. Baseline (no compensation) vs compensated.

Variants (v factor / yaw factor at traction 0.65):
  matched      0.650 / 0.650   the model the compensator assumes
  linear-only  0.650 / 1.000   v scaled, yaw not
  yaw-only     1.000 / 0.650   yaw scaled, v not
  half-linear  0.825 / 0.650   v loses half as much as yaw

Predictions written BEFORE the first run: linear-only is blind (yaw is unaffected, so the detector
never alarms and compensated == baseline); yaw-only is harmed (the compensator shrinks v although
true v is unchanged); half-linear is uncertain.

Pre-registered criteria, no parameter is tuned:
  X0 precondition: with the matched factors the copied step() reproduces the original plant
     (true tracking RMSE identical within 1e-12 for seed 1001), otherwise the script stops.
  X1 linear-only: compensation active fraction <= 0.005 and compensated / baseline true RMSE <= 1.02
  X2 yaw-only: compensated / baseline true RMSE <= 1.10
  X3 half-linear: compensated / baseline true RMSE <= 1.00
Simulation only. Results describe these four simulator variants, not real wheel slip.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from slip_compensation_experiment import SlipCompensator  # noqa: E402

SEEDS = tuple(range(1001, 1011))
DURATION = 30.0

VARIANTS = (
    ('matched', lambda tr: tr, lambda tr: tr),
    ('linear-only', lambda tr: tr, lambda tr: 1.0),
    ('yaw-only', lambda tr: 1.0, lambda tr: tr),
    ('half-linear', lambda tr: 1.0 - 0.5 * (1.0 - tr), lambda tr: tr),
)


def make_step(v_factor, w_factor):
    def step(self, t, dt, traction=1.0, load=1.0):
        if not 0 < dt <= 0.1 or not 0 <= traction <= 1 or not 0 < load <= 1:
            raise ValueError('Invalid plant integration step or disturbance')
        c = self.config
        age = t - self.last_command
        self.watchdog = not 0 <= age < c.command_timeout
        target = np.zeros(2) if self.watchdog else self.target * load
        delta = (target - self.wheels) * (1 - math.exp(-dt / c.motor_time_constant))
        self.wheels += np.clip(delta, -c.wheel_accel_limit * dt, c.wheel_accel_limit * dt)
        self.wheel_position += self.wheels * dt
        left, right = self.wheels * c.wheel_radius
        v = v_factor(traction) * (left + right) / 2
        w = w_factor(traction) * (right - left) / c.wheel_separation
        self.pose[0] += v * math.cos(self.pose[2] + w * dt / 2) * dt
        self.pose[1] += v * math.sin(self.pose[2] + w * dt / 2) * dt
        self.pose[2] = sim.normalize_angle(self.pose[2] + w * dt)
        self.velocity[:] = v, w
    return step


def run(seed, compensate, step_override=None):
    compensator = SlipCompensator() if compensate else None
    original_sensors = sim.DifferentialDrivePlant.sensors
    original_step = sim.DifferentialDrivePlant.step

    def wrapped(self):
        wheel, imu = original_sensors(self)
        if compensator is not None:
            wheel = wheel * compensator.factor(float(wheel[1]), float(imu))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = wrapped
    if step_override is not None:
        sim.DifferentialDrivePlant.step = step_override
    try:
        _, metrics = sim.run_trial('lyapunov_integral', 'slip', seed, DURATION)
    finally:
        sim.DifferentialDrivePlant.sensors = original_sensors
        sim.DifferentialDrivePlant.step = original_step
    active = compensator.active / compensator.samples if compensator else 0.0
    return metrics, active


def verdict(name, ok):
    print('[%s] %s' % ('PASS' if ok else 'FAIL', name), flush=True)


def main():
    print('simulation module:', sim.__file__)
    print('seeds:', SEEDS)

    reference, _ = run(SEEDS[0], False)
    copy, _ = run(SEEDS[0], False, make_step(VARIANTS[0][1], VARIANTS[0][2]))
    difference = abs(reference['true_tracking_rmse_m'] - copy['true_tracking_rmse_m'])
    print('X0 copy check: original %.15f | copy %.15f | difference %.3e'
          % (reference['true_tracking_rmse_m'], copy['true_tracking_rmse_m'], difference))
    if difference > 1e-12:
        print('[FAIL] X0 the copied step() does not reproduce the original plant; stopping.')
        raise SystemExit(1)
    print('[PASS] X0 the copied step() reproduces the original plant', flush=True)

    ratios = {}
    active_fraction = {}
    for name, v_factor, w_factor in VARIANTS:
        override = make_step(v_factor, w_factor)
        base, comp, loc0, loc1, est0, est1, active = [], [], [], [], [], [], []
        for seed in SEEDS:
            m0, _ = run(seed, False, override)
            m1, a = run(seed, True, override)
            base.append(m0['true_tracking_rmse_m'])
            comp.append(m1['true_tracking_rmse_m'])
            loc0.append(m0['localization_rmse_m'])
            loc1.append(m1['localization_rmse_m'])
            est0.append(m0['estimated_tracking_rmse_m'])
            est1.append(m1['estimated_tracking_rmse_m'])
            active.append(a)
        ratios[name] = float(np.mean(comp) / np.mean(base))
        active_fraction[name] = float(np.mean(active))
        print('\n##### variant %s (v factor %.3f, yaw factor %.3f at traction 0.65)'
              % (name, v_factor(0.65), w_factor(0.65)))
        print('                         baseline   compensated')
        print('true tracking RMSE [m]   %8.3f   %11.3f' % (np.mean(base), np.mean(comp)))
        print('est. tracking RMSE [m]   %8.3f   %11.3f' % (np.mean(est0), np.mean(est1)))
        print('localization RMSE [m]    %8.3f   %11.3f' % (np.mean(loc0), np.mean(loc1)))
        print('true RMSE ratio compensated / baseline: %.3f | compensation active fraction: %.5f'
              % (ratios[name], active_fraction[name]))
        print('per-seed true RMSE baseline:    ' + ' '.join('%.3f' % x for x in base))
        print('per-seed true RMSE compensated: ' + ' '.join('%.3f' % x for x in comp), flush=True)

    print('\n##### verdicts (computed by this script)')
    verdict('X1 linear-only: active fraction <= 0.005 and ratio <= 1.02',
            active_fraction['linear-only'] <= 0.005 and ratios['linear-only'] <= 1.02)
    verdict('X2 yaw-only: ratio <= 1.10', ratios['yaw-only'] <= 1.10)
    verdict('X3 half-linear: ratio <= 1.00', ratios['half-linear'] <= 1.00)
    print('ratios: ' + ' | '.join('%s %.3f' % (k, v) for k, v in ratios.items()))


if __name__ == '__main__':
    main()
