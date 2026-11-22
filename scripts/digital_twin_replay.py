#!/usr/bin/env python3
"""Replay simulator runs through AMRDigitalTwin and write a JSON log for a dashboard.

Exploratory, no pre-registered criteria. The simulator, estimator and controller are
unchanged: sensors() is wrapped only to record the raw readings the EKF receives. The
twin is fed the same readings offline. The EKF in these runs is the baseline EKF (no
slip compensation). Everything here is a simulation replay, not hardware data.
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from ros2_amr_nmpc_telemetry.digital_twin import AMRDigitalTwin  # noqa: E402

SEED = 1101
DURATION = 30.0
FAULT = (12.0, 16.0)
SENSOR_DT = 0.02
STRIDE = 5
OUTPUT = os.path.join(ROOT, 'docs', 'twin_demo', 'replay.json')


def replay(scenario):
    records = []
    original = sim.DifferentialDrivePlant.sensors

    def recording(self):
        wheel, imu = original(self)
        # Ground truth is captured at the sensor timestamp, before plant.step().
        records.append((float(wheel[0]), float(wheel[1]), float(imu),
                        float(self.pose[0]), float(self.pose[1])))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = recording
    try:
        data, metrics = sim.run_trial('lyapunov_integral', scenario, SEED, DURATION,
                                      fault_windows=(FAULT,))
    finally:
        sim.DifferentialDrivePlant.sensors = original

    twin = AMRDigitalTwin()
    rows = []
    for i, (wheel_v, wheel_w, gyro_w, true_x, true_y) in enumerate(records):
        t = SENSOR_DT * i
        out = twin.update(t, wheel_v, wheel_w, gyro_w)
        k = 2 * i
        # run_trial rows describe the end of each 10 ms step.
        # The preceding row contains the EKF pose at this sensor time.
        if i == 0:
            estimated_xy = (0.0, 0.0)
        else:
            if not np.isclose(data[k - 1, 0], t, rtol=0.0, atol=1e-9):
                raise ValueError('Replay sensor and EKF timestamps do not align')
            estimated_xy = (float(data[k - 1, 4]), float(data[k - 1, 5]))
        reference_xy = sim.reference(t)[:2]
        in_fault = scenario == 'slip' and FAULT[0] <= t < FAULT[1]
        rows.append({
            't': t,
            'true': (true_x, true_y),
            'ekf': estimated_xy,
            'ref': reference_xy,
            'twin': (out['x'], out['y']),
            'traction_true': 0.65 if in_fault else 1.0,
            'traction_est': out['traction'],
            'slip': bool(out['slip']),
            'gyro_bias': out['gyro_bias'],
        })
    return rows, metrics


def main():
    print('simulation module:', sim.__file__)
    print('seed %d, %.0f s, fault window %s s (slip scenario only)' % (SEED, DURATION, FAULT))
    log = {'seed': SEED, 'duration_s': DURATION, 'fault_window_s': list(FAULT),
           'note': 'simulation replay, not hardware data', 'scenarios': {}}
    for scenario in ('nominal', 'slip'):
        rows, metrics = replay(scenario)
        ekf_err = np.array([np.hypot(r['true'][0] - r['ekf'][0], r['true'][1] - r['ekf'][1]) for r in rows])
        twin_err = np.array([np.hypot(r['true'][0] - r['twin'][0], r['true'][1] - r['twin'][1]) for r in rows])
        t = np.array([r['t'] for r in rows])
        slip_flags = np.array([r['slip'] for r in rows])
        window = (t >= FAULT[0]) & (t < FAULT[1])
        first = np.flatnonzero(slip_flags & window)
        latency = '%.2f s' % (t[first[0]] - FAULT[0]) if scenario == 'slip' and len(first) else 'none'
        print('\n##### %s' % scenario)
        print('mean |true - EKF| position error [m]:   %.3f   (end of run %.3f)' % (ekf_err.mean(), ekf_err[-1]))
        print('mean |true - twin| position error [m]:  %.3f   (end of run %.3f)' % (twin_err.mean(), twin_err[-1]))
        print('twin slip alarm fraction of run: %.4f | first alarm after fault start: %s'
              % (slip_flags.mean(), latency))
        est = np.array([r['traction_est'] for r in rows])
        if scenario == 'slip':
            late = (t >= FAULT[0] + 2.5) & (t < FAULT[1])
            print('twin traction estimate, last 1.5 s of the fault: mean %.3f (true 0.650)' % est[late].mean())
        print('twin gyro-bias estimate at end: %.4f rad/s (injected 0.0050)' % rows[-1]['gyro_bias'])
        keep = rows[::STRIDE]
        log['scenarios'][scenario] = {
            'fields': ['t', 'true_x', 'true_y', 'ekf_x', 'ekf_y', 'twin_x', 'twin_y', 'ref_x', 'ref_y',
                       'traction_true', 'traction_est', 'slip', 'gyro_bias'],
            'rows': [[round(r['t'], 3), round(r['true'][0], 4), round(r['true'][1], 4),
                      round(r['ekf'][0], 4), round(r['ekf'][1], 4),
                      round(r['twin'][0], 4), round(r['twin'][1], 4),
                      round(r['ref'][0], 4), round(r['ref'][1], 4),
                      r['traction_true'], round(r['traction_est'], 4), int(r['slip']),
                      round(r['gyro_bias'], 5)] for r in keep],
        }
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    with open(OUTPUT, 'w') as handle:
        json.dump(log, handle, separators=(',', ':'))
    print('\nwrote %s (%d bytes)' % (OUTPUT, os.path.getsize(OUTPUT)))


if __name__ == '__main__':
    main()
