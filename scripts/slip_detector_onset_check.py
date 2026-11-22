#!/usr/bin/env python3
"""Pre-registered check: does slip-detection latency depend on WHEN the traction loss starts?

The detector configuration and the criteria below were fixed BEFORE this script was
run, and no parameter is tuned here. The 2.0 s latency limit was chosen knowing the
earlier result at onset 12 s (median about 1.7 s), so O2 is not a blind test.
The simulator, estimator and controller are unchanged: sensors() is wrapped only to
record the raw streams, and the existing fault_windows argument of sim.run_trial()
moves the 4 s traction-loss window (traction 0.65 from simulation.py). The detector
is replayed offline and never feeds back into the loop.

Scope: slip scenario only, one fixed reference trajectory, simulator's uniform
traction-loss model (traction scales both v and yaw rate).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from ros2_amr_nmpc_telemetry.slip_detector import YawRateSlipDetector  # noqa: E402

SENSOR_DT = 0.02
WARMUP_END = 2.0
THRESHOLD = 0.030
CONFIRM = 1
SEEDS = tuple(range(701, 711))
ONSETS = (4.0, 8.0, 12.0, 16.0, 20.0, 24.0)
WINDOW_LENGTH = 4.0
DURATION = 30.0

CRITERIA = (
    'O1 every onset: slip detected inside the fault window in at least 9 of 10 seeds',
    'O2 every onset: median first-alarm latency <= 2.0 s',
    'O3 every onset: mean alarm fraction before the fault (2.0 s to onset - 0.5 s) <= 0.005',
    'O4 every onset: mean alarm fraction from fault end + 1.0 s to end of run <= 0.005',
)


def record(onset, seed):
    records = []
    original = sim.DifferentialDrivePlant.sensors

    def recording_sensors(self):
        wheel, imu = original(self)
        records.append((float(wheel[1]), float(imu), float(self.velocity[1])))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = recording_sensors
    try:
        sim.run_trial('lyapunov_integral', 'slip', seed, DURATION,
                      fault_windows=((onset, onset + WINDOW_LENGTH),))
    finally:
        sim.DifferentialDrivePlant.sensors = original
    arr = np.asarray(records)
    t = SENSOR_DT * np.arange(len(arr))
    return t, arr


def alarms_for(arr):
    detector = YawRateSlipDetector(threshold=THRESHOLD, confirm_samples=CONFIRM)
    return np.array([detector.update(w, g) for w, g in zip(arr[:, 0], arr[:, 1])], dtype=bool)


def fraction(alarms, mask):
    return float(np.mean(alarms[mask])) if mask.any() else 0.0


def verdict(name, ok):
    print('[%s] %s' % ('PASS' if ok else 'FAIL', name), flush=True)


def main():
    print('simulation module:', sim.__file__)
    print('detector: threshold=%.4f confirm=%d window=25 warmup=100 samples' % (THRESHOLD, CONFIRM))
    print('fresh seeds:', SEEDS)
    print('onsets (s):', ONSETS, '| fault length %.1f s | duration %.0f s | traction 0.65'
          % (WINDOW_LENGTH, DURATION))
    print('criteria (fixed before running):')
    for line in CRITERIA:
        print('  ' + line)

    results = []
    print('\n##### per-onset results', flush=True)
    for onset in ONSETS:
        end = onset + WINDOW_LENGTH
        latencies, turn_rates, pre, post = [], [], [], []
        for seed in SEEDS:
            t, arr = record(onset, seed)
            alarms = alarms_for(arr)
            window = (t >= onset) & (t < end)
            early = (t >= onset) & (t < onset + 2.0)
            first = np.flatnonzero(alarms & window)
            if len(first):
                latencies.append(float(t[first[0]] - onset))
            turn_rates.append(float(np.mean(np.abs(arr[early, 2]))))
            pre.append(fraction(alarms, (t >= WARMUP_END) & (t < onset - 0.5)))
            post.append(fraction(alarms, t >= end + 1.0))
        hits = len(latencies)
        median = float(np.median(latencies)) if latencies else math.inf
        results.append({'onset': onset, 'hits': hits, 'median': median,
                        'pre': float(np.mean(pre)), 'post': float(np.mean(post))})
        text = ', '.join('%.2f' % x for x in latencies) if latencies else 'none'
        print('onset %5.1f s | detected %2d/%d | latencies s [%s] | median %.2f | '
              'mean |true yaw rate| in first 2 s of fault %.3f rad/s | '
              'alarm frac before fault %.5f | after fault+1 s %.5f'
              % (onset, hits, len(SEEDS), text, median, float(np.mean(turn_rates)),
                 float(np.mean(pre)), float(np.mean(post))), flush=True)

    print('\n##### verdicts (computed by this script)')
    verdict(CRITERIA[0], all(r['hits'] >= 9 for r in results))
    verdict(CRITERIA[1], all(r['median'] <= 2.0 for r in results))
    verdict(CRITERIA[2], all(r['pre'] <= 0.005 for r in results))
    verdict(CRITERIA[3], all(r['post'] <= 0.005 for r in results))
    print('onsets failing O1:', [r['onset'] for r in results if r['hits'] < 9])
    print('onsets failing O2:', [r['onset'] for r in results if r['median'] > 2.0])
    print('onsets failing O3:', [r['onset'] for r in results if r['pre'] > 0.005])
    print('onsets failing O4:', [r['onset'] for r in results if r['post'] > 0.005])


if __name__ == '__main__':
    main()
