#!/usr/bin/env python3
"""Pre-registered confirmation of YawRateSlipDetector on fresh seeds (detection only).

The configuration and the pass/fail criteria below were fixed BEFORE this script
was run, and no parameter is tuned here. The simulator, estimator and controller
are unchanged: sensors() is wrapped only to record the raw streams, and step() is
wrapped only to replace the traction value used inside the slip window (severity
sweep). The detector is replayed offline and never feeds back into the loop.

Scope: nominal, load and slip scenarios only. Sensor-fault scenarios (dropouts,
delay, non-finite values) are NOT evaluated because the recorded streams do not
contain those faults. Results hold for the simulator's uniform traction-loss
model, where traction scales both v and yaw rate.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from ros2_amr_nmpc_telemetry.slip_detector import YawRateSlipDetector  # noqa: E402

SENSOR_DT = 0.02
FAULT = (12.0, 16.0)
WARMUP_END = 2.0
THRESHOLD = 0.030
CONFIRM = 1
SEEDS = tuple(range(501, 511))
LONG_SEEDS = tuple(range(601, 613))
LONG_DURATION = 300.0
SWEEP = (0.65, 0.80, 0.90, 0.95)

CRITERIA = (
    'C1 slip (traction 0.65): detected in at least 9 of 10 seeds',
    'C2 slip (traction 0.65): median first-alarm latency <= 2.0 s',
    'C3 nominal 30 s runs: mean alarm fraction <= 0.005',
    'C4 load 30 s runs: mean alarm fraction inside the fault window <= 0.005',
    'C5 nominal long runs (12 x 300 s): false-alarm events <= 1',
)


def record(scenario, seed, duration, traction_override=None):
    records = []
    original_sensors = sim.DifferentialDrivePlant.sensors
    original_step = sim.DifferentialDrivePlant.step

    def recording_sensors(self):
        wheel, imu = original_sensors(self)
        records.append((float(wheel[1]), float(imu), float(self.velocity[1])))
        return wheel, imu

    def scaled_step(self, t, dt, traction=1.0, load=1.0):
        if traction_override is not None and traction < 1.0:
            traction = traction_override
        return original_step(self, t, dt, traction=traction, load=load)

    sim.DifferentialDrivePlant.sensors = recording_sensors
    sim.DifferentialDrivePlant.step = scaled_step
    try:
        sim.run_trial('lyapunov_integral', scenario, seed, duration)
    finally:
        sim.DifferentialDrivePlant.sensors = original_sensors
        sim.DifferentialDrivePlant.step = original_step
    arr = np.asarray(records)
    t = SENSOR_DT * np.arange(len(arr))
    return t, arr


def alarms_for(arr):
    detector = YawRateSlipDetector(threshold=THRESHOLD, confirm_samples=CONFIRM)
    return np.array([detector.update(w, g) for w, g in zip(arr[:, 0], arr[:, 1])], dtype=bool)


def slip_stats(traction):
    hits, latencies, fractions = 0, [], []
    for seed in SEEDS:
        t, arr = record('slip', seed, 30.0, traction)
        alarms = alarms_for(arr)
        fault = (t >= FAULT[0]) & (t < FAULT[1])
        first = np.flatnonzero(alarms & fault)
        if len(first):
            hits += 1
            latencies.append(float(t[first[0]] - FAULT[0]))
        fractions.append(float(np.mean(alarms[fault])))
    median = float(np.median(latencies)) if latencies else math.inf
    return hits, latencies, median, float(np.mean(fractions))


def verdict(name, ok):
    print('[%s] %s' % ('PASS' if ok else 'FAIL', name), flush=True)


def main():
    print('simulation module:', sim.__file__)
    print('detector: threshold=%.4f confirm=%d window=25 warmup=100 samples' % (THRESHOLD, CONFIRM))
    print('fresh seeds: slip/load/nominal', SEEDS, '| long nominal', LONG_SEEDS)
    print('criteria (fixed before running):')
    for line in CRITERIA:
        print('  ' + line)

    print('\n##### slip severity sweep (exploratory except traction 0.65)', flush=True)
    sweep = {}
    for traction in SWEEP:
        sweep[traction] = slip_stats(traction)
        hits, latencies, median, fraction = sweep[traction]
        text = ', '.join('%.2f' % x for x in latencies) if latencies else 'none'
        print('traction=%.2f | detected %d/%d | latencies s [%s] | median %.2f | alarm frac in window %.2f'
              % (traction, hits, len(SEEDS), text, median, fraction), flush=True)

    print('\n##### nominal and load, 30 s runs', flush=True)
    nominal = []
    load = []
    for seed in SEEDS:
        t, arr = record('nominal', seed, 30.0)
        nominal.append(float(np.mean(alarms_for(arr)[t >= WARMUP_END])))
        t, arr = record('load', seed, 30.0)
        window = (t >= FAULT[0]) & (t < FAULT[1])
        load.append(float(np.mean(alarms_for(arr)[window])))
    print('nominal alarm fraction: mean %.5f, max %.5f' % (np.mean(nominal), np.max(nominal)))
    print('load alarm fraction in window: mean %.5f, max %.5f' % (np.mean(load), np.max(load)), flush=True)

    print('\n##### long nominal runs', flush=True)
    events, minutes = 0, 0.0
    for seed in LONG_SEEDS:
        t, arr = record('nominal', seed, LONG_DURATION)
        alarms = alarms_for(arr)[t >= WARMUP_END]
        run_events = int(np.sum(alarms[1:] & ~alarms[:-1])) + (1 if alarms[0] else 0)
        events += run_events
        minutes += (t[-1] - WARMUP_END) / 60.0
        print('seed %d: false-alarm events %d' % (seed, run_events), flush=True)
    print('total false-alarm events %d over %.1f min = %.3f per min' % (events, minutes, events / minutes))

    hits, _, median, _ = sweep[0.65]
    print('\n##### verdicts (computed by this script)')
    verdict(CRITERIA[0], hits >= 9)
    verdict(CRITERIA[1], median <= 2.0)
    verdict(CRITERIA[2], float(np.mean(nominal)) <= 0.005)
    verdict(CRITERIA[3], float(np.mean(load)) <= 0.005)
    verdict(CRITERIA[4], events <= 1)


if __name__ == '__main__':
    main()
