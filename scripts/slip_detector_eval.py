#!/usr/bin/env python3
"""Detection-only evaluation of YawRateSlipDetector on recorded simulator streams.

Nothing in the simulator, estimator or controller is modified: the plant's
sensors() method is wrapped only to record the raw wheel yaw rate, the raw gyro
yaw rate and the true yaw rate while sim.run_trial() runs unchanged. The detector
is then replayed offline on the recorded streams. It does not feed back, so the
recorded closed-loop behavior is identical to a run without a detector.

Protocol: configurations are compared on the calibration seeds; the held-out
seeds are reported once and are never used to choose parameters.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros2_amr_nmpc_telemetry import simulation as sim  # noqa: E402
from ros2_amr_nmpc_telemetry.slip_detector import YawRateSlipDetector  # noqa: E402

SENSOR_DT = 0.02
FAULT = (12.0, 16.0)
WARMUP_END = 2.0
CALIBRATION_SEEDS = (201, 202, 203)
HELDOUT_SEEDS = (301, 302, 303, 304, 305)
LONG_NOMINAL_SEEDS = (401, 402, 403)
LONG_DURATION = 300.0
CONFIGS = ((0.0196, 1), (0.025, 1), (0.030, 1), (0.0196, 10), (0.025, 10))


def record(scenario, seed, duration):
    records = []
    original = sim.DifferentialDrivePlant.sensors

    def recording_sensors(self):
        wheel, imu = original(self)
        records.append((float(wheel[1]), float(imu), float(self.velocity[1])))
        return wheel, imu

    sim.DifferentialDrivePlant.sensors = recording_sensors
    try:
        _, metrics = sim.run_trial('lyapunov_integral', scenario, seed, duration)
    finally:
        sim.DifferentialDrivePlant.sensors = original
    arr = np.asarray(records)
    t = SENSOR_DT * np.arange(len(arr))
    return t, arr, metrics


def alarms_for(arr, threshold, confirm):
    detector = YawRateSlipDetector(threshold=threshold, confirm_samples=confirm)
    return np.array([detector.update(w, g) for w, g in zip(arr[:, 0], arr[:, 1])], dtype=bool)


def build_streams(seeds):
    streams = {}
    for scenario in ('nominal', 'load', 'slip'):
        for seed in seeds:
            t, arr, _ = record(scenario, seed, 30.0)
            streams[(scenario, seed)] = (t, arr)
    return streams


def report(label, seeds, streams):
    print(f'\n##### {label}: seeds {seeds}')
    for threshold, confirm in CONFIGS:
        nominal, load, tail, window_frac, latencies = [], [], [], [], []
        hits = 0
        for seed in seeds:
            t, arr = streams[('nominal', seed)]
            alarms = alarms_for(arr, threshold, confirm)
            nominal.append(float(np.mean(alarms[t >= WARMUP_END])))
            t, arr = streams[('load', seed)]
            alarms = alarms_for(arr, threshold, confirm)
            load.append(float(np.mean(alarms[(t >= FAULT[0]) & (t < FAULT[1])])))
            t, arr = streams[('slip', seed)]
            alarms = alarms_for(arr, threshold, confirm)
            fault = (t >= FAULT[0]) & (t < FAULT[1])
            first = np.flatnonzero(alarms & fault)
            if len(first):
                hits += 1
                latencies.append(float(t[first[0]] - FAULT[0]))
            window_frac.append(float(np.mean(alarms[fault])))
            tail.append(float(np.mean(alarms[t >= FAULT[1] + 1.0])))
        latency_text = ', '.join(f'{x:.2f}' for x in latencies) if latencies else 'none'
        print(f'thr={threshold:.4f} confirm={confirm:2d} | '
              f'nominal alarm frac={np.mean(nominal):.4f} (max {np.max(nominal):.4f}) | '
              f'load alarm frac in window={np.mean(load):.4f} | '
              f'slip detected {hits}/{len(seeds)}, first-alarm latency s [{latency_text}], '
              f'alarm frac in window={np.mean(window_frac):.2f}, '
              f'alarm frac after fault+1 s={np.mean(tail):.4f}')


def long_nominal():
    print(f'\n##### long nominal runs: {LONG_DURATION:.0f} s, seeds {LONG_NOMINAL_SEEDS}')
    streams = {seed: record('nominal', seed, LONG_DURATION)[:2] for seed in LONG_NOMINAL_SEEDS}
    for threshold, confirm in CONFIGS:
        events, fractions, minutes = 0, [], 0.0
        for seed, (t, arr) in streams.items():
            alarms = alarms_for(arr, threshold, confirm)[t >= WARMUP_END]
            events += int(np.sum(alarms[1:] & ~alarms[:-1])) + (1 if alarms[0] else 0)
            fractions.append(float(np.mean(alarms)))
            minutes += (t[-1] - WARMUP_END) / 60.0
        print(f'thr={threshold:.4f} confirm={confirm:2d} | '
              f'false-alarm events={events} over {minutes:.1f} min = {events / minutes:.3f} per min | '
              f'alarm frac={np.mean(fractions):.5f}')


def main():
    print('simulation module:', sim.__file__)
    print('detector module:', YawRateSlipDetector.__module__)
    print(f'fault window = {FAULT} s, sensor period = {SENSOR_DT} s, '
          f'samples before t = {WARMUP_END} s excluded from false-alarm counts')
    calibration = build_streams(CALIBRATION_SEEDS)
    report('calibration seeds', CALIBRATION_SEEDS, calibration)
    heldout = build_streams(HELDOUT_SEEDS)
    report('held-out seeds', HELDOUT_SEEDS, heldout)
    long_nominal()


if __name__ == '__main__':
    main()
