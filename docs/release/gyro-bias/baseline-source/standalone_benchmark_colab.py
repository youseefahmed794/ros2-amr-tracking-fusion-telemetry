#!/usr/bin/env python3
"""Seeded velocity-fusion benchmark using the same core as the ROS nodes."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_amr_nmpc_telemetry.core import Tracker, VelocityEKF, reference, normalize_angle


def simulate(seed=42, duration=30.0):
    rng = np.random.default_rng(seed)
    dt = 0.02
    true = np.zeros(3)
    ekf = VelocityEKF()
    tracker = Tracker()
    rows = []
    for k in range(int(duration/dt)):
        t = k*dt
        ref = reference(t)
        v, w = tracker.step(ekf.x[:3, 0], t, dt)
        rows.append([t, *true, *ekf.x[:3, 0], ref[0], ref[1], v, w])
        # A speed loss observed by wheel velocity; unobserved wheel slip is a
        # different problem and cannot be corrected by velocity-only fusion.
        actual_v = v * (0.7 if 12 <= t < 16 else 1.0)
        true[0] += actual_v*math.cos(true[2])*dt
        true[1] += actual_v*math.sin(true[2])*dt
        true[2] = normalize_angle(true[2]+w*dt)
        ekf.update([actual_v+rng.normal(0, 0.03), w+rng.normal(0, 0.03)], (3, 4), ekf.R_odom)
        ekf.update([w+rng.normal(0, 0.01)], (4,), ekf.R_imu)
        ekf.predict(dt)
    data = np.asarray(rows)
    errors = np.linalg.norm(data[:, 1:3]-data[:, 7:9], axis=1)
    metrics = {
        'seed': seed, 'duration_sec': duration, 'dt_sec': dt,
        'tracking_rmse_m': float(np.sqrt(np.mean(errors**2))),
        'max_tracking_error_m': float(errors.max()),
        'localization_rmse_m': float(np.sqrt(np.mean(np.sum((data[:, 1:3]-data[:, 4:6])**2, axis=1)))),
        'peak_linear_command_m_s': float(np.abs(data[:, 9]).max()),
        'peak_angular_command_rad_s': float(np.abs(data[:, 10]).max()),
    }
    return data, errors, metrics


def run_benchmark(seed=42, output_dir=Path('.')):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    data, errors, metrics = simulate(seed)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(data[:, 7], data[:, 8], 'k--', label='Reference')
    axes[0].plot(data[:, 1], data[:, 2], label='True trajectory')
    axes[0].plot(data[:, 4], data[:, 5], alpha=0.6, label='Velocity EKF')
    axes[0].set(xlabel='X [m]', ylabel='Y [m]', title='Figure-eight tracking', aspect='equal')
    axes[1].plot(data[:, 0], errors, label=f"Position error; RMSE={metrics['tracking_rmse_m']:.3f} m")
    axes[1].axvspan(12, 16, color='orange', alpha=0.25, label='Observed speed loss')
    axes[1].set(xlabel='Time [s]', ylabel='Euclidean position error [m]', title='Tracking error')
    for ax in axes:
        ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir/'benchmark_results.png', dpi=200)
    plt.close(fig)
    (output_dir/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    np.savetxt(output_dir/'trajectory.csv', data, delimiter=',',
               header='time,x,y,yaw,estimated_x,estimated_y,estimated_yaw,reference_x,reference_y,v_cmd,omega_cmd', comments='')
    print(json.dumps(metrics, indent=2))
    return metrics


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output-dir', type=Path, default=Path('.'))
    args = parser.parse_args()
    run_benchmark(args.seed, args.output_dir)
