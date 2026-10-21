#!/usr/bin/env python3
"""Run paired seeded comparisons and fault experiments without ROS."""
import argparse
import csv
import json
import platform
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_amr_nmpc_telemetry.simulation import CONTROLLERS, SCENARIOS, run_trial


def run(output, seeds, duration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    results, traces = [], {}
    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            for seed in seeds:
                data, metrics = run_trial(controller, scenario, seed, duration)
                results.append(metrics)
                if seed == seeds[0] and (scenario == 'nominal' or controller == CONTROLLERS[0]):
                    traces[f'{scenario}__{controller}'] = data
        print(f'Finished {scenario}', flush=True)
    with (output/'trials.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=results[0].keys())
        writer.writeheader(); writer.writerows(results)
    aggregates = []
    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            group = [row for row in results if row['scenario'] == scenario and row['controller'] == controller]
            rmse = np.array([row['tracking_rmse_m'] for row in group])
            aggregates.append({'scenario': scenario, 'controller': controller,
                               'trials': len(group), 'rmse_mean_m': float(rmse.mean()),
                               'rmse_std_m': float(rmse.std(ddof=1)) if len(group)>1 else 0.0,
                               'rmse_min_m': float(rmse.min()), 'rmse_max_m': float(rmse.max())})
    (output/'summary.json').write_text(json.dumps(aggregates, indent=2)+'\n')
    np.savez_compressed(output/'representative_traces.npz', **traces)
    (output/'manifest.json').write_text(json.dumps({
        'seeds': seeds, 'duration_sec': duration, 'fault_window_sec': [12, 16],
        'controllers': CONTROLLERS, 'scenarios': SCENARIOS, 'python': platform.python_version(),
        'numpy': np.__version__, 'wall_time_sec': time.monotonic()-started,
        'trace_columns': ['t','x','y','yaw','estimated_x','estimated_y','estimated_yaw',
                          'reference_x','reference_y','v_cmd','w_cmd','v_actual','w_actual',
                          'controller_stopped','base_watchdog'],
        'model': 'wheel actuator lag, acceleration bounds, traction, independent command watchdog',
        'limits': 'Python control core at 100 Hz; not Gazebo or measured C++ timing; no absolute pose correction',
    }, indent=2)+'\n')
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    colors = ('#147d92', '#d28a18', '#8859a3')
    for controller, color in zip(CONTROLLERS, colors):
        data = traces[f'nominal__{controller}']
        axes[0].plot(data[:,1], data[:,2], label=controller, color=color)
    axes[0].plot(data[:,7], data[:,8], 'k--', label='reference')
    axes[0].set(title='Nominal tracking: first seed', xlabel='X [m]', ylabel='Y [m]', aspect='equal')
    for i, scenario in enumerate(('nominal','load','slip')):
        rows = [a for a in aggregates if a['scenario'] == scenario]
        for j, (a, color) in enumerate(zip(rows, colors)):
            axes[1].bar(i+(j-1)*0.24, a['rmse_mean_m'], width=0.23,
                        yerr=a['rmse_std_m'], color=color, capsize=3)
    axes[1].set(xticks=range(3), xticklabels=['nominal','load','slip'], ylabel='RMSE [m]',
                title='Mean +/- sample SD across paired seeds')
    for scenario in ('imu_dropout','odom_delay','command_dropout'):
        data = traces[f'{scenario}__{CONTROLLERS[0]}']
        axes[2].plot(data[:,0], data[:,11], label=scenario)
    axes[2].axvspan(12,16,color='orange',alpha=0.15)
    axes[2].set(title='Actual base speed under faults', xlabel='Time [s]', ylabel='Speed [m/s]')
    for ax in (axes[0], axes[2]): ax.legend(fontsize=7)
    for ax in axes: ax.grid(alpha=0.2)
    fig.tight_layout(); fig.savefig(output/'comparison.png', dpi=180); plt.close(fig)
    lines = ['# Reproducible experiment results', '',
             f'{len(results)} trials; seeds: {seeds}; duration {duration:g} s per trial.', '',
             'Fault window: [12,16) seconds. All controllers share noise seeds, plant, initial pose and output limits.',
             'Pure Pursuit uses a fixed 0.6 s preview; gains were not optimized per scenario.', '',
             '| Scenario | Controller | Mean RMSE [m] | Sample SD [m] |',
             '|---|---|---:|---:|']
    lines += [f"| {a['scenario']} | {a['controller']} | {a['rmse_mean_m']:.4f} | {a['rmse_std_m']:.4f} |" for a in aggregates]
    lines += ['', '![Comparison](comparison.png)', '',
              'Fault scenarios intentionally interrupt tracking; RMSE measures the resulting degradation, not only tracking quality.',
              'Slip affects ground speed but not encoders; absolute localization drift is expected.',
              'The headless simulator models wheel dynamics, not contact physics, obstacles or Gazebo.',
              'Raw per-trial metrics: trials.csv. Selected first-seed traces: representative_traces.npz; column names: manifest.json.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('docs/experiments'))
    parser.add_argument('--seeds', type=int, nargs='+', default=[11,23,42,71,101])
    parser.add_argument('--duration', type=float, default=30)
    args = parser.parse_args()
    if args.duration <= 16 or len(set(args.seeds)) != len(args.seeds):
        parser.error('Use duration >16 seconds and unique seeds')
    run(args.output_dir, args.seeds, args.duration)
