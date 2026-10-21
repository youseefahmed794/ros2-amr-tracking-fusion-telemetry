#!/usr/bin/env python3
"""Paired multi-minute bias ablation, slip and repeated sensor-fault validation.

Ground truth is used ONLY by the plant and evaluation, never estimator input.
Output directory must be new. All trials, failures and representative traces are
retained. Acceptance checks cover finite/bounded output and fault stop/recovery;
slip/pose errors are measurements, not silently classified as successful tracking.
"""
import argparse
import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_amr_nmpc_telemetry.simulation import PlantConfig, run_trial


PROFILES = {
    'nominal': ('nominal', (), 0.005),
    'load': ('load', ((12,16),), 0.005),
    'slip_transient': ('slip', ((12,16),), 0.005),
    'slip_sustained': ('slip', ((60,240),), 0.005),
    'imu_dropout': ('imu_dropout', ((60,64),(180,184)), 0.005),
    'odom_dropout': ('odom_dropout', ((60,64),(180,184)), 0.005),
    'odom_delay': ('odom_delay', ((60,64),(180,184)), 0.005),
    'nonfinite': ('nonfinite', ((60,64),(180,184)), 0.005),
    'command_dropout': ('command_dropout', ((60,64),(180,184)), 0.005),
    'negative_bias': ('nominal', (), -0.02),
    'positive_bias': ('nominal', (), 0.03),
}
TRACE_COLUMNS = ['t','x','y','yaw','estimated_x','estimated_y','estimated_yaw',
                 'reference_x','reference_y','v_cmd','w_cmd','v_actual','w_actual',
                 'controller_stopped','base_watchdog','gyro_bias_rad_s','gyro_bias_variance']


def run(args):
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError('Choose a new output directory; previous results are preserved')
    args.output.mkdir(parents=True,exist_ok=True)
    started=time.monotonic()
    results, checks, traces = [], [], {}
    # Write each trial immediately, so partial/failed experiments remain inspectable.
    with (args.output/'trials.csv').open('w',newline='') as stream:
        writer=None
        for profile,(scenario,windows,bias) in PROFILES.items():
            for seed in args.seeds:
                for enabled in (False,True):
                    data,metrics=run_trial(scenario=scenario,seed=seed,duration=args.duration,
                                           estimate_gyro_bias=enabled,fault_windows=windows,
                                           plant_config=replace(PlantConfig(),imu_bias=bias))
                    metrics['profile']=profile
                    if writer is None:
                        writer=csv.DictWriter(stream,fieldnames=metrics.keys());writer.writeheader()
                    writer.writerow(metrics);stream.flush();results.append(metrics)
                    trial=dict(profile=profile,seed=seed,estimate_gyro_bias=enabled,
                               finite=bool(np.isfinite(data).all()),
                               bounded_commands=bool(np.abs(data[:,9]).max()<=1.5 and np.abs(data[:,10]).max()<=2),
                               nonnegative_bias_variance=bool(np.all(data[:,16]>=-1e-12)),fault_windows=[])
                    if scenario in ('imu_dropout','odom_dropout','odom_delay','nonfinite','command_dropout'):
                        for start,end in windows:
                            settled=(data[:,0]>start+2)&(data[:,0]<end-0.2)
                            after=(data[:,0]>end+1)&(data[:,0]<end+3)
                            stops=(data[:,0]>=start)&(data[:,0]<end)&(data[:,13 if scenario!='command_dropout' else 14]>0)
                            entry=dict(start=start,end=end,
                                       stop_delay_sec=float(data[stops,0][0]-start) if stops.any() else None,
                                       settled_linear_speed_m_s=float(np.abs(data[settled,11]).max()),
                                       settled_angular_speed_rad_s=float(np.abs(data[settled,12]).max()),
                                       resumed=bool(np.any(np.abs(data[after,9])>0.05)),
                                       zero_commands=bool(np.all(data[settled,9:11]==0)) if scenario!='command_dropout' else None)
                            entry['passed']=bool(entry['stop_delay_sec'] is not None and
                                                  entry['stop_delay_sec']<(0.23 if scenario=='command_dropout' else 1.1)
                                                  and entry['settled_linear_speed_m_s']<0.02
                                                  and entry['settled_angular_speed_rad_s']<0.02
                                                  and entry['resumed'] and entry['zero_commands'] is not False)
                            trial['fault_windows'].append(entry)
                    trial['passed']=all(trial[k] for k in ('finite','bounded_commands','nonnegative_bias_variance')) and all(x['passed'] for x in trial['fault_windows'])
                    checks.append(trial)
                    if seed==args.seeds[0]:
                        traces[f'{profile}__{"bias" if enabled else "baseline"}']=data
                print(f'{profile} seed={seed}: both estimators completed',flush=True)
    (args.output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    np.savez_compressed(args.output/'traces.npz',**traces)
    aggregates=[]
    for profile in PROFILES:
        for enabled in (False,True):
            rows=[r for r in results if r['profile']==profile and r['estimate_gyro_bias']==enabled]
            aggregate=dict(profile=profile,estimate_gyro_bias=enabled,trials=len(rows))
            for key in ('true_tracking_rmse_m','estimated_tracking_rmse_m',
                        'last_30s_true_tracking_rmse_m','last_30s_estimated_tracking_rmse_m',
                        'final_heading_error_est_minus_true_rad','final_gyro_bias_rad_s'):
                values=np.array([r[key] for r in rows])
                aggregate[key]=dict(mean=float(values.mean()),sd=float(values.std(ddof=1)) if len(values)>1 else 0.0,
                                    minimum=float(values.min()),maximum=float(values.max()))
            aggregates.append(aggregate)
    (args.output/'summary.json').write_text(json.dumps(aggregates,indent=2)+'\n')
    (args.output/'manifest.json').write_text(json.dumps(dict(
        trials=len(results),duration_sec=args.duration,dt_sec=0.01,sensor_rate_hz=50,
        seeds=args.seeds,profiles=PROFILES,trace_columns=TRACE_COLUMNS,
        controller='lyapunov_integral; identical gains/limits',
        baseline='bias state fixed at zero; numerically checked against archived five-state EKF',
        estimator='six-state EKF, gyro=w+b, wheel yaw=w, q_bias=1e-8, P_bias_initial=0.01',
        pair_design='same seeds, sensor draw counts, initial state, noise/bias and plant; trajectory differs under feedback',
        observability='wheel yaw must be unbiased and non-slipping; absolute pose/yaw unobservable; sustained slip can corrupt bias',
        limits='Offline model, not runtime timing, Gazebo or hardware. Covariance is conditional on model assumptions.',
        python=platform.python_version(),numpy=np.__version__,wall_time_sec=time.monotonic()-started,
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (Path(__file__),Path('ros2_amr_nmpc_telemetry/core.py'),Path('ros2_amr_nmpc_telemetry/simulation.py'))}
    ),indent=2)+'\n')
    lines=['# Paired multi-minute gyro-bias validation','',f'{len(results)} trials; {args.duration:g} simulated seconds each; paired seeds {args.seeds}.',
           '', 'Ground truth is evaluation-only. Default injected bias stays +0.005 rad/s; signed-bias stress cases use -0.02/+0.03 rad/s.',
           'No per-scenario tuning. Original artifacts remain untouched. All failures and unfavorable pose errors are retained.',
           '', '| Profile | Bias estimation | Last 30 s TRUE RMSE mean ± SD [m] | Last 30 s ESTIMATED RMSE mean ± SD [m] | Final bias mean [rad/s] |',
           '|---|---|---:|---:|---:|']
    for a in aggregates:
        true=a['last_30s_true_tracking_rmse_m'];estimated=a['last_30s_estimated_tracking_rmse_m']
        lines.append(f"| {a['profile']} | {a['estimate_gyro_bias']} | {true['mean']:.4f} ± {true['sd']:.4f} | {estimated['mean']:.4f} ± {estimated['sd']:.4f} | {a['final_gyro_bias_rad_s']['mean']:.6f} |")
    lines += ['',f"Numerical finite/command/fault-stop/recovery checks: {sum(c['passed'] for c in checks)}/{len(checks)} trials passed.",
              'This check count is not a declaration of acceptable tracking in slip/fault cases. See true-pose errors above.',
              'Wheel slip and wheel scale/yaw bias are not independently separable from gyro bias with these sensors.',
              'No absolute pose correction is provided. Estimated tracking error can remain small while true-pose tracking is poor.',
              '', '![True/estimated error and learned bias](drift.png)']
    (args.output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(13,10))
    for j,profile in enumerate(('nominal','slip_sustained')):
        for enabled in (False,True):
            data=traces[f'{profile}__{"bias" if enabled else "baseline"}']
            label='bias EKF' if enabled else 'old model'
            axes[0,j].plot(data[:,0],np.linalg.norm(data[:,1:3]-data[:,7:9],axis=1),label=label)
            axes[1,j].plot(data[:,0],np.linalg.norm(data[:,4:6]-data[:,7:9],axis=1),label=label)
            axes[2,j].plot(data[:,0],data[:,15],label=label)
        axes[0,j].set_title(profile)
        axes[2,j].axhline(0.005,color='black',linestyle='--',label='injected bias (evaluation only)')
        for i in range(3): axes[i,j].grid(alpha=.2);axes[i,j].legend(fontsize=8)
    for ax,label in zip(axes[:,0],('True tracking error [m]','Estimated tracking error [m]','Estimated bias [rad/s]')):
        ax.set_ylabel(label)
    for ax in axes[-1,:]: ax.set_xlabel('Simulated time [s]')
    fig.tight_layout();fig.savefig(args.output/'drift.png',dpi=160);plt.close(fig)
    if not all(c['passed'] for c in checks):
        raise AssertionError('One or more numerical/fault checks failed; results preserved')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('docs/release/gyro-bias/long-runs'))
    parser.add_argument('--duration',type=float,default=240)
    parser.add_argument('--seeds',type=int,nargs='+',default=[11,23,42,71,101])
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<240 or len(set(args.seeds))!=len(args.seeds) or len(args.seeds)<2:
        parser.error('Use duration >=240 s and at least two distinct paired seeds')
    run(args)
