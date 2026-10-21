#!/usr/bin/env python3
"""Multi-minute installed C++ ROS pipeline measurement, no truth fusion.

Uses controller ROS-clock stamp_sec and interpolates truth/filtered source stamps
onto reference sample times. Raw traces and bias status accompany success JSON.
Requires an isolated ROS domain; numerical simulator only, no physical robot.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def run(args):
    if not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Set isolated ROS_DOMAIN_ID without a physical robot')
    if args.output.exists() or args.output.with_suffix('.launch.log').exists():
        raise FileExistsError('Choose a fresh output path to preserve previous results')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    log_path=args.output.with_suffix('.launch.log')
    result={'passed':False,'duration_sec':args.duration,'seed':args.seed,'controller':'cpp'}
    with log_path.open('w') as log:
        process=subprocess.Popen(['ros2','launch','ros2_amr_nmpc_telemetry','amr_simulation.launch.py',
                                  f'seed:={args.seed}',f'scenario:={args.scenario}'],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init()
        probe=rclpy.create_node('long_localization_probe')
        truth,filtered,refs,bias,commands,status=[],[],[],[],[],[]
        def odometry(target,m):
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            p,q=m.pose.pose.position,m.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            target.append([stamp,p.x,p.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z])
        probe.create_subscription(Odometry,'/ground_truth',lambda m:odometry(truth,m),100)
        probe.create_subscription(Odometry,'/odometry/filtered',lambda m:odometry(filtered,m),100)
        def telemetry(m):
            v=json.loads(m.data)
            if 'stamp_sec' in v:
                refs.append([v['stamp_sec'],v['timestamp_sec'],v['x_ref'],v['y_ref'],v['x'],v['y']])
        probe.create_subscription(String,'/controller/telemetry_json',telemetry,100)
        probe.create_subscription(String,'/localization/gyro_bias',lambda m:bias.append(json.loads(m.data)),10)
        probe.create_subscription(String,'/simulation/status',lambda m:status.append(json.loads(m.data)),10)
        probe.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(time.monotonic()),100)
        try:
            deadline=time.monotonic()+args.duration+25
            while time.monotonic()<deadline:
                rclpy.spin_once(probe,timeout_sec=0.01)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited; see launch log')
                if status and status[-1]['elapsed_sec']>=args.duration:
                    break
            assert status and status[-1]['elapsed_sec']>=args.duration, 'Simulation did not complete duration'
            assert len(truth)>args.duration*70 and len(filtered)>args.duration*30 and len(refs)>args.duration*70
            assert probe.count_publishers('/cmd_vel')==1
            assert bias and all(v['enabled'] for v in bias)
            a,b,c=map(np.asarray,(truth,filtered,refs))
            valid=(c[:,0]>=max(a[0,0],b[0,0]))&(c[:,0]<=min(a[-1,0],b[-1,0]))
            c=c[valid]
            true_xy=np.column_stack([np.interp(c[:,0],a[:,0],a[:,i]) for i in (1,2)])
            est_xy=np.column_stack([np.interp(c[:,0],b[:,0],b[:,i]) for i in (1,2)])
            true_error=np.linalg.norm(true_xy-c[:,2:4],axis=1)
            estimated_error=np.linalg.norm(est_xy-c[:,2:4],axis=1)
            tail=c[:,0]>c[-1,0]-30
            assert np.isfinite(a).all() and np.isfinite(b).all() and np.isfinite(c).all()
            result.update(passed=True,scenario=args.scenario,
                          true_tracking_rmse_m=float(np.sqrt(np.mean(true_error**2))),
                          estimated_tracking_rmse_m=float(np.sqrt(np.mean(estimated_error**2))),
                          last_30s_true_tracking_rmse_m=float(np.sqrt(np.mean(true_error[tail]**2))),
                          last_30s_estimated_tracking_rmse_m=float(np.sqrt(np.mean(estimated_error[tail]**2))),
                          final_bias=bias[-1],command_receipt_hz=(len(commands)-1)/(commands[-1]-commands[0]),
                          truth_samples=len(truth),filtered_samples=len(filtered),reference_samples=len(refs),
                          note='passed covers stream integrity/completion, not an absolute pose guarantee; no tracking threshold hidden in this flag',
                          alignment='truth/filtered source stamps linearly interpolated onto controller ROS stamp_sec; <1-tick interpolation approximation')
            np.savez_compressed(args.output.with_suffix('.npz'),truth=a,filtered=b,reference=c,
                                true_error=true_error,estimated_error=estimated_error)
        except Exception as exc:
            result['error']=f'{type(exc).__name__}: {exc}'
            raise
        finally:
            args.output.with_suffix('.bias.json').write_text(json.dumps(bias,indent=2)+'\n')
            probe.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=240)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--scenario',choices=['nominal','slip','imu_dropout','odom_dropout','odom_delay','nonfinite','command_dropout'],default='nominal')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or args.duration<240:
        parser.error('Use duration >=240 seconds')
    run(args)
