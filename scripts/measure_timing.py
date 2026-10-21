#!/usr/bin/env python3
"""Paired ROS/EKF controller timing. Isolated domain required; no physical robot.

Records bounded callback start/duration samples in the controller process; file
I/O and statistics run after shutdown. cmd_vel frequency is independently measured
at this subscriber. Warmup is excluded. No WebSocket client is connected.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def measure(args):
    if 'ROS_DOMAIN_ID' not in os.environ:
        raise RuntimeError('Set an isolated ROS_DOMAIN_ID')
    args.output.mkdir(parents=True, exist_ok=True)
    threads = args.threads if args.controller == 'python' else 2
    label = f'{args.controller}_{args.rate:g}hz_t{threads}'
    samples_path = (args.output/(label+'.csv')).resolve()
    with (args.output/(label+'.log')).open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'ros2_amr_nmpc_telemetry', 'amr_closed_loop.launch.py',
            f'controller:={args.controller}', f'control_rate_hz:={float(args.rate)}',
            f'executor_threads:={args.threads}', f'timing_output:={samples_path}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        probe = rclpy.create_node('timing_probe')
        odom = probe.create_publisher(Odometry, '/wheel_odom', 10)
        imu = probe.create_publisher(Imu, '/imu/data', 10)
        commands = []
        states = []
        probe.create_subscription(Twist, '/cmd_vel', lambda msg: commands.append(time.monotonic_ns()), 100)
        probe.create_subscription(Odometry, '/odometry/filtered', lambda msg: states.append(time.monotonic_ns()), 100)

        def sensors():
            wheel, inertial = Odometry(), Imu()
            wheel.header.stamp = probe.get_clock().now().to_msg()
            inertial.header.stamp = wheel.header.stamp
            odom.publish(wheel)
            imu.publish(inertial)

        probe.create_timer(0.02, sensors)
        try:
            # Wait for real state/command transport before starting warmup.
            deadline = time.monotonic()+10
            while (not states or not commands) and time.monotonic()<deadline:
                rclpy.spin_once(probe, timeout_sec=0.01)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited; inspect log')
            if not states or not commands:
                raise RuntimeError('No state or command transport')
            start = time.monotonic_ns()+int(args.warmup*1e9)
            end = start+int(args.duration*1e9)
            while time.monotonic_ns()<end:
                rclpy.spin_once(probe, timeout_sec=0.005)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited; inspect log')
            if probe.count_publishers('/cmd_vel') != 1:
                raise RuntimeError('Expected exactly one command publisher')
        finally:
            probe.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    with samples_path.open() as stream:
        rows = list(csv.DictReader(stream))
    samples = np.array([(int(row['start_ns']), int(row['duration_ns'])) for row in rows], dtype=np.int64)
    samples = samples[(samples[:, 0]>=start) & (samples[:, 0]<end)]
    if len(samples)<2:
        raise RuntimeError('Too few callback samples')
    intervals = np.diff(samples[:, 0])*1e-6
    durations = samples[:, 1]*1e-6
    command_times = [t for t in commands if start<=t<end]
    state_times = [t for t in states if start<=t<end]
    result = dict(controller=args.controller, requested_hz=args.rate,
                  executor_threads=args.threads if args.controller=='python' else 2,
                  duration_sec=args.duration, warmup_sec=args.warmup,
                  openblas_num_threads=os.environ.get('OPENBLAS_NUM_THREADS', 'unset'),
                  condition='50Hz wheel+IMU -> separate EKF process; no WebSocket clients; no plant',
                  callback_samples=len(samples), callback_hz=1000/float(intervals.mean()),
                  cmd_vel_hz=(len(command_times)-1)*1e9/(command_times[-1]-command_times[0]),
                  ekf_hz=(len(state_times)-1)*1e9/(state_times[-1]-state_times[0]))
    for key, values in [('callback_duration_ms',durations),('interval_ms',intervals),
                        ('absolute_interval_jitter_ms',abs(intervals-1000/args.rate))]:
        result[key] = {f'p{p}':float(np.percentile(values,p)) for p in (50,95,99,100)}
    (args.output/(label+'.json')).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=['cpp','python'], required=True)
    parser.add_argument('--rate', type=float, default=50)
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--warmup', type=float, default=3)
    parser.add_argument('--output', type=Path, default=Path('docs/release/timing'))
    args = parser.parse_args()
    if args.rate<=0 or args.duration<=0 or args.warmup<0 or args.threads<1:
        parser.error('rate/duration/threads must be positive; warmup nonnegative')
    measure(args)
