#!/usr/bin/env python3
"""Host-only Gazebo check; unverified until executed with installed ros_gz.

Starts its own headless launch, measures tracking, SIGSTOPs only its controller
PID (read from that launch log), checks independent gate/physical stopping, then
SIGCONTs it and checks recovery. Requires isolated ROS and Gazebo domains.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


def run(args):
    if args.output.exists() or args.output.with_suffix('.launch.log').exists():
        raise FileExistsError('Choose a fresh evidence path; existing results will be preserved')
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GZ_PARTITION'):
        raise RuntimeError('Set isolated ROS_DOMAIN_ID and GZ_PARTITION; no physical robot')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.launch.log')
    process = None
    stopped_pid = None
    probe = None
    result = {'passed': False, 'backend': 'gazebo', 'max_rmse_m': args.max_rmse,
              'limitations': 'ROS gate cannot protect against gate/bridge death; not hardware safety validation'}
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(['ros2','launch','ros2_amr_nmpc_telemetry',
                                        'amr_gazebo.launch.py','gui:=false','rviz:=false'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = rclpy.create_node('gazebo_release_probe')
            data = {key: [] for key in ('truth','wheel','imu','joint','clock','state','cmd','gate','error')}
            paths, parents = {}, {}
            def odom(key, msg):
                data[key].append((time.monotonic(), msg))
            for key, topic in [('truth','/ground_truth'),('wheel','/wheel_odom'),('state','/odometry/filtered')]:
                probe.create_subscription(Odometry, topic, lambda m,k=key: odom(k,m), 50)
            probe.create_subscription(Imu, '/imu/data', lambda m: odom('imu',m), qos_profile_sensor_data)
            probe.create_subscription(JointState, '/joint_states', lambda m: odom('joint',m), 50)
            probe.create_subscription(Clock, '/clock', lambda m: data['clock'].append(
                (time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)), qos_profile_sensor_data)
            for key, topic in [('cmd','/cmd_vel'),('gate','/gazebo/cmd_vel')]:
                probe.create_subscription(Twist, topic, lambda m,k=key: odom(k,m), 50)
            def telemetry(msg):
                value = json.loads(msg.data)
                if 'x_ref' in value and data['truth']:
                    received, truth = data['truth'][-1]
                    if time.monotonic()-received < 0.1:
                        p = truth.pose.pose.position
                        data['error'].append(math.hypot(p.x-value['x_ref'],p.y-value['y_ref']))
            probe.create_subscription(String, '/controller/telemetry_json', telemetry, 50)
            for name in ('reference','true','estimated'):
                probe.create_subscription(RosPath, '/simulation/'+name+'_path',
                                          lambda m,k=name: paths.update({k:m.header.frame_id}), 10)
            def tf(msg):
                for t in msg.transforms:
                    parents[t.child_frame_id] = t.header.frame_id
            probe.create_subscription(TFMessage, '/tf', tf, 100)
            probe.create_subscription(TFMessage, '/tf_static', tf,
                                      QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            def pump(seconds):
                end = time.monotonic()+seconds
                while time.monotonic()<end:
                    if process.poll() is not None:
                        raise RuntimeError('Gazebo launch exited; inspect launch log')
                    rclpy.spin_once(probe, timeout_sec=0.01)
            deadline = time.monotonic()+40
            required = ('truth','wheel','imu','joint','clock','state','cmd','gate')
            while not all(data[k] for k in required) and time.monotonic()<deadline:
                pump(0.1)
            assert all(data[k] for k in required), f'Missing streams: {[k for k in required if not data[k]]}'
            sim_start = data['clock'][-1][1]
            data['error'].clear()
            deadline = time.monotonic()+90
            while data['clock'][-1][1]-sim_start<20 and time.monotonic()<deadline:
                pump(0.1)
            assert data['clock'][-1][1]-sim_start>=20, 'Simulated clock did not advance 20 seconds'
            assert probe.count_publishers('/cmd_vel')==1, 'Unexpected controller publisher count'
            assert len(data['error'])>100, 'Missing ground-truth/reference tracking measurements'
            rmse = math.sqrt(sum(e*e for e in data['error'])/len(data['error']))
            assert math.isfinite(rmse) and rmse<args.max_rmse, f'Tracking RMSE {rmse} exceeds criterion'
            for key in ('truth','wheel','state'):
                assert all(m.header.frame_id=='odom' and m.child_frame_id=='base_link'
                           for _,m in data[key]), f'Unexpected {key} frames'
            assert paths == {k:'odom' for k in ('reference','true','estimated')}, paths
            expected = {'base_footprint':'odom','base_link':'base_footprint',
                        'left_wheel':'base_link','right_wheel':'base_link','caster':'base_link'}
            assert all(parents.get(k)==v for k,v in expected.items()), parents
            assert all(math.isfinite(m.angular_velocity.z) and m.header.frame_id for _,m in data['imu'])
            assert len({tuple(m.position) for _,m in data['joint']})>10, 'No wheel motion'
            positions = [m.pose.pose.position for _,m in data['truth']]
            assert max(math.hypot(p.x-positions[0].x,p.y-positions[0].y) for p in positions)>0.3, 'No base motion'
            result.update(tracking_rmse_m=rmse, tracking_samples=len(data['error']),
                          simulated_seconds=data['clock'][-1][1]-sim_start,
                          samples={k:len(data[k]) for k in required}, tf_parents=parents, path_frames=paths,
                          imu_frames=sorted({m.header.frame_id for _,m in data['imu']}))
            # Locate only the controller created by this specific launch.
            matches = re.findall(r'\[adaptive_controller_cpp_node-\d+\]: process started with pid \[(\d+)\]', log_path.read_text())
            assert len(matches)==1, 'Cannot uniquely identify launched controller PID'
            pid = int(matches[0])
            cmdline = Path(f'/proc/{pid}/cmdline').read_bytes()
            assert b'adaptive_controller_cpp_node' in cmdline, 'Controller process identity mismatch'
            assert any(abs(m.linear.x)>0.05 for t,m in data['cmd'] if t>time.monotonic()-1), 'Not moving before command-loss injection'
            os.kill(pid, signal.SIGSTOP)
            stopped_pid = pid
            lost_at = time.monotonic()
            pump(5)
            zeros = [(t,m) for t,m in data['gate'] if t>lost_at+0.1 and m.linear.x==m.angular.z==0]
            assert zeros and zeros[0][0]-lost_at<0.5, 'Gate command-loss stop delayed/missing'
            settled = [m for t,m in data['truth'] if t>lost_at+3]
            assert settled, 'Missing physical stop samples'
            assert all(abs(m.twist.twist.linear.x)<0.02 and abs(m.twist.twist.angular.z)<0.05
                       for m in settled), 'Gazebo base did not settle after command loss'
            os.kill(pid, signal.SIGCONT)
            stopped_pid = None
            resumed_at = time.monotonic()
            pump(4)
            assert any(abs(m.linear.x)>0.05 for t,m in data['cmd'] if t>resumed_at+1), 'Controller did not resume'
            assert any(abs(m.twist.twist.linear.x)>0.03 for t,m in data['truth'] if t>resumed_at+1), 'Base did not recover'
            result.update(passed=True, controller_stall_gate_stop_sec=zeros[0][0]-lost_at,
                          physical_stop=True, recovery=True)
        except Exception as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            if stopped_pid:
                try: os.kill(stopped_pid, signal.SIGCONT)
                except ProcessLookupError: pass
            if probe:
                probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process:
                try: os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError: pass
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-rmse', type=float, default=1.0,
                        help='Host acceptance threshold in metres; not a published benchmark result')
    args = parser.parse_args()
    if not math.isfinite(args.max_rmse) or args.max_rmse<=0:
        parser.error('--max-rmse must be finite and positive')
    run(args)
