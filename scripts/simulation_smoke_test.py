#!/usr/bin/env python3
"""Cross-process dynamics and fault validation; use an isolated ROS domain."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy


def run(scenario, output, controller="cpp"):
    if 'ROS_DOMAIN_ID' not in os.environ:
        raise RuntimeError('Choose a dedicated ROS_DOMAIN_ID without a robot')
    with tempfile.TemporaryDirectory(prefix='amr-dynamics-test-') as directory:
        with (Path(directory)/'launch.log').open('w') as log:
            process = subprocess.Popen(['ros2','launch','ros2_amr_nmpc_telemetry',
                                        'amr_simulation.launch.py',f'scenario:={scenario}', f'controller:={controller}'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = Node('simulation_test_probe')
            records, commands, paths, statuses, states = [], [], [], [], []
            probe.create_subscription(Odometry, '/ground_truth', lambda m: records.append(
                (time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x)),10)
            probe.create_subscription(Twist, '/cmd_vel', lambda m: commands.append(
                (time.monotonic(),m.linear.x,m.angular.z)),10)
            probe.create_subscription(String, '/simulation/status', lambda m: statuses.append(
                (time.monotonic(),json.loads(m.data))),10)
            probe.create_subscription(Odometry, '/odometry/filtered', lambda m: states.append(m),10)
            probe.create_subscription(RosPath, '/simulation/reference_path', lambda m: paths.append(len(m.poses)),10)
            transforms, path_frames, joint_positions = {}, {}, []
            def remember_tf(msg):
                for transform in msg.transforms:
                    transforms[transform.child_frame_id] = transform.header.frame_id
            probe.create_subscription(TFMessage, '/tf', remember_tf, 100)
            probe.create_subscription(TFMessage, '/tf_static', remember_tf,
                                      QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            probe.create_subscription(JointState, '/joint_states',
                                      lambda m: joint_positions.append(tuple(m.position)), 10)
            for name in ('reference', 'true', 'estimated'):
                probe.create_subscription(RosPath, '/simulation/'+name+'_path',
                                          lambda m, key=name: path_frames.update({key:m.header.frame_id}), 10)
            try:
                # Wait on simulator time, not startup wall time, for fault alignment.
                deadline = time.monotonic()+40
                target = 18 if scenario != 'nominal' else 6
                while time.monotonic() < deadline:
                    rclpy.spin_once(probe, timeout_sec=0.01)
                    if process.poll() is not None:
                        raise RuntimeError((Path(directory)/'launch.log').read_text())
                    if statuses and statuses[-1][1]['elapsed_sec'] > target:
                        break
                assert statuses and statuses[-1][1]['elapsed_sec'] > target, 'Simulator did not advance'
                assert records and states and paths, 'Missing truth, EKF or reference path'
                assert path_frames == {name:'odom' for name in ('reference','true','estimated')}, 'Missing paths or wrong path frame'
                expected_tf = {'base_footprint':'odom', 'base_link':'base_footprint',
                               'caster':'base_link', 'left_wheel':'base_link', 'right_wheel':'base_link'}
                assert all(transforms.get(child)==parent for child,parent in expected_tf.items()), transforms
                assert len(set(joint_positions))>1, 'Wheel joint positions did not change'
                assert all(m.header.frame_id=='odom' and m.child_frame_id=='base_link' for m in states), 'Wrong estimator frames'
                assert probe.count_publishers('/cmd_vel') == 1, 'Competing controllers'
                assert any(abs(v)>0.1 for _,v,w in commands), 'No motion command'
                assert max(abs(v) for _,v,w in commands) <= 1.5
                assert max(abs(w) for _,v,w in commands) <= 2.0
                result = {'scenario':scenario,'controller':controller,'passed':True,'truth_samples':len(records),
                          'state_samples':len(states),'command_samples':len(commands), 'tf_parents':transforms,
                          'path_frames':path_frames, 'wheel_joint_motion':True}
                if scenario != 'nominal':
                    # Interpolate simulator elapsed time onto command/truth receipt times.
                    origin = sum(t-s['elapsed_sec'] for t,s in statuses[-20:])/min(20,len(statuses))
                    window = [(t,v,w) for t,v,w in commands if 13.4 < t-origin < 15.8]
                    assert window, 'Missing samples in fault window'
                    speed = [abs(v) for t,x,y,v in records if 14 < t-origin < 15.8]
                    assert speed and max(speed)<0.02, 'Base did not settle to rest during fault'
                    if scenario != 'command_dropout':
                        assert all(v==0 and w==0 for _,v,w in window), 'Sensor fault did not stop controller'
                    else:
                        assert any(s['watchdog'] for _,s in statuses if 12.5 < s['elapsed_sec'] < 15.8)
                    assert any(abs(v)>0.05 for t,v,w in commands if t-origin>17), 'No recovery'
                    result['fault_window_peak_speed_m_s'] = max(speed)
                output.parent.mkdir(parents=True,exist_ok=True)
                output.write_text(json.dumps(result,indent=2)+'\n')
                print(json.dumps(result),flush=True)
            except Exception:
                print((Path(directory)/'launch.log').read_text(),flush=True)
                raise
            finally:
                probe.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=['nominal','imu_dropout','odom_dropout','odom_delay','nonfinite','command_dropout'],default='nominal')
    parser.add_argument('--output',type=Path,default=Path('/tmp/amr-simulation-test.json'))
    parser.add_argument('--controller',choices=['cpp','python'],default='cpp')
    args=parser.parse_args();run(args.scenario,args.output,args.controller)
