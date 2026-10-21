#!/usr/bin/env python3
"""Exercise installed C++ launch, EKF, and WebSocket bridge in an isolated ROS domain.

Requires a built/sourced workspace and websockets. Never run in a robot's ROS domain.
"""
import argparse
import asyncio
import json
import math
import shutil
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from websockets import connect


PAYLOAD_FIELDS = {'timestamp_sec', 'dt_ms', 'x', 'y', 'x_ref', 'y_ref',
                  'v_cmd', 'omega_cmd', 'tracking_error_m'}


def tracking_payload(payload):
    return (PAYLOAD_FIELDS <= payload.keys()
            and all(isinstance(payload[key], (int, float))
                    and not isinstance(payload[key], bool)
                    and math.isfinite(payload[key]) for key in PAYLOAD_FIELDS)
            and abs(payload['v_cmd']) <= 1.5 and abs(payload['omega_cmd']) <= 2.0)


async def receive_until(websocket, predicate, timeout=3.0):
    deadline = asyncio.get_running_loop().time()+timeout
    while True:
        remaining = deadline-asyncio.get_running_loop().time()
        if remaining <= 0:
            raise AssertionError('Expected WebSocket payload not received before deadline')
        payload = json.loads(await asyncio.wait_for(websocket.recv(), timeout=remaining))
        assert isinstance(payload, dict), payload
        if predicate(payload):
            return payload


async def verify(controller='cpp', output=None):
    if output and (output.exists() or output.with_suffix('.launch.log').exists()):
        raise FileExistsError('Choose a fresh evidence path; existing results will be preserved')
    if 'ROS_DOMAIN_ID' not in os.environ:
        raise RuntimeError('Set a dedicated ROS_DOMAIN_ID before running this smoke test')
    with tempfile.TemporaryDirectory(prefix='amr-hybrid-smoke-') as directory:
        log_path = Path(directory)/'launch.log'
        with log_path.open('w') as log:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'ros2_amr_nmpc_telemetry', 'amr_closed_loop.launch.py',
                 f'controller:={controller}'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = Node('hybrid_smoke_probe')
            odom = probe.create_publisher(Odometry, '/wheel_odom', 10)
            imu = probe.create_publisher(Imu, '/imu/data', 10)
            commands, states = [], []
            probe.create_subscription(Twist, '/cmd_vel',
                                      lambda msg: commands.append((time.monotonic(), msg)), 10)
            probe.create_subscription(Odometry, '/odometry/filtered', states.append, 10)

            async def pump(seconds, sensors=True):
                end, next_sensor = time.monotonic()+seconds, 0.0
                while time.monotonic() < end:
                    if launch.poll() is not None:
                        raise RuntimeError(log_path.read_text())
                    if sensors and time.monotonic() >= next_sensor:
                        wheel, inertial = Odometry(), Imu()
                        wheel.header.stamp = probe.get_clock().now().to_msg()
                        inertial.header.stamp = wheel.header.stamp
                        odom.publish(wheel)
                        imu.publish(inertial)
                        next_sensor = time.monotonic()+0.02
                    rclpy.spin_once(probe, timeout_sec=0)
                    await asyncio.sleep(0.002)

            try:
                await pump(5)
                assert states, 'No EKF output through process boundary'
                assert commands and any(abs(msg.linear.x) > 0 for _, msg in commands)
                assert probe.count_publishers('/cmd_vel') == 1, 'Multiple controllers are active'
                assert all(abs(msg.linear.x) <= 1.5 and abs(msg.angular.z) <= 2
                           for _, msg in commands)
                # Both servers must survive a clean disconnect and a new connection.
                async with connect('ws://127.0.0.1:8765', open_timeout=3) as websocket:
                    task = asyncio.create_task(pump(1))
                    try:
                        first = await receive_until(websocket, tracking_payload)
                    finally:
                        await task
                await pump(0.2)
                async with connect('ws://127.0.0.1:8765', open_timeout=3) as websocket:
                    task = asyncio.create_task(pump(1))
                    try:
                        reconnected = await receive_until(websocket, tracking_payload)
                    finally:
                        await task
                    assert reconnected['timestamp_sec'] > first['timestamp_sec'], 'Frozen telemetry after reconnect'
                    await pump(1.5, sensors=False)
                    latest = commands[-1][1]
                    assert latest.linear.x == latest.angular.z == 0.0, 'No sensor-loss stop'
                    stopped = await receive_until(websocket, lambda p: p.get('status') == 'stopped')
                    assert stopped['v_cmd'] == stopped['omega_cmd'] == 0, stopped
                    # Drain queued stops until genuinely newer active telemetry arrives.
                    task = asyncio.create_task(pump(1.5, sensors=True))
                    try:
                        recovered = await receive_until(websocket, lambda p: tracking_payload(p)
                            and p['timestamp_sec'] > reconnected['timestamp_sec']+1.0)
                    finally:
                        await task
                    assert any(abs(msg.linear.x)>0 for t,msg in commands
                               if t>commands[-1][0]-0.3), 'No sensor recovery'
                # Verify a second reconnect after the complete fault/recovery cycle.
                async with connect('ws://127.0.0.1:8765', open_timeout=3) as websocket:
                    task = asyncio.create_task(pump(0.5))
                    try:
                        final = await receive_until(websocket, lambda p: tracking_payload(p)
                            and p['timestamp_sec'] >= recovered['timestamp_sec'])
                    finally:
                        await task
                observed = [t for t, _ in commands if commands[0][0]+1 <= t <= commands[-1][0]-1]
                hz = (len(observed)-1)/(observed[-1]-observed[0])
                result = dict(controller=controller, passed=True, payload_delivery=True,
                              disconnect_reconnect=True, sensor_stop=True,
                              sensor_recovery_payload=True, post_fault_reconnect=True,
                              observed_cmd_vel_hz=hz, first_payload=first,
                              reconnected_payload=reconnected, stopped_payload=stopped,
                              recovered_payload=recovered, final_payload=final)
                print(json.dumps(result), flush=True)
            finally:
                probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=5)
                if output:
                    output.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(log_path, output.with_suffix('.launch.log'))
                if launch.returncode not in (0, -signal.SIGINT):
                    print(log_path.read_text())
            if output:
                output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=['cpp', 'python'], default='cpp')
    parser.add_argument('--output', type=Path, help='Write success JSON and retain launch log beside it')
    args = parser.parse_args()
    asyncio.run(verify(args.controller, args.output))
