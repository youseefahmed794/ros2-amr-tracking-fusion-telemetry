"""Exercise real ROS messages and controller validation, without Gazebo installed."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from ros2_amr_nmpc_telemetry import adaptive_controller_node as controller


def test_python_controller_rejects_and_normalizes_quaternion(monkeypatch):
    monkeypatch.setattr(controller, 'WEBSOCKETS_AVAILABLE', False)
    rclpy.init()
    node = controller.AdaptiveAMRControllerNode()
    try:
        msg = Odometry()
        msg.pose.pose.orientation.w = 0.0
        msg.header.stamp = node.get_clock().now().to_msg()
        node.odom_callback(msg)
        assert node.last_state_time is None
        assert node.state_gate.last is None
        msg.pose.pose.orientation.z = 2*math.sin(0.7)
        msg.pose.pose.orientation.w = 2*math.cos(0.7)
        node.odom_callback(msg)
        assert node.yaw == pytest.approx(1.4)
        accepted = node.last_state_time
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = math.nan
        node.odom_callback(msg)
        assert node.last_state_time == accepted
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_gazebo_command_gate_loss_stop_and_recovery(tmp_path):
    """Separate watchdog process, frozen ROS time, real DDS, no Gazebo claims."""
    script = Path(__file__).resolve().parents[1]/'scripts/command_watchdog_node.py'
    with (tmp_path/'watchdog.log').open('w') as log:
        process = subprocess.Popen([sys.executable, str(script), '--ros-args',
                                    '-p', 'use_sim_time:=true'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        probe = Node('command_gate_test_probe')
        publisher = probe.create_publisher(Twist, '/cmd_vel', 1)
        records = []
        probe.create_subscription(Twist, '/gazebo/cmd_vel',
                                  lambda msg: records.append((time.monotonic(), msg)), 10)

        def spin(seconds, command=None):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                assert process.poll() is None, (tmp_path/'watchdog.log').read_text()
                if command is not None:
                    publisher.publish(command)
                rclpy.spin_once(probe, timeout_sec=0.005)
                time.sleep(0.005)

        try:
            spin(2)
            assert records and records[-1][1].linear.x == 0, 'Missing startup zeros'
            moving = Twist(); moving.linear.x = 0.6; moving.angular.z = 0.2
            spin(0.3, moving)
            assert records[-1][1].linear.x == pytest.approx(0.6)
            stopped_publishing = time.monotonic()
            records.clear()
            spin(0.4)
            zero = [(t, m) for t, m in records if m.linear.x == m.angular.z == 0]
            assert zero, 'Command loss did not produce zero'
            assert zero[0][0]-stopped_publishing < 0.3
            assert records[-1][1].linear.x == records[-1][1].angular.z == 0
            spin(0.25, moving)
            assert records[-1][1].linear.x == pytest.approx(0.6), 'No recovery'
            invalid = Twist(); invalid.linear.x = math.nan
            spin(0.1, invalid)
            assert records[-1][1].linear.x == records[-1][1].angular.z == 0
            assert all(math.isfinite(m.linear.x) and math.isfinite(m.angular.z) for _, m in records)
        finally:
            probe.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
