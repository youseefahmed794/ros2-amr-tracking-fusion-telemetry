#!/usr/bin/env python3
import math
import json
import threading
import asyncio
import csv
import time
from pathlib import Path
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .core import Tracker, reference
from .freshness import TimestampGate

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AdaptiveAMRControllerNode(Node):
    """
    Nonlinear trajectory tracker with integral compensation,
    Anti-Windup Clamping, Actuator Saturation Limits, and Isolated Async WebSocket Telemetry.
    """

    def __init__(self):
        super().__init__('adaptive_controller_node')

        self.timing_output = self.declare_parameter("timing_output", "").value
        self.timing_samples = []
        self.executor_threads = self.declare_parameter("executor_threads", 1).value
        if self.executor_threads < 1:
            raise ValueError("executor_threads must be positive")

        # Declare explicit control & safety parameters
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('kx', 1.2)
        self.declare_parameter('ky', 2.5)
        self.declare_parameter('ktheta', 1.8)
        self.declare_parameter('ki_v', 0.25)
        self.declare_parameter('integral_limit', 0.4)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('omega_max', 2.0)
        self.declare_parameter('accel_max', 2.5)
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('ws_host', '127.0.0.1')
        self.declare_parameter('state_timeout_sec', 0.5)

        self.control_rate = self.get_parameter('control_rate_hz').value
        self.kx = self.get_parameter('kx').value
        self.ky = self.get_parameter('ky').value
        self.ktheta = self.get_parameter('ktheta').value
        self.ki_v = self.get_parameter('ki_v').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.accel_max = self.get_parameter('accel_max').value
        self.ws_port = self.get_parameter('ws_port').value

        self.ws_host = self.get_parameter('ws_host').value
        self.state_timeout = self.get_parameter('state_timeout_sec').value
        if not math.isfinite(self.control_rate) or self.control_rate <= 0 or not math.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError('Control rate and state timeout must be positive')
        self.tracker = Tracker(**{key: getattr(self, key) for key in (
            'kx', 'ky', 'ktheta', 'ki_v', 'integral_limit', 'v_max', 'omega_max', 'accel_max')})
        self.last_state_time = None
        self.state_gate = TimestampGate()
        self.ws_stop = threading.Event()

        # Serialize odometry and control; WebSocket I/O runs on its own thread.
        self.control_cb_group = MutuallyExclusiveCallbackGroup()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            reliable_qos,
            callback_group=self.control_cb_group
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', reliable_qos
        )

        # Internal Robot State & Control Memory
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.current_v = 0.0

        self.start_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()

        # Latest telemetry snapshot protected by lock for thread-safe async streaming
        self.telemetry_lock = threading.Lock()
        self.latest_telemetry = {}

        # Requested 50 Hz control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.measured_control_callback,
            callback_group=self.control_cb_group
        )

        # Start Isolated Background Thread for Asyncio WebSocket Server
        self.ws_clients: Set = set()
        if WEBSOCKETS_AVAILABLE:
            self.ws_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
            self.ws_thread.start()
            self.get_logger().info(f'Starting telemetry on ws://{self.ws_host}:{self.ws_port}')
        else:
            self.get_logger().warn('Python websockets package not found; running without WebSocket stream.')

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                  msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        if not all(math.isfinite(value) for value in values):
            return
        q = msg.pose.pose.orientation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-9 or not math.isfinite(norm):
            return
        if not self.state_gate.accept(stamp, now.nanoseconds):
            return
        self.last_state_time = rclpy.time.Time.from_msg(msg.header.stamp, clock_type=now.clock_type)
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        self.yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.current_v = msg.twist.twist.linear.x

    def get_reference_trajectory(self, t: float):
        return reference(t)

    def stop_motion(self):
        self.tracker.reset()
        self.cmd_pub.publish(Twist())
        with self.telemetry_lock:
            self.latest_telemetry = {'status': 'stopped', 'v_cmd': 0.0, 'omega_cmd': 0.0}

    def measured_control_callback(self):
        if not self.timing_output:
            return self.control_loop_callback()
        start = time.perf_counter_ns()
        try:
            self.control_loop_callback()
        finally:
            duration = time.perf_counter_ns()-start
            if len(self.timing_samples) < 100000:
                self.timing_samples.append((start, duration))

    def control_loop_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        # Dynamic dt jitter validation
        if self.last_state_time is None or not 0 <= (now-self.last_state_time).nanoseconds*1e-9 < self.state_timeout:
            self.stop_motion()
            return
        if dt <= 1e-5 or dt > 0.2:
            self.stop_motion()
            self.get_logger().warn(f'Control dt out of bounds ({dt:.4f}s); stopping.')
            return

        elapsed = (now - self.start_time).nanoseconds * 1e-9
        x_r, y_r, theta_r, v_r, omega_r = self.get_reference_trajectory(elapsed)

        dx, dy = x_r-self.pose_x, y_r-self.pose_y
        v_cmd, omega_cmd = self.tracker.step((self.pose_x, self.pose_y, self.yaw), elapsed, dt)

        # Publish Twist Command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(v_cmd)
        cmd_msg.angular.z = float(omega_cmd)
        self.cmd_pub.publish(cmd_msg)

        # Update a small telemetry snapshot under a shared lock
        tracking_error = math.hypot(dx, dy)
        with self.telemetry_lock:
            self.latest_telemetry = {
                'timestamp_sec': round(elapsed, 3),
                'dt_ms': round(dt * 1000.0, 2),
                'x': round(self.pose_x, 3),
                'y': round(self.pose_y, 3),
                'x_ref': round(x_r, 3),
                'y_ref': round(y_r, 3),
                'v_cmd': round(v_cmd, 3),
                'omega_cmd': round(omega_cmd, 3),
                'tracking_error_m': round(tracking_error, 4),
            }

    def _run_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(websocket):
            self.ws_clients.add(websocket)
            try:
                await websocket.wait_closed()
            finally:
                self.ws_clients.discard(websocket)

        async def broadcast_loop():
            while not self.ws_stop.is_set():
                if self.ws_clients:
                    with self.telemetry_lock:
                        snapshot = self.latest_telemetry
                    payload = json.dumps(snapshot) if snapshot else None
                    if payload:
                        await asyncio.gather(
                            *[asyncio.wait_for(client.send(payload), timeout=0.2) for client in list(self.ws_clients)],
                            return_exceptions=True
                        )
                await asyncio.sleep(0.05)  # 20 Hz Telemetry Stream

        async def main_ws():
            async with websockets.serve(handler, self.ws_host, self.ws_port, close_timeout=0.5):
                await broadcast_loop()

        try:
            loop.run_until_complete(main_ws())
        except OSError as exc:
            self.get_logger().error(f'WebSocket server failed: {exc}')
        finally:
            loop.close()

    def destroy_node(self):
        self.control_timer.cancel()
        if rclpy.ok():
            self.stop_motion()
        self.ws_stop.set()
        if hasattr(self, 'ws_thread'):
            self.ws_thread.join(timeout=3.0)
        if self.timing_output:
            with Path(self.timing_output).open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(('start_ns', 'duration_ns'))
                writer.writerows(self.timing_samples)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveAMRControllerNode()
    from rclpy.executors import SingleThreadedExecutor
    executor = (SingleThreadedExecutor() if node.executor_threads == 1
                else MultiThreadedExecutor(num_threads=node.executor_threads))
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

