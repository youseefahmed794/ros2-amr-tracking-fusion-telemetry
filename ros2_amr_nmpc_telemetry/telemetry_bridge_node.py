"""Forward C++ diagnostic JSON to a read-only asyncio WebSocket server."""
import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryBridgeNode(Node):
    def __init__(self):
        super().__init__('telemetry_bridge_node')
        self.host = self.declare_parameter('ws_host', '127.0.0.1').value
        self.port = self.declare_parameter('ws_port', 8765).value
        self.timeout = self.declare_parameter('telemetry_timeout_sec', 0.5).value
        if not 0 < self.port < 65536 or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Invalid WebSocket port or telemetry timeout')
        self.lock = threading.Lock()
        self.payload = None
        self.received = 0.0
        self.stop = threading.Event()
        self.subscription = self.create_subscription(
            String, '/controller/telemetry_json', self.on_message, 10)
        self.thread = None
        try:
            import websockets
        except ImportError:
            self.get_logger().error('Install python3-websockets to enable the telemetry bridge')
            return
        self.thread = threading.Thread(target=self.run_server, args=(websockets,), daemon=True)
        self.thread.start()

    def on_message(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            payload = json.dumps(value, allow_nan=False)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.payload = payload
            self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            if self.payload is not None and time.monotonic()-self.received < self.timeout:
                return self.payload
        return json.dumps({'status': 'telemetry_stale'})

    def run_server(self, websockets):
        async def handler(client):
            try:
                while not self.stop.is_set():
                    await asyncio.wait_for(client.send(self.snapshot()), timeout=0.2)
                    await asyncio.sleep(0.05)
            except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
                pass
            finally:
                await client.close()

        async def serve():
            async with websockets.serve(handler, self.host, self.port, close_timeout=0.5):
                self.get_logger().info(f'Telemetry bridge listening on ws://{self.host}:{self.port}')
                while not self.stop.is_set():
                    await asyncio.sleep(0.05)

        try:
            asyncio.run(serve())
        except OSError as exc:
            self.get_logger().error(f'WebSocket bridge failed: {exc}')

    def destroy_node(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
