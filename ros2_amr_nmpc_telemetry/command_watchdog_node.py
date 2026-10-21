"""Wall-clock command-loss gate for the ROS -> Gazebo drive bridge.

This separate process must remain alive; it cannot protect against its own death
or a failed bridge. Twist has no source stamp, so freshness is receipt-based.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class CommandWatchdogNode(Node):
    def __init__(self):
        super().__init__('gazebo_command_watchdog')
        self.timeout = self.declare_parameter('command_timeout_sec', 0.2).value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('command_timeout_sec must be finite and positive')
        self.command = Twist()
        self.received = None
        self.publisher = self.create_publisher(Twist, '/gazebo/cmd_vel', 1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        # Explicit steady clock: still publish zeros when /clock is paused.
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        if not all(math.isfinite(value) for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)):
            self.received = None
            self.command = Twist()
            self.publisher.publish(self.command)
            return
        # This is a planar drive; forward only its two commanded axes.
        self.command = Twist()
        self.command.linear.x = max(-1.5, min(1.5, msg.linear.x))
        self.command.angular.z = max(-2.0, min(2.0, msg.angular.z))
        self.received = time.monotonic()
        self.publisher.publish(self.command)

    def tick(self):
        fresh = self.received is not None and time.monotonic()-self.received < self.timeout
        self.publisher.publish(self.command if fresh else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CommandWatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
