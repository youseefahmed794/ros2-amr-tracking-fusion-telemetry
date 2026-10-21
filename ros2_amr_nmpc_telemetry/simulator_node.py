"""Wall-clock ROS wrapper for the shared wheel-dynamics simulation."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from .simulation import DifferentialDrivePlant, SCENARIOS


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('amr_simulator')
        seed = self.declare_parameter('seed', 42).value
        self.scenario = self.declare_parameter('scenario', 'nominal').value
        if self.scenario not in SCENARIOS:
            raise ValueError('Unknown fault scenario')
        self.plant = DifferentialDrivePlant(seed)
        self.start = self.previous = time.monotonic()
        self.pending = []
        self.tick = 0
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/simulation/status', 10)
        self.timer = self.create_timer(0.01, self.step)

    def elapsed(self):
        return time.monotonic()-self.start

    def active_fault(self, t):
        return 12 <= t < 16

    def on_command(self, msg):
        t = self.elapsed()
        if not (self.scenario == 'command_dropout' and self.active_fault(t)):
            self.plant.command(msg.linear.x, msg.angular.z, t)

    def step(self):
        now = time.monotonic()
        dt, self.previous = now-self.previous, now
        t = now-self.start
        if not 0 < dt <= 0.1:
            # Break a long scheduling stall into bounded dynamics steps; watchdog
            # sees real elapsed time and therefore cannot keep an old command alive.
            dt = max(0, min(dt, 1.0))
        fault = self.active_fault(t)
        count = max(1, math.ceil(dt/0.01))
        for _ in range(count):
            self.plant.step(t, max(dt/count, 1e-6),
                            traction=0.65 if self.scenario == 'slip' and fault else 1,
                            load=0.7 if self.scenario == 'load' and fault else 1)
        stamp = self.get_clock().now().to_msg()
        truth = Odometry(); truth.header.stamp = stamp
        truth.header.frame_id = 'odom'; truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x, truth.pose.pose.position.y = map(float, self.plant.pose[:2])
        truth.pose.pose.orientation.z = math.sin(self.plant.pose[2]/2)
        truth.pose.pose.orientation.w = math.cos(self.plant.pose[2]/2)
        truth.twist.twist.linear.x, truth.twist.twist.angular.z = map(float, self.plant.velocity)
        self.truth_pub.publish(truth)
        joint = JointState(); joint.header.stamp = stamp
        joint.name = ['left_wheel_joint', 'right_wheel_joint']
        joint.position = list(map(float, self.plant.wheel_position))
        joint.velocity = list(map(float, self.plant.wheels))
        self.joint_pub.publish(joint)
        if self.tick % 2 == 0:
            wheel, rate = self.plant.sensors()
            odom = Odometry(); odom.header.stamp = stamp
            odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = float(wheel[0])
            odom.twist.twist.angular.z = float(wheel[1])
            odom.twist.covariance[0] = odom.twist.covariance[35] = self.plant.config.encoder_std**2
            if self.scenario == 'nonfinite' and fault:
                odom.twist.twist.linear.x = float('nan')
            imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'base_link'
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.z = float(rate)
            imu.angular_velocity_covariance[8] = self.plant.config.imu_std**2
            if not (self.scenario == 'imu_dropout' and fault):
                self.imu_pub.publish(imu)
            if not (self.scenario == 'odom_dropout' and fault):
                delay = 0.8 if self.scenario == 'odom_delay' and fault else 0
                self.pending.append((t+delay, odom))
            status = String()
            status.data = json.dumps({'elapsed_sec': t, 'scenario': self.scenario,
                                      'fault_active': fault, 'watchdog': self.plant.watchdog})
            self.status_pub.publish(status)
        ready = [p for p in self.pending if p[0] <= t]
        self.pending = [p for p in self.pending if p[0] > t]
        for _, odom in ready:
            self.odom_pub.publish(odom)
        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
