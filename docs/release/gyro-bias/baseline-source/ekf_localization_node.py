#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from .core import VelocityEKF
from .freshness import TimestampGate


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class EKFLocalizationNode(Node):
    """
    Extended Kalman Filter (EKF) Node fusing noisy Wheel Odometry and IMU yaw rate.
    State vector: x = [px, py, theta, v, omega]^T
    """

    def __init__(self):
        super().__init__('ekf_localization_node')

        # Declare explicit parameters with default values
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('process_noise_pos', 0.01)
        self.declare_parameter('process_noise_yaw', 0.005)
        self.declare_parameter('meas_noise_odom', 0.05)
        self.declare_parameter('meas_noise_imu', 0.002)

        rate_hz = self.get_parameter('publish_rate_hz').value
        q_pos = self.get_parameter('process_noise_pos').value
        q_yaw = self.get_parameter('process_noise_yaw').value
        r_odom = self.get_parameter('meas_noise_odom').value
        r_imu = self.get_parameter('meas_noise_imu').value

        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('publish_rate_hz must be positive')
        self.filter = VelocityEKF(q_pos, q_yaw, r_odom, r_imu)
        self.last_odom = None
        self.last_imu = None
        self.odom_gate = TimestampGate()
        self.imu_gate = TimestampGate()

        # Configure QoS Profiles: Best Effort for high-rate sensors, Reliable for filtered state
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.odom_callback, sensor_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos
        )
        self.filtered_pub = self.create_publisher(
            Odometry, '/odometry/filtered', reliable_qos
        )

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate_hz, self.predict_and_publish)
        self.get_logger().info('EKF Localization Node initialized at %.1f Hz.' % rate_hz)

    def predict_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Validate sampling time dt to protect filter stability against clock jitter
        if dt <= 0.0 or dt > 0.5:
            self.get_logger().warn(f'Invalid or excessive EKF dt detected ({dt:.4f}s). Skipping step.')
            return

        # Do not publish fresh-looking state when either sensor has stopped.
        if any(stamp is None or not 0 <= (now-stamp).nanoseconds*1e-9 < 0.5
               for stamp in (self.last_odom, self.last_imu)):
            return
        self.filter.predict(dt)

        # Publish Filtered Odometry
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.filter.x[0, 0])
        msg.pose.pose.position.y = float(self.filter.x[1, 0])
        msg.pose.pose.orientation = yaw_to_quaternion(float(self.filter.x[2, 0]))
        msg.twist.twist.linear.x = float(self.filter.x[3, 0])
        msg.twist.twist.angular.z = float(self.filter.x[4, 0])
        for i, state_i in enumerate((0, 1, 2)):
            for j, state_j in enumerate((0, 1, 2)):
                msg.pose.covariance[(0, 1, 5)[i]*6+(0, 1, 5)[j]] = float(self.filter.P[state_i, state_j])
        msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e6
        msg.twist.covariance[0] = float(self.filter.P[3, 3])
        msg.twist.covariance[35] = float(self.filter.P[4, 4])
        self.filtered_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if not self.odom_gate.accept(stamp, now.nanoseconds):
            return
        if self.filter.update([msg.twist.twist.linear.x, msg.twist.twist.angular.z],
                              (3, 4), self.filter.R_odom):
            self.last_odom = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if not self.imu_gate.accept(stamp, now.nanoseconds):
            return
        if self.filter.update([msg.angular_velocity.z], (4,), self.filter.R_imu):
            self.last_imu = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

