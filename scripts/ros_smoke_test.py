import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from ros2_amr_nmpc_telemetry.ekf_localization_node import EKFLocalizationNode
from ros2_amr_nmpc_telemetry.adaptive_controller_node import AdaptiveAMRControllerNode

rclpy.init()
ekf = EKFLocalizationNode()
controller = AdaptiveAMRControllerNode()
probe = Node('amr_validation_probe')
odom = probe.create_publisher(Odometry, '/wheel_odom', 10)
imu = probe.create_publisher(Imu, '/imu/data', 10)
commands, states = [], []
probe.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((m.linear.x, m.angular.z)), 10)
probe.create_subscription(Odometry, '/odometry/filtered', lambda m: states.append(m), 10)
executor = MultiThreadedExecutor(num_threads=3)
for node in (ekf, controller, probe): executor.add_node(node)
try:
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        msg = Odometry(); msg.twist.twist.linear.x = 0.3
        msg.header.stamp = probe.get_clock().now().to_msg()
        inertial = Imu(); inertial.header.stamp = msg.header.stamp
        odom.publish(msg); imu.publish(inertial)
        executor.spin_once(timeout_sec=0.01)
    assert states, 'No filtered state'
    assert any(abs(v)>0 for v,w in commands), 'No moving command'
    assert all(abs(v)<=1.5 and abs(w)<=2 for v,w in commands)
    deadline = time.monotonic()+1.5
    while time.monotonic() < deadline: executor.spin_once(timeout_sec=0.01)
    assert commands[-1] == (0.0, 0.0), commands[-1]
    assert controller.latest_telemetry['status'] == 'stopped'
    print('PASS: sensor -> EKF -> controller commands; limits; stale sensor stop')
finally:
    executor.shutdown()
    for node in (controller, ekf, probe): node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
