"""RViz ground truth, EKF and reference paths; ground truth TF is display-only."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from .core import reference


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('amr_visualization')
        self.tf = TransformBroadcaster(self)
        self.paths = {'true': Path(), 'estimated': Path()}
        self.path_publishers = {name: self.create_publisher(Path, '/simulation/'+name+'_path', 10)
                           for name in self.paths}
        self.ref_pub = self.create_publisher(Path, '/simulation/reference_path', 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_estimate, 10)
        self.create_timer(0.5, self.publish_reference)
        self.counter = {'true': 0, 'estimated': 0}

    def append(self, name, msg):
        self.counter[name] += 1
        if self.counter[name] % 5:
            return
        pose = PoseStamped(); pose.header = msg.header; pose.pose = msg.pose.pose
        path = self.paths[name]; path.header = msg.header
        path.poses.append(pose)
        path.poses = path.poses[-4000:]
        self.path_publishers[name].publish(path)

    def on_truth(self, msg):
        self.append('true', msg)
        transform = TransformStamped(); transform.header = msg.header
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(transform)

    def on_estimate(self, msg):
        self.append('estimated', msg)

    def publish_reference(self):
        path = Path(); path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for k in range(630):
            x, y, yaw, _, _ = reference(k*0.04)
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = x; pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw/2); pose.pose.orientation.w = math.cos(yaw/2)
            path.poses.append(pose)
        self.ref_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
