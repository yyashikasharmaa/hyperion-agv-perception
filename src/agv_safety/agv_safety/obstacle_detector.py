#!/usr/bin/env python3

"""DBSCAN obstacle detector with optional STVL costmap validation.

This sanitized public node is derived from the deployed HYPERION perception
pipeline. It clusters /merged_cloud, rejects undersized/noise clusters,
validates centroids against the local costmap when available, and publishes
obstacle poses plus four directional safety-zone occupancy flags.
"""

import math
import time

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        self.declare_parameter("cloud_topic", "/merged_cloud")
        self.declare_parameter("costmap_topic", "/costmap/costmap")
        self.declare_parameter("target_frame", "load_wheel_base_link")
        self.declare_parameter("dbscan_eps", 0.10)
        self.declare_parameter("dbscan_min_samples", 5)
        self.declare_parameter("min_cluster_points", 12)
        self.declare_parameter("zone_range", 1.2)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.dbscan_eps = float(self.get_parameter("dbscan_eps").value)
        self.dbscan_min_samples = int(self.get_parameter("dbscan_min_samples").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.zone_range = float(self.get_parameter("zone_range").value)

        self.grid = None
        self.grid_meta = None
        self.costmap_frame = None
        self.last_costmap_warn = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, "/obstacle_poses", 10)
        self.zone_pubs = {
            name: self.create_publisher(Bool, f"/safety_zones/{name}/occupied", 10)
            for name in ("front", "left", "back", "right")
        }

        self.get_logger().info(
            f"Obstacle detector ready: eps={self.dbscan_eps}, "
            f"min_samples={self.dbscan_min_samples}, min_cluster={self.min_cluster_points}"
        )

    def costmap_callback(self, msg):
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.grid_meta = msg.info
        self.costmap_frame = msg.header.frame_id

    def transform_xy(self, x, y, source_frame, target_frame):
        if source_frame == target_frame:
            return x, y
        p = PointStamped()
        p.header.frame_id = source_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y, p.point.z = float(x), float(y), 0.0
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            q = tf2_geometry_msgs.do_transform_point(p, t)
            return q.point.x, q.point.y
        except Exception:
            return None

    def costmap_confirms(self, x, y, frame_id):
        if self.grid is None:
            now = time.time()
            if now - self.last_costmap_warn > 5.0:
                self.get_logger().warn("No costmap yet; accepting LiDAR clusters without validation")
                self.last_costmap_warn = now
            return True

        transformed = self.transform_xy(x, y, frame_id, self.costmap_frame)
        if transformed is None:
            return False
        x, y = transformed
        info = self.grid_meta
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        w = 2
        r0, r1 = max(0, row - w), min(info.height, row + w + 1)
        c0, c1 = max(0, col - w), min(info.width, col + w + 1)
        if r0 >= r1 or c0 >= c1:
            return False
        return bool(np.any(self.grid[r0:r1, c0:c1] >= 50))

    @staticmethod
    def zone_for(x, y):
        angle = math.atan2(y, x)
        if -math.pi / 4 <= angle < math.pi / 4:
            return "front"
        if math.pi / 4 <= angle < 3 * math.pi / 4:
            return "left"
        if -3 * math.pi / 4 <= angle < -math.pi / 4:
            return "right"
        return "back"

    def cloud_callback(self, msg):
        pts = np.asarray([
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(pts) == 0:
            return

        labels = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit_predict(pts[:, :2])
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = self.target_frame
        zones = {"front": False, "left": False, "back": False, "right": False}

        for cluster_id in set(labels) - {-1}:
            cluster = pts[labels == cluster_id]
            if len(cluster) < self.min_cluster_points:
                continue
            centroid = cluster.mean(axis=0)
            if not self.costmap_confirms(centroid[0], centroid[1], msg.header.frame_id):
                continue

            transformed = self.transform_xy(centroid[0], centroid[1], msg.header.frame_id, self.target_frame)
            if transformed is None:
                continue
            x, y = transformed

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, 0.0
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            if math.hypot(x, y) <= self.zone_range:
                zones[self.zone_for(x, y)] = True

        self.pose_pub.publish(poses)
        for name, occupied in zones.items():
            self.zone_pubs[name].publish(Bool(data=occupied))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
