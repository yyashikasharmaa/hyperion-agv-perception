#!/usr/bin/env python3

"""Nearest-obstacle safety layer for HYPERION.

Consumes STVL's voxel cloud, transforms points into the robot frame, excludes
points inside the published robot footprint, rejects isolated returns using a
small XY clustering tolerance, and publishes the nearest supported obstacle.
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PolygonStamped, Point
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class VoxelDistanceNode(Node):
    def __init__(self):
        super().__init__("voxel_distance_node")
        self.declare_parameter("robot_frame", "load_wheel_base_link")
        self.declare_parameter("cloud_topic", "/costmap/voxel_grid")
        self.declare_parameter("footprint_topic", "/costmap/published_footprint")
        self.declare_parameter("cluster_tolerance", 0.15)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("slow_distance", 1.2)

        self.robot_frame = self.get_parameter("robot_frame").value
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.footprint_topic = self.get_parameter("footprint_topic").value
        self.cluster_tolerance = float(self.get_parameter("cluster_tolerance").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)

        self.footprint = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(PolygonStamped, self.footprint_topic, self.footprint_callback, 10)
        self.distance_pub = self.create_publisher(Float32, "/nearest_obstacle_distance", 10)
        self.status_pub = self.create_publisher(String, "/safety_status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/safety_markers", 10)

    def footprint_callback(self, msg):
        self.footprint = [(p.x, p.y) for p in msg.polygon.points]

    @staticmethod
    def point_in_polygon(x, y, polygon):
        if not polygon or len(polygon) < 3:
            return False
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
                inside = not inside
            j = i
        return inside

    def transform_points(self, pts, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                self.robot_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception as exc:
            self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=2.0)
            return None

        q = t.transform.rotation
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        return pts @ rot.T + np.array([tx, ty, tz])

    def supported_mask(self, xy):
        if len(xy) < self.min_cluster_points:
            return np.zeros(len(xy), dtype=bool)
        tol2 = self.cluster_tolerance ** 2
        mask = np.zeros(len(xy), dtype=bool)
        for i, p in enumerate(xy):
            count = np.sum(np.sum((xy - p) ** 2, axis=1) <= tol2)
            mask[i] = count >= self.min_cluster_points
        return mask

    def classify(self, distance):
        if distance <= self.stop_distance:
            return "STOP"
        if distance <= self.slow_distance:
            return "SLOW"
        return "SAFE"

    def cloud_callback(self, msg):
        pts = np.asarray([
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(pts) == 0:
            return

        pts = self.transform_points(pts, msg.header.frame_id)
        if pts is None:
            return

        if self.footprint:
            pts = np.asarray([p for p in pts if not self.point_in_polygon(p[0], p[1], self.footprint)])
        if len(pts) == 0:
            return

        xy = pts[:, :2]
        pts = pts[self.supported_mask(xy)]
        if len(pts) == 0:
            return

        distances = np.linalg.norm(pts[:, :2], axis=1)
        idx = int(np.argmin(distances))
        nearest = pts[idx]
        distance = float(distances[idx])
        state = self.classify(distance)

        self.distance_pub.publish(Float32(data=distance))
        self.status_pub.publish(String(data=state))
        self.publish_marker(nearest, distance, state)

    def publish_marker(self, p, distance, state):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nearest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=float(p[0]), y=float(p[1]), z=0.05)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        marker.color.a = 1.0
        if state == "STOP":
            marker.color.r = 1.0
        elif state == "SLOW":
            marker.color.r = marker.color.g = 1.0
        else:
            marker.color.g = 1.0

        text = Marker()
        text.header = marker.header
        text.ns = "nearest_obstacle"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(x=float(p[0]), y=float(p[1]), z=0.35)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color.a = text.color.r = text.color.g = text.color.b = 1.0
        text.text = f"{state}  {distance:.2f} m"

        arr = MarkerArray()
        arr.markers = [marker, text]
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VoxelDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
