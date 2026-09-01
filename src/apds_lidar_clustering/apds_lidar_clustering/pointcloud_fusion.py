#!/usr/bin/env python3

"""HYPERION multi-LiDAR fusion node.

Projects configured LaserScan streams into PointCloud2, transforms each cloud
into a common robot frame with TF2, and publishes a fixed-rate merged cloud.
This is the sanitized public version of the deployment node used on the BOPT.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs

DEFAULT_LIDAR_TOPICS = [
    "/Lidar_1", "/Lidar_2", "/Lidar_3", "/Lidar_4",
    "/Lidar_LFT", "/Lidar_RFT",
]
FUSION_RATE_HZ = 20.0
TF_TIMEOUT_SEC = 0.1


class PointCloudFusion(Node):
    def __init__(self):
        super().__init__("pointcloud_fusion")

        self.declare_parameter("target_frame", "load_wheel_base_link")
        self.declare_parameter("lidar_topics", DEFAULT_LIDAR_TOPICS)
        self.declare_parameter("fusion_rate_hz", FUSION_RATE_HZ)

        self.target_frame = self.get_parameter("target_frame").value
        self.lidar_topics = list(self.get_parameter("lidar_topics").value)
        fusion_rate = float(self.get_parameter("fusion_rate_hz").value)

        self.latest_scans = {}
        self.subscribers = []
        for topic in self.lidar_topics:
            sub = self.create_subscription(
                LaserScan,
                topic,
                lambda msg, t=topic: self._scan_callback(msg, t),
                qos_profile_sensor_data,
            )
            self.subscribers.append(sub)

        self.cloud_pub = self.create_publisher(PointCloud2, "/merged_cloud", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.projector = LaserProjection()
        self.publish_count = 0
        self.create_timer(1.0 / fusion_rate, self._fuse)

        self.get_logger().info(
            f"Fusion ready: {len(self.lidar_topics)} LiDARs -> {self.target_frame} "
            f"at {fusion_rate:.1f} Hz"
        )

    def _scan_callback(self, msg, topic):
        self.latest_scans[topic] = msg

    def _fuse(self):
        if not self.latest_scans:
            return

        merged_points = []
        active_sensors = 0

        for topic in self.lidar_topics:
            scan = self.latest_scans.get(topic)
            if scan is None:
                continue

            cloud = self.projector.projectLaser(scan)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    cloud.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=TF_TIMEOUT_SEC),
                )
                transformed = tf2_sensor_msgs.do_transform_cloud(cloud, transform)
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping {topic}; TF {cloud.header.frame_id} -> "
                    f"{self.target_frame} unavailable: {exc}",
                    throttle_duration_sec=5.0,
                )
                continue

            for p in point_cloud2.read_points(
                transformed, field_names=("x", "y", "z"), skip_nans=True
            ):
                merged_points.append((float(p[0]), float(p[1]), float(p[2])))
            active_sensors += 1

        if not merged_points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        output = point_cloud2.create_cloud_xyz32(header, merged_points)
        self.cloud_pub.publish(output)

        self.publish_count += 1
        if self.publish_count % 100 == 0:
            self.get_logger().info(
                f"Publishing {len(merged_points)} points from "
                f"{active_sensors}/{len(self.lidar_topics)} LiDARs"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
