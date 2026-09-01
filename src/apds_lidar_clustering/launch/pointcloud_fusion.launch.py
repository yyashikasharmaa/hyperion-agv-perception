#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_TOPICS = [
    "/Lidar_1", "/Lidar_2", "/Lidar_3", "/Lidar_4",
    "/Lidar_LFT", "/Lidar_RFT",
]


def generate_launch_description():
    target_frame = LaunchConfiguration("target_frame")
    return LaunchDescription([
        DeclareLaunchArgument(
            "target_frame",
            default_value="load_wheel_base_link",
            description="Common frame for all fused LiDAR data",
        ),
        Node(
            package="apds_lidar_clustering",
            executable="pointcloud_fusion.py",
            name="pointcloud_fusion",
            output="screen",
            parameters=[{
                "target_frame": target_frame,
                "lidar_topics": LIDAR_TOPICS,
                "fusion_rate_hz": 20.0,
            }],
        ),
    ])
