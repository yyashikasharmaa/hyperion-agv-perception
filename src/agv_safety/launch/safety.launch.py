#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="agv_safety",
            executable="obstacle_detector.py",
            name="obstacle_detector",
            output="screen",
        ),
        Node(
            package="agv_safety",
            executable="voxel_distance_node.py",
            name="voxel_distance_node",
            output="screen",
        ),
    ])
