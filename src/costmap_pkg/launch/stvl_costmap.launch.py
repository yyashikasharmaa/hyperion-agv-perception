#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("costmap_pkg"), "config", "stvl_costmap.yaml"
    )
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the HYPERION STVL costmap YAML",
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="costmap",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["costmap/costmap"],
            }],
        ),
    ])
