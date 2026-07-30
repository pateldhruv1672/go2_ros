import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    explorer_share = get_package_share_directory("go2_object_explorer")
    safe_launch = os.path.join(explorer_share, "launch", "safe_live_nav_demo.launch.py")
    fallback_launch = os.path.join(explorer_share, "launch", "explorer_live_nav.launch.py")
    base_launch = safe_launch if os.path.exists(safe_launch) else fallback_launch

    include_args = {}
    if base_launch == safe_launch:
        include_args = {
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_dashboard": LaunchConfiguration("launch_dashboard"),
            "launch_overlay": LaunchConfiguration("launch_overlay"),
            "enable_sam2": LaunchConfiguration("enable_sam2"),
            "yolo_conf": LaunchConfiguration("yolo_conf"),
        }

    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_dashboard", default_value="true"),
        DeclareLaunchArgument("launch_overlay", default_value="true"),
        DeclareLaunchArgument("enable_sam2", default_value="false"),
        DeclareLaunchArgument("yolo_conf", default_value="0.60"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments=include_args.items(),
        ),
        Node(
            package="go2_object_explorer",
            executable="safe_frontier_filter_node",
            name="safe_frontier_filter_node",
            output="screen",
            parameters=[{
                "map_topic": "/map",
                "footprint_radius_m": 0.53,
                "frontier_goal_min_dist_m": 0.50,
                "frontier_goal_max_dist_m": 1.20,
                "robot_min_goal_dist_m": 0.70,
                "robot_max_goal_dist_m": 5.0,
                "clutter_radius_m": 0.70,
                "max_clutter_fraction": 0.30,
                "min_cluster_cells": 10,
            }],
        ),
        Node(
            package="go2_nav_tools",
            executable="nav2_tool_server",
            name="go2_nav2_tool_server",
            output="screen",
            parameters=[{"enable_motion": True, "preflight_path": True}],
        ),
        Node(
            package="go2_nav_tools",
            executable="agentic_explorer_supervisor",
            name="go2_agentic_explorer_supervisor",
            output="screen",
            parameters=[{"auto_start": LaunchConfiguration("auto_start")}],
        ),
    ])
