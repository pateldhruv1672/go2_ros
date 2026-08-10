import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_robot_sdk_dir = get_package_share_directory("go2_robot_sdk")
    explorer_dir = get_package_share_directory("go2_object_explorer")

    robot_launch = os.path.join(go2_robot_sdk_dir, "launch", "robot.launch.py")
    nav2_params = os.path.join(explorer_dir, "config", "nav2_params_explorer.yaml")

    motion_arbiter = Node(
        package="go2_nav_tools",
        executable="motion_arbiter",
        name="go2_motion_arbiter",
        output="screen",
        parameters=[
            {
                "nav2_topic": "/cmd_vel_nav2",
                "omi_topic": "/cmd_vel_omi",
                "escape_topic": "/cmd_vel_escape",
                "output_topic": "/cmd_vel_nav",
                "source_timeout_sec": 0.40,
                "nav2_max_x": 0.75,
                "nav2_max_y": 0.0,
                "nav2_max_theta": 0.90,
                "omi_max_x": 0.25,
                "omi_max_y": 0.20,
                "omi_max_theta": 0.60,
                "escape_max_x": 0.30,
                "escape_max_y": 0.25,
                "escape_max_theta": 0.70,
            }
        ],
    )

    padded_map = Node(
        package="go2_object_explorer",
        executable="padded_map_node",
        name="padded_map_node",
        output="screen",
        parameters=[
            {
                "input_map_topic": "/map",
                "output_map_topic": "/map_padded",
                "padding_m": LaunchConfiguration("padding_m"),
            }
        ],
    )

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "foxglove": LaunchConfiguration("foxglove"),
            "slam": "true",
            "nav2": "true",
            "rviz2": LaunchConfiguration("rviz2"),
            "nav2_params_file": nav2_params,
            "nav2_start_delay_sec": LaunchConfiguration("nav2_start_delay_sec"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("rviz2", default_value="false"),
            DeclareLaunchArgument("padding_m", default_value="3.0"),
            DeclareLaunchArgument("nav2_start_delay_sec", default_value="30.0"),
            motion_arbiter,
            padded_map,
            base_stack,
        ]
    )
