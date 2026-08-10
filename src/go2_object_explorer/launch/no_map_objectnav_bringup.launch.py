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
    padded_nav2_params = os.path.join(
        explorer_dir,
        "config",
        "nav2_params_padded_map.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("padding_m", default_value="2.0"),
            DeclareLaunchArgument("nav2_start_delay_sec", default_value="14.0"),
            DeclareLaunchArgument("rviz2", default_value="false"),
            DeclareLaunchArgument("foxglove", default_value="false"),

            Node(
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
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    "foxglove": LaunchConfiguration("foxglove"),
                    "slam": "true",
                    "nav2": "true",
                    "rviz2": LaunchConfiguration("rviz2"),
                    "nav2_params_file": padded_nav2_params,
                    "nav2_start_delay_sec": LaunchConfiguration("nav2_start_delay_sec"),
                }.items(),
            ),
        ]
    )
