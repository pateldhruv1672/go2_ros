from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("marker_topic", default_value="/mrkl_explorer/safe_frontiers"),
            DeclareLaunchArgument("json_topic", default_value="/mrkl_explorer/safe_frontier_candidates"),
            DeclareLaunchArgument("footprint_radius_m", default_value="0.34"),
            DeclareLaunchArgument("max_clutter_fraction", default_value="0.35"),
            DeclareLaunchArgument("max_frontiers", default_value="10"),
            Node(
                package="go2_object_explorer",
                executable="safe_frontier_filter_node",
                name="safe_frontier_filter_node",
                output="screen",
                parameters=[
                    {
                        "map_topic": LaunchConfiguration("map_topic"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "json_topic": LaunchConfiguration("json_topic"),
                        "footprint_radius_m": LaunchConfiguration("footprint_radius_m"),
                        "max_clutter_fraction": LaunchConfiguration("max_clutter_fraction"),
                        "max_frontiers": LaunchConfiguration("max_frontiers"),
                    }
                ],
            ),
        ]
    )
