from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434"),
            DeclareLaunchArgument("ollama_model", default_value="gemma4:e4b"),
            DeclareLaunchArgument("ollama_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("ollama_keep_alive", default_value="30m"),
            DeclareLaunchArgument("ollama_num_ctx", default_value="8192"),
            DeclareLaunchArgument("ollama_num_predict", default_value="512"),
            DeclareLaunchArgument("ollama_temperature", default_value="0.1"),
            DeclareLaunchArgument("decision_period_sec", default_value="3.0"),
            DeclareLaunchArgument("frontier_marker_topic", default_value="/object_explorer/frontiers"),
            DeclareLaunchArgument("frontier_required_ns_contains", default_value=""),
            DeclareLaunchArgument("frontier_standoff_m", default_value="0.65"),
            DeclareLaunchArgument("frontier_footprint_radius_m", default_value="0.32"),
            DeclareLaunchArgument("frontier_clutter_radius_m", default_value="0.65"),
            DeclareLaunchArgument("frontier_max_clutter_fraction", default_value="0.40"),
            DeclareLaunchArgument("scan_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("object_approach_distance_m", default_value="0.85"),
            Node(
                package="go2_object_explorer",
                executable="mrkl_explorer_agent_node",
                name="mrkl_explorer_agent_node",
                output="screen",
                parameters=[
                    {
                        "ollama_url": LaunchConfiguration("ollama_url"),
                        "ollama_model": LaunchConfiguration("ollama_model"),
                        "ollama_timeout_sec": LaunchConfiguration("ollama_timeout_sec"),
                        "ollama_keep_alive": LaunchConfiguration("ollama_keep_alive"),
                        "ollama_num_ctx": LaunchConfiguration("ollama_num_ctx"),
                        "ollama_num_predict": LaunchConfiguration("ollama_num_predict"),
                        "ollama_temperature": LaunchConfiguration("ollama_temperature"),
                        "decision_period_sec": LaunchConfiguration("decision_period_sec"),
                        "frontier_marker_topic": LaunchConfiguration("frontier_marker_topic"),
                        "frontier_required_ns_contains": LaunchConfiguration("frontier_required_ns_contains"),
                        "frontier_standoff_m": LaunchConfiguration("frontier_standoff_m"),
                        "frontier_footprint_radius_m": LaunchConfiguration("frontier_footprint_radius_m"),
                        "frontier_clutter_radius_m": LaunchConfiguration("frontier_clutter_radius_m"),
                        "frontier_max_clutter_fraction": LaunchConfiguration("frontier_max_clutter_fraction"),
                        "scan_duration_sec": LaunchConfiguration("scan_duration_sec"),
                        "object_approach_distance_m": LaunchConfiguration("object_approach_distance_m"),
                    }
                ],
            ),
        ]
    )
