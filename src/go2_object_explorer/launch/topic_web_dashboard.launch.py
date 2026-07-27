from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("auto_discover", default_value="true"),
            DeclareLaunchArgument("topic_allow_regex", default_value=".*"),
            DeclareLaunchArgument("topic_deny_regex", default_value="^/parameter_events$"),
            DeclareLaunchArgument("max_messages_per_topic", default_value="100"),
            DeclareLaunchArgument("max_global_events", default_value="300"),
            DeclareLaunchArgument("discover_period_sec", default_value="2.0"),
            DeclareLaunchArgument("log_dir", default_value="~/.ros/go2_object_explorer/topic_web_logs"),
            Node(
                package="go2_object_explorer",
                executable="topic_web_dashboard_node",
                name="topic_web_dashboard_node",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "auto_discover": LaunchConfiguration("auto_discover"),
                        "topic_allow_regex": LaunchConfiguration("topic_allow_regex"),
                        "topic_deny_regex": LaunchConfiguration("topic_deny_regex"),
                        "max_messages_per_topic": LaunchConfiguration("max_messages_per_topic"),
                        "max_global_events": LaunchConfiguration("max_global_events"),
                        "discover_period_sec": LaunchConfiguration("discover_period_sec"),
                        "log_dir": LaunchConfiguration("log_dir"),
                    }
                ],
            ),
        ]
    )
