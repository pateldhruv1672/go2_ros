from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("omi_text_topic", default_value=""),
        DeclareLaunchArgument("require_wake_word", default_value="false"),
        DeclareLaunchArgument("semantic_command_topic", default_value="/semantic_nav/command"),
        DeclareLaunchArgument("omi_status_topic", default_value="/semantic_nav/omi_status"),
        DeclareLaunchArgument("canonical_transcript_topic", default_value="/omi/transcript"),
        DeclareLaunchArgument("enable_direct_motion", default_value="true"),
        DeclareLaunchArgument("motion_cmd_topic", default_value="/cmd_vel_nav2"),

        Node(
            package="go2_semantic_nav_agent",
            executable="semantic_omi_command_router",
            name="semantic_omi_command_router",
            output="screen",
            parameters=[{
                "explicit_omi_topic": LaunchConfiguration("omi_text_topic"),
                "require_wake_word": LaunchConfiguration("require_wake_word"),
                "semantic_command_topic": LaunchConfiguration("semantic_command_topic"),
                "status_topic": LaunchConfiguration("omi_status_topic"),
                "canonical_transcript_topic": LaunchConfiguration("canonical_transcript_topic"),
                "enable_direct_motion": LaunchConfiguration("enable_direct_motion"),
                "motion_cmd_topic": LaunchConfiguration("motion_cmd_topic"),
            }],
        ),
    ])
