from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("go2_semantic_nav_agent")
    rviz_cfg = PathJoinSubstitution([pkg_share, "config", "semantic_nav.rviz"])

    session_root = LaunchConfiguration("session_root")
    session_name = LaunchConfiguration("session_name")

    return LaunchDescription([
        DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
        DeclareLaunchArgument("session_name", default_value="latest"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("publish_markers", default_value="true"),
        DeclareLaunchArgument("enable_goal_bridge", default_value="true"),
        DeclareLaunchArgument("use_omi", default_value="true"),
        DeclareLaunchArgument("use_voice_transcript", default_value="true"),
        DeclareLaunchArgument("text_command_topic", default_value="/semantic_nav/text_command"),
        DeclareLaunchArgument("omi_transcript_topic", default_value="/omi/transcript_raw"),
        DeclareLaunchArgument("voice_transcript_topic", default_value="/go2_voice/transcript"),
        DeclareLaunchArgument("navigate_action", default_value="/navigate_to_pose"),
        DeclareLaunchArgument("ignore_commands_without_sparky", default_value="false"),

        Node(
            package="go2_semantic_nav_agent",
            executable="semantic_demo_marker_publisher",
            name="semantic_demo_marker_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_markers")),
            parameters=[{
                "session_root": session_root,
                "session_name": session_name,
                "marker_topic": "/semantic_nav/demo_markers",
                "status_topic": "/semantic_nav/demo_status",
            }],
        ),

        Node(
            package="go2_semantic_nav_agent",
            executable="semantic_goal_bridge",
            name="semantic_goal_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_goal_bridge")),
            parameters=[{
                "session_root": session_root,
                "session_name": session_name,
                "text_command_topic": LaunchConfiguration("text_command_topic"),
                "omi_transcript_topic": LaunchConfiguration("omi_transcript_topic"),
                "voice_transcript_topic": LaunchConfiguration("voice_transcript_topic"),
                "navigate_action": LaunchConfiguration("navigate_action"),
                "ignore_commands_without_sparky": LaunchConfiguration("ignore_commands_without_sparky"),
                "status_topic": "/semantic_nav/goal_bridge_status",
                "target_marker_topic": "/semantic_nav/target_marker",
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="semantic_demo_rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", rviz_cfg],
            additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
        ),
    ])
