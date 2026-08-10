from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("go2_sysnav_vln")
    params_file = os.path.join(package_share, "config", "sysnav_vln.yaml")
    default_rviz_config = os.path.join(package_share, "config", "sysnav_vln.rviz")
    object_explorer_share = get_package_share_directory("go2_object_explorer")
    base_launch = os.path.join(
        object_explorer_share, "launch", "safe_live_nav_demo.launch.py"
    )

    launch_base_stack = LaunchConfiguration("launch_base_stack")
    launch_nav2_tool_server = LaunchConfiguration("launch_nav2_tool_server")
    launch_sysnav_rviz = LaunchConfiguration("launch_sysnav_rviz")
    launch_dashboard = LaunchConfiguration("launch_dashboard")
    launch_overlay = LaunchConfiguration("launch_overlay")
    enable_sam2 = LaunchConfiguration("enable_sam2")
    enable_open_vocab_detector = LaunchConfiguration("enable_open_vocab_detector")
    enable_registered_cloud_projector = LaunchConfiguration(
        "enable_registered_cloud_projector"
    )
    organized_cloud_topic = LaunchConfiguration("organized_cloud_topic")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_frontier_vision_guard = LaunchConfiguration(
        "enable_frontier_vision_guard"
    )
    auto_start = LaunchConfiguration("auto_start")
    mission_mode = LaunchConfiguration("mission_mode")
    ollama_model = LaunchConfiguration("ollama_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    object_nav_condition = IfCondition(
        PythonExpression(["'", mission_mode, "' == 'object_nav'"])
    )
    autonomous_explore_condition = IfCondition(
        PythonExpression(["'", mission_mode, "' == 'autonomous_explore'"])
    )

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        condition=IfCondition(launch_base_stack),
        launch_arguments={
            "launch_rviz": "false",
            "launch_dashboard": launch_dashboard,
            "launch_overlay": launch_overlay,
            "enable_sam2": enable_sam2,
        }.items(),
    )

    sysnav_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_sysnav_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_sysnav_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_base_stack", default_value="true"),
            DeclareLaunchArgument("launch_nav2_tool_server", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_sysnav_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("rviz_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("launch_dashboard", default_value="true"),
            DeclareLaunchArgument("launch_overlay", default_value="true"),
            DeclareLaunchArgument("enable_sam2", default_value="false"),
            DeclareLaunchArgument("enable_open_vocab_detector", default_value="true"),
            DeclareLaunchArgument(
                "enable_registered_cloud_projector", default_value="false"
            ),
            DeclareLaunchArgument(
                "organized_cloud_topic", default_value="/camera/depth/color/points"
            ),
            DeclareLaunchArgument(
                "mission_mode",
                default_value="object_nav",
                description="object_nav or autonomous_explore",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description="Explicit physical-motion arm. Keep false for dry-run validation.",
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "enable_frontier_vision_guard",
                default_value="false",
                description=(
                    "Require a fresh camera-based Ollama safety assessment before each "
                    "frontier dispatch. Keep false until the offline probe passes."
                ),
            ),
            DeclareLaunchArgument(
                "enable_frontier_vision_guard",
                default_value="false",
                description=(
                    "Require a fresh camera-based Ollama safety assessment before each "
                    "frontier dispatch. Keep false until the offline probe passes."
                ),
            ),
            DeclareLaunchArgument("ollama_model", default_value="gemma4:e4b"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("max_mission_duration_sec", default_value="600.0"),
            DeclareLaunchArgument("max_goals", default_value="12"),
            DeclareLaunchArgument("max_failures", default_value="4"),
            DeclareLaunchArgument("max_travel_distance_m", default_value="20.0"),
            DeclareLaunchArgument("require_stable_frontier", default_value="true"),
            sysnav_rviz,
            base_stack,
            Node(
                package="go2_nav_tools",
                executable="nav2_tool_server",
                name="go2_nav2_tool_server",
                output="screen",
                condition=IfCondition(launch_nav2_tool_server),
                parameters=[
                    {
                        "enable_motion": ParameterValue(enable_motion, value_type=bool),
                        "preflight_path": True,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="target_conditioned_detector",
                name="go2_target_conditioned_detector",
                output="screen",
                condition=IfCondition(enable_open_vocab_detector),
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="registered_cloud_object_projector",
                name="go2_registered_cloud_object_projector",
                output="screen",
                condition=IfCondition(enable_registered_cloud_projector),
                parameters=[
                    params_file,
                    {
                        "organized_cloud_topic": organized_cloud_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="pose_aware_object_mapper",
                name="go2_pose_aware_object_mapper",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="hierarchical_frontier_planner",
                name="go2_hierarchical_frontier_planner",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="vln_supervisor",
                name="go2_vln_supervisor",
                output="screen",
                condition=object_nav_condition,
                parameters=[
                    params_file,
                    {
                        "ollama_model": ollama_model,
                        "enable_motion": enable_motion,
                        "auto_start": auto_start,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="go2_sysnav_vln",
                executable="autonomous_explore_supervisor",
                name="go2_autonomous_explore_supervisor",
                output="screen",
                condition=autonomous_explore_condition,
                parameters=[
                    params_file,
                    {
                        "ollama_model": ollama_model,
                        "enable_motion": ParameterValue(enable_motion, value_type=bool),
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                        "enable_frontier_vision_guard": ParameterValue(
                            enable_frontier_vision_guard, value_type=bool
                        ),
                        "max_mission_duration_sec": ParameterValue(
                            LaunchConfiguration("max_mission_duration_sec"),
                            value_type=float,
                        ),
                        "max_goals": ParameterValue(
                            LaunchConfiguration("max_goals"), value_type=int
                        ),
                        "max_failures": ParameterValue(
                            LaunchConfiguration("max_failures"), value_type=int
                        ),
                        "max_travel_distance_m": ParameterValue(
                            LaunchConfiguration("max_travel_distance_m"),
                            value_type=float,
                        ),
                        "require_stable_frontier": ParameterValue(
                            LaunchConfiguration("require_stable_frontier"),
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
