#!/usr/bin/env python3
"""Canonical Sparky Tour launcher.

This wrapper deliberately owns RViz at the top level.  The previously existing
Tour launch is preserved as semantic_nav_tour_world_core.launch.py and is
included with its internal RViz disabled, preventing duplicate/suppressed RViz
instances while keeping every other Tour argument and behavior unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("go2_semantic_nav_agent")

    rviz_enabled = LaunchConfiguration("rviz2")
    rviz_config = LaunchConfiguration("rviz_config")

    # Parent launch configurations (session_name, phone flags, perception flags,
    # motion flags, Ollama model, etc.) remain visible to the included core.
    # Only RViz is overridden so there is exactly one owner.
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "semantic_nav_tour_world_core.launch.py"]
            )
        ),
        launch_arguments={"rviz2": "false"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="sparky_tour_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz_enabled),
        respawn=True,
        respawn_delay=3.0,
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_X11_NO_MITSHM": "1",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz2",
                default_value="true",
                description="Launch the canonical Sparky Tour RViz process.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "semantic_nav.rviz"]
                ),
                description="RViz configuration used by Tour Mode.",
            ),
            core,
            rviz,
        ]
    )
