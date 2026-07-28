from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(
    package,
    filename,
    arguments=None,
    condition=None,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package), "launch", filename]
            )
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    launch_frontier = LaunchConfiguration(
        "launch_frontier_explorer"
    )

    safe_stack = include_launch(
        "go2_object_explorer",
        "safe_live_nav_demo.launch.py",
        {
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_dashboard": LaunchConfiguration(
                "launch_dashboard"
            ),
            "launch_overlay": LaunchConfiguration(
                "launch_overlay"
            ),
            "enable_sam2": LaunchConfiguration("enable_sam2"),
            "yolo_conf": LaunchConfiguration("yolo_conf"),
        },
    )

    diagnostic_logger = Node(
        package="go2_object_explorer",
        executable="nav2_diagnostic_logger_node",
        name="nav2_diagnostic_logger_node",
        output="screen",
        parameters=[
            {
                "log_dir": LaunchConfiguration(
                    "diagnostic_log_dir"
                ),
                "sample_hz": LaunchConfiguration(
                    "diagnostic_sample_hz"
                ),
                "snapshot_delay_sec": LaunchConfiguration(
                    "snapshot_delay_sec"
                ),
                "workspace_dir": LaunchConfiguration(
                    "workspace_dir"
                ),
                "map_frame": "map",
                "base_frame": "base_link",
            }
        ],
    )

    # Optional. Disabled by default so baseline Nav2 can be tested
    # without autonomous frontier goals.
    frontier_explorer = TimerAction(
        period=LaunchConfiguration(
            "frontier_start_delay_sec"
        ),
        actions=[
            include_launch(
                "go2_object_explorer",
                "frontier_object_explore.launch.py",
                {
                    "image_topic": "/camera/image_raw",
                    "camera_info_topic": "/camera/camera_info",
                    "scan_topic": "/scan_nav",
                    "map_topic": "/map",

                    # Motion remains inside:
                    # explorer -> arbiter -> collision monitor -> driver.
                    "cmd_vel_topic": "/cmd_vel_omi",

                    "yolo_model": "yolov8n.pt",
                    "yolo_conf": LaunchConfiguration("yolo_conf"),
                    "enable_sam2": LaunchConfiguration(
                        "enable_sam2"
                    ),
                    "auto_navigate": LaunchConfiguration(
                        "frontier_auto_navigate"
                    ),
                    "search_timeout_sec": "300.0",
                    "local_scan_duration_sec": "8.0",
                    "frontier_goal_timeout_sec": "90.0",
                    "frontier_max_distance_m": "6.0",
                    "frontier_camera_map_mode": "fallback_and",
                    "use_ollama_frontier_selector":
                        LaunchConfiguration(
                            "use_ollama_frontier_selector"
                        ),
                    "ollama_url": LaunchConfiguration(
                        "ollama_url"
                    ),
                    "ollama_model": LaunchConfiguration(
                        "ollama_model"
                    ),
                    "ollama_timeout_sec": "45.0",
                    "llm_top_k_frontiers": "8",
                },
                condition=IfCondition(launch_frontier),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "launch_dashboard",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "launch_overlay",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_sam2",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "yolo_conf",
                default_value="0.60",
            ),

            DeclareLaunchArgument(
                "diagnostic_log_dir",
                default_value=(
                    "~/.ros/go2_object_explorer/"
                    "nav2_diagnostics"
                ),
            ),
            DeclareLaunchArgument(
                "diagnostic_sample_hz",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "snapshot_delay_sec",
                default_value="55.0",
            ),
            DeclareLaunchArgument(
                "workspace_dir",
                default_value=(
                    "~/Dhruv/sparky/ros2_ws"
                ),
            ),

            DeclareLaunchArgument(
                "launch_frontier_explorer",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "frontier_auto_navigate",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "frontier_start_delay_sec",
                default_value="50.0",
            ),
            DeclareLaunchArgument(
                "use_ollama_frontier_selector",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "ollama_url",
                default_value="http://127.0.0.1:11434",
            ),
            DeclareLaunchArgument(
                "ollama_model",
                default_value="gemma4:e4b",
            ),

            LogInfo(
                msg=[
                    "[diagnostics] Logs will be written under ",
                    LaunchConfiguration("diagnostic_log_dir"),
                ]
            ),

            safe_stack,
            diagnostic_logger,
            frontier_explorer,
        ]
    )
