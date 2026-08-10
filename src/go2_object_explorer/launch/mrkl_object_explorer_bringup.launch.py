from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(pkg, relpath, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", relpath])
        ),
        launch_arguments=(args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    launch_robot = LaunchConfiguration("launch_robot")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard = LaunchConfiguration("launch_dashboard")
    launch_safe_frontiers = LaunchConfiguration("launch_safe_frontiers")
    launch_mrkl = LaunchConfiguration("launch_mrkl")

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_robot", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_dashboard", default_value="true"),
            DeclareLaunchArgument("launch_safe_frontiers", default_value="true"),
            DeclareLaunchArgument("launch_mrkl", default_value="true"),

            DeclareLaunchArgument("padding_m", default_value="3.0"),
            DeclareLaunchArgument("nav2_start_delay_sec", default_value="30.0"),

            DeclareLaunchArgument("robot_to_nav_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("perception_delay_sec", default_value="45.0"),
            DeclareLaunchArgument("param_fix_delay_sec", default_value="48.0"),
            DeclareLaunchArgument("safe_frontier_delay_sec", default_value="55.0"),
            DeclareLaunchArgument("mrkl_delay_sec", default_value="65.0"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),

            DeclareLaunchArgument("yolo_conf", default_value="0.60"),
            DeclareLaunchArgument("target_object", default_value="bottle"),

            DeclareLaunchArgument("safe_frontier_map_topic", default_value="/global_costmap/costmap"),
            DeclareLaunchArgument("safe_frontier_footprint_radius_m", default_value="0.42"),
            DeclareLaunchArgument("safe_frontier_max_clutter_fraction", default_value="0.25"),

            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434"),
            DeclareLaunchArgument("ollama_model", default_value="gemma4:e4b"),

            # 1. Base robot only. No base SLAM/Nav2/RViz.
            include_launch(
                "go2_robot_sdk",
                "robot.launch.py",
                {
                    "foxglove": "false",
                    "slam": "false",
                    "nav2": "false",
                    "rviz2": "false",
                },
                condition=IfCondition(launch_robot),
            ),

            # 2. Live SLAM + Nav2 + motion arbiter + scan_nav + padded map.
            TimerAction(
                period=LaunchConfiguration("robot_to_nav_delay_sec"),
                actions=[
                    include_launch(
                        "go2_object_explorer",
                        "explorer_live_nav.launch.py",
                        {
                            "padding_m": LaunchConfiguration("padding_m"),
                            "nav2_start_delay_sec": LaunchConfiguration("nav2_start_delay_sec"),
                        },
                    )
                ],
            ),

            # 3. Runtime safety patch: force collision monitor and local costmap to use /scan_nav.
            # This fixes the current observed bad wiring:
            #   collision_monitor scan.topic = /scan
            #   source_timeout = 0.5
            #   transform_tolerance = 0.5
            TimerAction(
                period=LaunchConfiguration("param_fix_delay_sec"),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            "ros2 param set /collision_monitor scan.topic /scan_nav || true; "
                            "ros2 param set /collision_monitor source_timeout 2.0 || true; "
                            "ros2 param set /collision_monitor transform_tolerance 1.0 || true; "
                            "ros2 param set /local_costmap/local_costmap obstacle_layer.scan.topic /scan_nav || true; "
                            "ros2 param set /local_costmap/local_costmap transform_tolerance 1.0 || true; "
                            "echo '[bringup] collision_monitor params:'; "
                            "ros2 param get /collision_monitor scan.topic || true; "
                            "ros2 param get /collision_monitor source_timeout || true; "
                            "ros2 param get /collision_monitor transform_tolerance || true",
                        ],
                        output="screen",
                    )
                ],
            ),

            # 4. Passive object explorer. It does perception/frontiers, but does not own navigation.
            TimerAction(
                period=LaunchConfiguration("perception_delay_sec"),
                actions=[
                    include_launch(
                        "go2_object_explorer",
                        "frontier_object_explore.launch.py",
                        {
                            "image_topic": "/camera/image_raw",
                            "camera_info_topic": "/camera/camera_info",
                            "scan_topic": "/scan_nav",
                            "map_topic": "/map",
                            "cmd_vel_topic": "/cmd_vel_omi",
                            "yolo_model": "yolov8n.pt",
                            "yolo_conf": LaunchConfiguration("yolo_conf"),
                            "enable_sam2": "false",
                            "auto_navigate": "false",
                            "frontier_camera_map_mode": "fallback_and",
                            "use_ollama_frontier_selector": "false",
                            "local_scan_duration_sec": "6.0",
                            "search_timeout_sec": "300.0",
                            "frontier_goal_timeout_sec": "120.0",
                        },
                    )
                ],
            ),

            # 5. Deterministic safe frontier filter.
            # MRKL should consume this, not raw /object_explorer/frontiers.
            TimerAction(
                period=LaunchConfiguration("safe_frontier_delay_sec"),
                actions=[
                    include_launch(
                        "go2_object_explorer",
                        "safe_frontier_filter.launch.py",
                        {
                            "map_topic": LaunchConfiguration("safe_frontier_map_topic"),
                            "marker_topic": "/mrkl_explorer/safe_frontiers",
                            "json_topic": "/mrkl_explorer/safe_frontier_candidates",
                            "footprint_radius_m": LaunchConfiguration("safe_frontier_footprint_radius_m"),
                            "max_clutter_fraction": LaunchConfiguration("safe_frontier_max_clutter_fraction"),
                            "max_frontiers": "8",
                        },
                        condition=IfCondition(launch_safe_frontiers),
                    )
                ],
            ),

            # 6. Web dashboard.
            include_launch(
                "go2_object_explorer",
                "topic_web_dashboard.launch.py",
                {
                    "port": "8766",
                    "topic_allow_regex": "(/rosout|/mrkl_explorer/.*|/object_explorer/.*|/cmd_vel.*|/scan.*|/map.*|/plan|/goal_pose|/tf|/collision_monitor.*|/global_costmap/.*|/local_costmap/.*)",
                    "topic_deny_regex": "^/parameter_events$",
                    "max_messages_per_topic": "150",
                    "max_global_events": "500",
                },
                condition=IfCondition(launch_dashboard),
            ),

            # 7. RViz.
            TimerAction(
                period=LaunchConfiguration("rviz_delay_sec"),
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=[
                            "-d",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("go2_object_explorer"),
                                    "config",
                                    "object_explorer_demo.rviz",
                                ]
                            ),
                        ],
                        condition=IfCondition(launch_rviz),
                    )
                ],
            ),

            # 8. MRKL agent. It consumes /mrkl_explorer/safe_frontiers.
            TimerAction(
                period=LaunchConfiguration("mrkl_delay_sec"),
                actions=[
                    include_launch(
                        "go2_object_explorer",
                        "mrkl_explorer_agent.launch.py",
                        {
                            "ollama_url": LaunchConfiguration("ollama_url"),
                            "ollama_model": LaunchConfiguration("ollama_model"),
                            "ollama_timeout_sec": "45.0",
                            "ollama_keep_alive": "30m",
                            "ollama_num_ctx": "8192",
                            "ollama_num_predict": "512",
                            "ollama_temperature": "0.1",
                            "decision_period_sec": "4.0",
                            "scan_duration_sec": "5.0",
                            "object_approach_distance_m": "0.85",
                            "frontier_marker_topic": "/mrkl_explorer/safe_frontiers",
                            "frontier_standoff_m": "0.65",
                            "frontier_footprint_radius_m": "0.32",
                            "frontier_clutter_radius_m": "0.65",
                            "frontier_max_clutter_fraction": "0.40",
                        },
                        condition=IfCondition(launch_mrkl),
                    )
                ],
            ),
        ]
    )
