from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package, filename, arguments=None, condition=None):
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
    launch_overlay = LaunchConfiguration("launch_overlay")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard = LaunchConfiguration("launch_dashboard")

    base = include_launch(
        "go2_robot_sdk",
        "robot.launch.py",
        {
            "foxglove": "false",
            "slam": "false",
            "nav2": "false",
            "rviz2": "false",
        },
    )

    live_nav = TimerAction(
        period=LaunchConfiguration("nav_overlay_delay_sec"),
        actions=[
            include_launch(
                "go2_object_explorer",
                "explorer_live_nav.launch.py",
                {
                    "padding_m": LaunchConfiguration("padding_m"),
                    "nav2_start_delay_sec": LaunchConfiguration(
                        "nav2_start_delay_sec"
                    ),
                },
            )
        ],
    )

    annotated_overlay = TimerAction(
        period=LaunchConfiguration("perception_delay_sec"),
        actions=[
            include_launch(
                "go2_object_explorer",
                "fast_sam2_tracker_overlay.launch.py",
                {
                    "image_topic": "/camera/image_raw",
                    "annotated_image_topic": (
                        "/object_explorer/annotated_image"
                    ),
                    "detections_topic": (
                        "/object_explorer/sam2_detections"
                    ),
                    "device": LaunchConfiguration("device"),
                    "yolo_model": "yolov8n.pt",
                    "yolo_imgsz": "640",
                    "yolo_conf": LaunchConfiguration("yolo_conf"),
                    "max_detections": "8",
                    "target_classes": (
                        "person,chair,tv,bottle,laptop,"
                        "backpack,cup,book"
                    ),
                    "enable_sam2": LaunchConfiguration("enable_sam2"),
                    "sam2_model": "sam2_t.pt",
                    "sam2_imgsz": "384",
                    "sam2_every_n": "2",
                    "inference_period_sec": "0.12",
                    "track_ttl_sec": "2.0",
                },
                condition=IfCondition(launch_overlay),
            )
        ],
    )


    dashboard = include_launch(
        "go2_object_explorer",
        "topic_web_dashboard.launch.py",
        {
            "host": "0.0.0.0",
            "port": "8766",
            "auto_discover": "true",
            "topic_allow_regex": (
                "(/rosout|/go2_agent/.*|/mrkl_explorer/.*|"
                "/object_explorer/.*|/cmd_vel.*|/scan.*|/map.*|"
                "/plan|/goal_pose|/tf|/tf_static|"
                "/collision_monitor.*|/global_costmap/.*|"
                "/local_costmap/.*)"
            ),
            "topic_deny_regex": "^/parameter_events$",
            "max_messages_per_topic": "150",
            "max_global_events": "500",
        },
        condition=IfCondition(launch_dashboard),
    )

    rviz = TimerAction(
        period=LaunchConfiguration("rviz_delay_sec"),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="object_explorer_rviz2",
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
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("padding_m", default_value="3.0"),
            DeclareLaunchArgument(
                "nav_overlay_delay_sec",
                default_value="8.0",
            ),
            DeclareLaunchArgument(
                "nav2_start_delay_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "perception_delay_sec",
                default_value="12.0",
            ),
            DeclareLaunchArgument(
                "rviz_delay_sec",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "launch_overlay",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "launch_dashboard",
                default_value="true",
            ),
            DeclareLaunchArgument("device", default_value="cuda:0"),
            DeclareLaunchArgument("yolo_conf", default_value="0.60"),
            # Keep SAM2 off for the first safe Nav2 test.
            DeclareLaunchArgument("enable_sam2", default_value="false"),
            base,
            live_nav,
            annotated_overlay,
            dashboard,
            rviz,
        ]
    )
