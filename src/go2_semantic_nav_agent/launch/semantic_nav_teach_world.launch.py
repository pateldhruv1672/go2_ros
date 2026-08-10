from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _reset_runtime_db(context):
    if not _truthy(LaunchConfiguration("reset_object_map_db").perform(context)):
        return []
    path = os.path.expanduser(LaunchConfiguration("object_map_db").perform(context))
    for suffix in ("", "-wal", "-shm"):
        candidate = path + suffix
        try:
            if os.path.exists(candidate):
                os.unlink(candidate)
                print(f"[semantic_nav_teach_world] removed runtime object DB {candidate}", flush=True)
        except OSError as exc:
            raise RuntimeError(f"Could not reset runtime object DB {candidate}: {exc}") from exc
    return []


def generate_launch_description():
    session_root = LaunchConfiguration("session_root")
    camera_topic = LaunchConfiguration("camera_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    semantic_teach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("go2_semantic_nav_agent"), "launch", "semantic_nav_teach.launch.py"
        ])),
        launch_arguments={
            "map_label": LaunchConfiguration("map_label"),
            "session_root": session_root,
            "auto_save_places": "false",
            "auto_save_use_vlm": "false",
            "clear_places_on_start": LaunchConfiguration("clear_places_on_start"),
            "save_map_on_shutdown": "true",
            "semantic_rviz": LaunchConfiguration("semantic_rviz"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_label", default_value="digital_twin_lab"),
        DeclareLaunchArgument("session_root", default_value="~/.ros/go2_semantic_nav_sessions"),
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("yolo_model", default_value=os.path.expanduser("~/.cache/sparky_models/yolov8n.pt")),
        DeclareLaunchArgument("sam2_model", default_value=os.path.expanduser("~/.cache/sparky_models/sam2_t.pt")),
        DeclareLaunchArgument("enable_sam2", default_value="true"),
        DeclareLaunchArgument("yolo_conf", default_value="0.25"),
        DeclareLaunchArgument("inference_period_sec", default_value="0.20"),
        DeclareLaunchArgument("min_confirmations", default_value="2"),
        DeclareLaunchArgument("rviz_publish_tentative_objects", default_value="true"),
        DeclareLaunchArgument("object_map_db", default_value="~/.ros/go2_sysnav_vln/teach_runtime_object_map.sqlite3"),
        DeclareLaunchArgument("reset_object_map_db", default_value="true"),
        DeclareLaunchArgument("clear_places_on_start", default_value="false"),
        DeclareLaunchArgument("semantic_rviz", default_value="true"),
        DeclareLaunchArgument("enable_vlm_backup", default_value="false"),
        # SPARKY_BACKGROUND_TEACH_CHECKPOINTS_V2
        DeclareLaunchArgument("enable_background_checkpoints", default_value="true"),
        DeclareLaunchArgument("background_checkpoint_period_sec", default_value="10.0"),
        DeclareLaunchArgument("vlm_provider", default_value="ollama"),
        DeclareLaunchArgument("vlm_model", default_value="gemma4:e4b"),

        OpaqueFunction(function=_reset_runtime_db),
        semantic_teach,
        Node(
            package="go2_object_explorer",
            executable="fast_sam2_tracker_overlay_node",
            name="go2_teach_yolo_sam2",
            output="screen",
            parameters=[{
                "image_topic": camera_topic,
                "device": LaunchConfiguration("device"),
                "yolo_model": LaunchConfiguration("yolo_model"),
                "sam2_model": LaunchConfiguration("sam2_model"),
                "enable_sam2": LaunchConfiguration("enable_sam2"),
                "yolo_conf": LaunchConfiguration("yolo_conf"),
                "inference_period_sec": LaunchConfiguration("inference_period_sec"),
                "detections_topic": "/object_explorer/sam2_detections",
                "annotated_image_topic": "/object_explorer/annotated_image",
            }],
        ),
        Node(
            package="go2_sysnav_vln",
            executable="pose_aware_object_mapper",
            name="go2_teach_pose_aware_object_mapper",
            output="screen",
            parameters=[{
                "detection_topics": ["/object_explorer/sam2_detections"],
                "scan_topic": scan_topic,
                "camera_info_topic": camera_info_topic,
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "min_confirmations": LaunchConfiguration("min_confirmations"),
                "publish_tentative": LaunchConfiguration("rviz_publish_tentative_objects"),
                "database_path": LaunchConfiguration("object_map_db"),
                "publish_topic": "/go2_vln/object_map",
                "marker_topic": "/go2_vln/object_markers",
            }],
        ),
        TimerAction(period=2.5, actions=[
            Node(
                package="go2_memory_core",
                executable="world_object_memory_node",
                name="go2_teach_world_object_memory",
                output="screen",
                parameters=[{
                    "session_root": session_root,
                    "session_name": "__latest_created__",
                    "camera_topic": camera_topic,
                    "detector_topic": "/object_explorer/sam2_detections",
                    "object_map_topic": "/go2_vln/object_map",
                    "ingest_historical_mapper_objects": False,
                    "enable_graph_memory": True,
                    "enable_vector_memory": True,
                    "enable_voxel_memory": True,
                }],
            ),
        ]),
        TimerAction(period=2.8, actions=[
            Node(
                package="go2_memory_core",
                executable="pose_snapshot_node",
                name="go2_teach_background_pose_memory",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_background_checkpoints")),
                parameters=[{
                    "session_root": session_root,
                    "session_name": "__latest_created__",
                    "map_frame": LaunchConfiguration("map_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "auto_write": True,
                    "write_period_sec": LaunchConfiguration("background_checkpoint_period_sec"),
                }],
            ),
        ]),
        TimerAction(period=3.0, actions=[
            Node(
                package="go2_memory_core",
                executable="vlm_checkpoint_node",
                name="go2_teach_vlm_backup",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_vlm_backup")),
                parameters=[{
                    "session_root": session_root,
                    "session_name": "__latest_created__",
                    "camera_topic": camera_topic,
                    "auto_write_checkpoints": False,
                    "write_period_sec": 60.0,
                    "vlm_provider": LaunchConfiguration("vlm_provider"),
                    "vlm_model": LaunchConfiguration("vlm_model"),
                }],
            ),
        ]),
    ])
