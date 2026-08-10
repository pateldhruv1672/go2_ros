from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _maybe_reset_object_db(context, *args, **kwargs):
    raw = LaunchConfiguration('reset_object_map_db').perform(context).strip().lower()
    if raw not in {'1', 'true', 'yes', 'on'}:
        return []
    db_raw = LaunchConfiguration('object_map_db').perform(context)
    db = Path(os.path.expanduser(os.path.expandvars(db_raw)))
    for candidate in (db, Path(str(db) + '-wal'), Path(str(db) + '-shm')):
        try:
            if candidate.exists() or candidate.is_symlink():
                candidate.unlink()
                print(f'[sparky_teach_world] removed stale object-map DB artifact: {candidate}')
        except Exception as exc:
            print(f'[sparky_teach_world] WARNING: could not remove {candidate}: {exc}')
    db.parent.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    home = EnvironmentVariable('HOME')

    robot_launch = PathJoinSubstitution([
        FindPackageShare('go2_robot_sdk'), 'launch', 'robot.launch.py'
    ])
    teach_overlay_launch = PathJoinSubstitution([
        FindPackageShare('go2_agentic_system'), 'launch', 'teach_world_memory.launch.py'
    ])

    args = [
        DeclareLaunchArgument('map_label', default_value='digital_twin_lab'),
        DeclareLaunchArgument(
            'session_root',
            default_value=PathJoinSubstitution([home, '.ros', 'go2_semantic_nav_sessions']),
        ),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=PathJoinSubstitution([home, '.cache', 'sparky_models', 'yolov8n.pt']),
        ),
        DeclareLaunchArgument(
            'sam2_model',
            default_value=PathJoinSubstitution([home, '.cache', 'sparky_models', 'sam2_t.pt']),
        ),
        DeclareLaunchArgument('enable_sam2', default_value='true'),
        DeclareLaunchArgument('yolo_conf', default_value='0.25'),
        DeclareLaunchArgument('inference_period_sec', default_value='0.20'),
        DeclareLaunchArgument('min_confirmations', default_value='2'),
        DeclareLaunchArgument(
            'object_map_db',
            default_value=PathJoinSubstitution([home, '.ros', 'go2_sysnav_vln', 'teach_object_map.sqlite3']),
        ),
        DeclareLaunchArgument(
            'reset_object_map_db',
            default_value='false',
            description='Delete the configured mapper DB before this teach run. Use true for a brand-new map/session.',
        ),
        DeclareLaunchArgument('vlm_provider', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='gemma4:e4b'),
        DeclareLaunchArgument('vlm_base_url', default_value='http://127.0.0.1:11434/api/chat'),
        DeclareLaunchArgument('vlm_checkpoint_period_sec', default_value='8.0'),
        DeclareLaunchArgument('semantic_rviz', default_value='true'),
        DeclareLaunchArgument('teach_delay_sec', default_value='6.0'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('pointcloud_aggregator', default_value='false'),
    ]

    # Teach-mode ownership contract:
    # - robot.launch owns driver/sensors/TF + ONE slam_toolbox map->odom authority
    # - Nav2/AMCL are OFF during teach
    # - robot RViz is OFF; semantic teach overlay owns the single RViz instance
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'slam': 'true',
            'nav2': 'false',
            'rviz2': 'false',
            'foxglove': LaunchConfiguration('foxglove'),
            'joystick': 'false',
            'teleop': 'false',
            'pointcloud_aggregator': LaunchConfiguration('pointcloud_aggregator'),
        }.items(),
    )

    teach_overlay = TimerAction(
        period=LaunchConfiguration('teach_delay_sec'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teach_overlay_launch),
                launch_arguments={
                    'map_label': LaunchConfiguration('map_label'),
                    'session_root': LaunchConfiguration('session_root'),
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'device': LaunchConfiguration('device'),
                    'yolo_model': LaunchConfiguration('yolo_model'),
                    'sam2_model': LaunchConfiguration('sam2_model'),
                    'enable_sam2': LaunchConfiguration('enable_sam2'),
                    'yolo_conf': LaunchConfiguration('yolo_conf'),
                    'inference_period_sec': LaunchConfiguration('inference_period_sec'),
                    'min_confirmations': LaunchConfiguration('min_confirmations'),
                    'object_map_db': LaunchConfiguration('object_map_db'),
                    'vlm_provider': LaunchConfiguration('vlm_provider'),
                    'vlm_model': LaunchConfiguration('vlm_model'),
                    'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                    'vlm_checkpoint_period_sec': LaunchConfiguration('vlm_checkpoint_period_sec'),
                    'semantic_rviz': LaunchConfiguration('semantic_rviz'),
                }.items(),
            )
        ],
    )

    return LaunchDescription(args + [OpaqueFunction(function=_maybe_reset_object_db), robot, teach_overlay])
