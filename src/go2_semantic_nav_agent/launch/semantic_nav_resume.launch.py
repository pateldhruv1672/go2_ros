from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import glob
import os
import tempfile

import yaml


def _latest_usable(root: str) -> str:
    candidates = sorted(glob.glob(os.path.join(root, '*')), key=os.path.getmtime, reverse=True)
    for d in candidates:
        if os.path.isdir(d) and all(os.path.isfile(os.path.join(d, f)) for f in ('map.yaml', 'map.pgm', 'places.yaml', 'session.yaml')):
            return os.path.basename(d)
    raise RuntimeError(f'No usable session found in {root}')


def _build_semantic_nav2_params(source_path: str) -> str:
    with open(source_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    try:
        params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = '/scan'
    except Exception:
        pass
    try:
        params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = '/scan'
    except Exception:
        pass
    try:
        params['collision_monitor']['ros__parameters']['scan']['topic'] = '/scan'
    except Exception:
        pass

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='semantic_nav2_params_', delete=False)
    with tmp:
        yaml.safe_dump(params, tmp, sort_keys=False)
    return tmp.name


def launch_setup(context, *args, **kwargs):
    session_root = os.path.expanduser(LaunchConfiguration('session_root').perform(context))
    session_name = LaunchConfiguration('session_name').perform(context).strip()
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes')
    rviz2 = LaunchConfiguration('rviz2').perform(context).lower() in ('1', 'true', 'yes')
    restore_spawn_on_start = LaunchConfiguration('restore_spawn_on_start').perform(context).lower() in ('1', 'true', 'yes')
    if not session_name:
        session_name = _latest_usable(session_root)
    session_dir = os.path.join(session_root, session_name)
    session_yaml = os.path.join(session_dir, 'session.yaml')
    with open(session_yaml, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}
    map_yaml = meta.get('map_yaml') or os.path.join(session_dir, 'map.yaml')
    map_exists = os.path.isfile(map_yaml)
    package_share = FindPackageShare('go2_semantic_nav_agent')
    go2_share = FindPackageShare('go2_robot_sdk')
    rviz_cfg = PathJoinSubstitution([package_share, 'config', 'semantic_nav.rviz'])
    amcl_cfg = PathJoinSubstitution([package_share, 'config', 'amcl_params.yaml'])
    nav2_launch = PathJoinSubstitution([go2_share, 'launch', 'navigation_no_docking.launch.py'])
    nav2_params = _build_semantic_nav2_params(os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml',
    ))
    if not map_exists:
        print(f'[semantic_nav_resume] session map not found at {map_yaml}, falling back to live /map topic')
    nodes = []
    if map_exists:
        nodes += [
            Node(package='nav2_map_server', executable='map_server', name='resume_map_server', output='screen', parameters=[{'yaml_filename': map_yaml}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl_cfg]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='resume_map_lifecycle_manager', output='screen', parameters=[{
                'autostart': True,
                'node_names': ['resume_map_server', 'amcl'],
            }]),
        ]
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': nav2_params,
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        )
    )
    nodes += [
        Node(package='go2_semantic_nav_agent', executable='semantic_nav_node', name='semantic_nav_node', output='screen', parameters=[{
            'mode': 'resume',
            'session_root': session_root,
            'session_name': session_name,
            'auto_save_places': False,
            'auto_save_use_vlm': False,
            'restore_spawn_on_start': restore_spawn_on_start,
            'allow_manual_initialpose_override': True,
            'scan_topic': '/scan',
        }]),
    ]
    if rviz or rviz2:
        nodes.append(Node(package='rviz2', executable='rviz2', name='semantic_nav_rviz2', output='screen', arguments=['-d', rviz_cfg], additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz2', default_value='false'),
        DeclareLaunchArgument('restore_spawn_on_start', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
