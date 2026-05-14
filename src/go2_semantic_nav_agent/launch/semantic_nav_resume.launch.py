from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob
import os
import yaml


def _latest_usable(root: str) -> str:
    candidates = sorted(glob.glob(os.path.join(root, '*')), key=os.path.getmtime, reverse=True)
    for d in candidates:
        if os.path.isdir(d) and all(os.path.isfile(os.path.join(d, f)) for f in ('map.yaml', 'map.pgm', 'places.yaml', 'session.yaml')):
            return os.path.basename(d)
    raise RuntimeError(f'No usable session found in {root}')


def launch_setup(context, *args, **kwargs):
    session_root = os.path.expanduser(LaunchConfiguration('session_root').perform(context))
    session_name = LaunchConfiguration('session_name').perform(context).strip()
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes')
    rviz2 = LaunchConfiguration('rviz2').perform(context).lower() in ('1', 'true', 'yes')
    if not session_name:
        session_name = _latest_usable(session_root)
    session_dir = os.path.join(session_root, session_name)
    session_yaml = os.path.join(session_dir, 'session.yaml')
    with open(session_yaml, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}
    map_yaml = meta.get('map_yaml') or os.path.join(session_dir, 'map.yaml')
    pkg_share = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'install', 'go2_semantic_nav_agent', 'share', 'go2_semantic_nav_agent')
    rviz_cfg = os.path.join(pkg_share, 'config', 'semantic_nav.rviz')
    amcl_cfg = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='resume_map_server', output='screen', parameters=[{'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl_cfg]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='resume_map_lifecycle_manager', output='screen', parameters=[{
            'autostart': True,
            'node_names': ['resume_map_server', 'amcl'],
        }]),
        Node(package='go2_semantic_nav_agent', executable='scan_retimestamp_node', name='scan_retimestamp_node', output='screen'),
        Node(package='go2_semantic_nav_agent', executable='semantic_nav_node', name='semantic_nav_node', output='screen', parameters=[{
            'mode': 'resume',
            'session_root': session_root,
            'session_name': session_name,
            'auto_save_places': False,
            'auto_save_use_vlm': False,
            'restore_spawn_on_start': True,
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
        OpaqueFunction(function=launch_setup),
    ])
