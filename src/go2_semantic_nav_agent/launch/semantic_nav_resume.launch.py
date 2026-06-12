from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import glob
import os
import tempfile

import yaml

from go2_semantic_nav_agent.session_store import SessionStore


def _latest_usable(root: str) -> str:
    store = SessionStore(root)
    session = store.latest_usable()
    if session is not None:
        return session.session_name
    raise RuntimeError(f'No usable session found in {root}')


def _build_semantic_nav2_params(source_path: str, scan_topic: str) -> str:
    with open(source_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    # GO2_SMAC_SMOOTH_PATH_FLAG
    # Runtime flag:
    #   GO2_SMAC_SMOOTH_PATH=0 disables SmacPlannerHybrid internal smoothing.
    #   GO2_SMAC_SMOOTH_PATH=1 enables it.
    smac_smooth_raw = os.environ.get('GO2_SMAC_SMOOTH_PATH', '1').strip().lower()
    smac_smooth_path = smac_smooth_raw not in ('0', 'false', 'no', 'off')

    planner_params = params.setdefault('planner_server', {}).setdefault('ros__parameters', {})
    grid = planner_params.setdefault('GridBased', {})
    grid['smooth_path'] = smac_smooth_path

    smac_smoother = grid.setdefault('smoother', {})
    smac_smoother['do_refinement'] = smac_smooth_path

    # These are ignored when smooth_path=false, but keep them minimal for clarity.
    if not smac_smooth_path:
        smac_smoother['max_iterations'] = 1
        smac_smoother['refinement_num'] = 1

    # GO2_ROS_SIDE_OBSTACLE_AVOIDANCE_PATCH
    # Live obstacle handling:
    #   /scan -> /scan_nav -> costmaps + collision_monitor -> /cmd_vel_out -> Go2 Sport MOVE.

    ctrl = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    ctrl['controller_frequency'] = 5.0

    progress = ctrl.setdefault('progress_checker', {})
    progress['plugin'] = 'nav2_controller::SimpleProgressChecker'
    progress['required_movement_radius'] = 0.03
    progress['movement_time_allowance'] = 120.0

    goal_checker = ctrl.setdefault('general_goal_checker', {})
    goal_checker['plugin'] = 'nav2_controller::SimpleGoalChecker'
    goal_checker['xy_goal_tolerance'] = 0.30
    goal_checker['yaw_goal_tolerance'] = 0.45
    goal_checker['stateful'] = True

    follow = ctrl.setdefault('FollowPath', {})
    follow['max_vel_x'] = 0.28
    follow['min_vel_x'] = 0.0
    follow['max_vel_y'] = 0.0
    follow['min_vel_y'] = 0.0
    follow['max_vel_theta'] = 0.45
    follow['min_speed_xy'] = 0.0
    follow['max_speed_xy'] = 0.28
    follow['min_speed_theta'] = 0.0
    follow['acc_lim_x'] = 0.35
    follow['acc_lim_y'] = 0.0
    follow['acc_lim_theta'] = 0.5
    follow['decel_lim_x'] = -0.45
    follow['decel_lim_y'] = 0.0
    follow['decel_lim_theta'] = -0.5
    follow['BaseObstacle.scale'] = 0.03

    local = params.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    local['rolling_window'] = True
    local['width'] = 6
    local['height'] = 6
    local['resolution'] = 0.05
    local['plugins'] = ['obstacle_layer', 'inflation_layer']

    local_obstacle = local.setdefault('obstacle_layer', {})
    local_obstacle['plugin'] = 'nav2_costmap_2d::ObstacleLayer'
    local_obstacle['enabled'] = True
    local_obstacle['footprint_clearing_enabled'] = True
    local_obstacle['observation_sources'] = 'scan'

    local_scan = local_obstacle.setdefault('scan', {})
    local_scan['topic'] = scan_topic
    local_scan['data_type'] = 'LaserScan'
    local_scan['clearing'] = True
    local_scan['marking'] = True
    local_scan['obstacle_min_range'] = 0.30
    local_scan['obstacle_max_range'] = 2.5
    local_scan['raytrace_min_range'] = 0.20
    local_scan['raytrace_max_range'] = 3.5
    local_scan['max_obstacle_height'] = 2.0
    local_scan['inf_is_valid'] = True
    local_scan['observation_persistence'] = 0.5
    local_scan['expected_update_rate'] = 0.0

    local_inflation = local.setdefault('inflation_layer', {})
    local_inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
    local_inflation['inflation_radius'] = 0.45
    local_inflation['cost_scaling_factor'] = 3.0

    global_cm = params.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    # GO2_GLOBAL_LIVE_OBSTACLES_FLAG
    # GO2_GLOBAL_LIVE_OBSTACLES=0 -> stable static-map global planning
    # GO2_GLOBAL_LIVE_OBSTACLES=1 -> global rerouting using live scan obstacles
    global_live_raw = os.environ.get('GO2_GLOBAL_LIVE_OBSTACLES', '1').strip().lower()
    global_live_obstacles = global_live_raw not in ('0', 'false', 'no', 'off')

    if global_live_obstacles:
        global_cm['plugins'] = ['static_layer', 'obstacle_layer', 'inflation_layer']
    else:
        global_cm['plugins'] = ['static_layer', 'inflation_layer']

    global_obstacle = global_cm.setdefault('obstacle_layer', {})
    global_obstacle['plugin'] = 'nav2_costmap_2d::ObstacleLayer'
    global_obstacle['enabled'] = True
    global_obstacle['footprint_clearing_enabled'] = True
    global_obstacle['observation_sources'] = 'scan'

    global_scan = global_obstacle.setdefault('scan', {})
    global_scan['topic'] = scan_topic
    global_scan['data_type'] = 'LaserScan'
    global_scan['clearing'] = True
    global_scan['marking'] = True
    global_scan['obstacle_min_range'] = 0.45
    global_scan['obstacle_max_range'] = 2.5
    global_scan['raytrace_min_range'] = 0.30
    global_scan['raytrace_max_range'] = 3.5
    global_scan['max_obstacle_height'] = 2.0
    global_scan['inf_is_valid'] = True
    global_scan['observation_persistence'] = 2.0
    global_scan['expected_update_rate'] = 0.0

    global_inflation = global_cm.setdefault('inflation_layer', {})
    global_inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
    global_inflation['inflation_radius'] = 0.52
    global_inflation['cost_scaling_factor'] = 3.0

    cm = params.setdefault('collision_monitor', {}).setdefault('ros__parameters', {})
    cm['enabled'] = True
    cm['base_frame_id'] = 'base_link'
    cm['odom_frame_id'] = 'odom'
    cm['cmd_vel_in_topic'] = 'cmd_vel_nav'
    cm['cmd_vel_out_topic'] = 'cmd_vel_out'
    cm['state_topic'] = 'collision_monitor_state'
    cm['transform_tolerance'] = 2.0
    cm['source_timeout'] = 2.5
    cm['stop_pub_timeout'] = 2.0
    cm['polygons'] = ['StopPolygon', 'SlowdownPolygon']
    cm['observation_sources'] = ['scan']

    stop = cm.setdefault('StopPolygon', {})
    stop['type'] = 'polygon'
    stop['points'] = '[[0.68, 0.32], [0.68, -0.32], [-0.25, -0.32], [-0.25, 0.32]]'
    stop['action_type'] = 'stop'
    stop['min_points'] = 3
    stop['visualize'] = True
    stop['polygon_pub_topic'] = 'stop_polygon'

    slow = cm.setdefault('SlowdownPolygon', {})
    slow['type'] = 'polygon'
    slow['points'] = '[[1.00, 0.42], [1.00, -0.42], [-0.35, -0.42], [-0.35, 0.42]]'
    slow['action_type'] = 'slowdown'
    slow['slowdown_ratio'] = 0.85
    slow['min_points'] = 3
    slow['visualize'] = True
    slow['polygon_pub_topic'] = 'slowdown_polygon'

    cm_scan = cm.setdefault('scan', {})
    cm_scan['type'] = 'scan'
    cm_scan['topic'] = scan_topic
    cm_scan['enabled'] = True
    try:
        params.setdefault('amcl', {}).setdefault('ros__parameters', {})['scan_topic'] = scan_topic
    except Exception:
        pass

    try:
        params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = scan_topic
    except Exception:
        pass
    try:
        params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = scan_topic
    except Exception:
        pass
    try:
        params['collision_monitor']['ros__parameters']['scan']['topic'] = scan_topic
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
    nav2_start_delay_sec = float(LaunchConfiguration('nav2_start_delay_sec').perform(context))
    scan_input_topic = LaunchConfiguration('scan_input_topic').perform(context).strip() or '/scan'
    scan_nav_topic = LaunchConfiguration('scan_nav_topic').perform(context).strip() or '/scan_nav'
    scan_frame_id = LaunchConfiguration('scan_frame_id').perform(context).strip() or 'base_link'
    scan_stamp_offset_sec = float(LaunchConfiguration('scan_stamp_offset_sec').perform(context))
    if not session_name:
        session_name = _latest_usable(session_root)
    store = SessionStore(session_root)
    session = store.for_name(session_name)
    if not os.path.isfile(session.session_yaml_path):
        raise RuntimeError(f'Resume session not found: {session.session_yaml_path}')
    session_dir = os.path.join(session_root, session_name)
    with open(session.session_yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}
    map_yaml = store.resolve_map_yaml(session)
    package_share = FindPackageShare('go2_semantic_nav_agent')
    go2_share = FindPackageShare('go2_robot_sdk')
    rviz_cfg = PathJoinSubstitution([package_share, 'config', 'semantic_nav.rviz'])
    amcl_cfg = PathJoinSubstitution([package_share, 'config', 'amcl_params.yaml'])
    nav2_launch = PathJoinSubstitution([go2_share, 'launch', 'navigation_no_docking.launch.py'])
    nav2_params = _build_semantic_nav2_params(
        os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            'config',
            'nav2_params.yaml',
        ),
        scan_nav_topic,
    )
    if map_yaml is None:
        configured = str(meta.get('map_yaml', '') or '').strip() or os.path.join(session_dir, 'map.yaml')
        raise RuntimeError(
            f'Resume session "{session_name}" has no usable saved map. '
            f'Checked configured path "{configured}" and session-local map files in "{session_dir}".'
        )
    print(f'[semantic_nav_resume] using session={session_name} map={map_yaml}')
    nodes = [
        Node(
            package='go2_semantic_nav_agent',
            executable='scan_retimestamp_node',
            name='scan_retimestamp_node',
            output='screen',
            parameters=[{
                'input_topic': scan_input_topic,
                'output_topic': scan_nav_topic,
                'frame_id': scan_frame_id,
                'stamp_offset_sec': scan_stamp_offset_sec,
            }],
        ),
        Node(package='nav2_map_server', executable='map_server', name='resume_map_server', output='screen', parameters=[{'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl_cfg, {'scan_topic': scan_nav_topic}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='resume_map_lifecycle_manager', output='screen', parameters=[{
            'autostart': True,
            'node_names': ['resume_map_server', 'amcl'],
        }]),
    ]
    delayed_resume_actions = [
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
        ),
        Node(package='go2_semantic_nav_agent', executable='semantic_nav_node', name='semantic_nav_node', output='screen', parameters=[{
            'mode': 'resume',
            'session_root': session_root,
            'session_name': session_name,
            'auto_save_places': False,
            'auto_save_use_vlm': False,
            'restore_spawn_on_start': restore_spawn_on_start,
            'allow_manual_initialpose_override': True,
            'scan_topic': scan_nav_topic,
        }]),
    ]
    if rviz or rviz2:
        delayed_resume_actions.append(
            Node(package='rviz2', executable='rviz2', name='semantic_nav_rviz2', output='screen', arguments=['-d', rviz_cfg], additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'})
        )
    nodes.append(TimerAction(period=nav2_start_delay_sec, actions=delayed_resume_actions))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz2', default_value='false'),
        DeclareLaunchArgument('restore_spawn_on_start', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay_sec', default_value='2.5'),
        DeclareLaunchArgument('scan_input_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_nav_topic', default_value='/scan_nav'),
        DeclareLaunchArgument('scan_frame_id', default_value='base_link'),
        DeclareLaunchArgument('scan_stamp_offset_sec', default_value='0.30'),
        OpaqueFunction(function=launch_setup),
    ])
