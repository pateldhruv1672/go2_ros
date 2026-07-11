from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
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


def _bool_flag(value: str, default: bool = False) -> bool:
    raw = (value or '').strip().lower()
    if not raw:
        return default
    if raw in ('1', 'true', 'yes', 'on'):
        return True
    if raw in ('0', 'false', 'no', 'off'):
        return False
    return default


def _package_available(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _mppi_follow_path_params() -> dict:
    return {
        'plugin': 'nav2_mppi_controller::MPPIController',
        'time_steps': 30,
        'model_dt': 0.10,
        'batch_size': 1000,
        'iteration_count': 1,
        'vx_std': 0.18,
        'vy_std': 0.0,
        'wz_std': 0.18,
        'vx_max': 0.32,
        'vx_min': 0.0,
        'vy_max': 0.0,
        'wz_max': 0.40,
        'ax_max': 0.35,
        'ax_min': -0.30,
        'ay_max': 0.0,
        'ay_min': 0.0,
        'az_max': 0.45,
        'prune_distance': 1.2,
        'transform_tolerance': 1.0,
        'temperature': 0.3,
        'gamma': 0.015,
        'motion_model': 'DiffDrive',
        'diff_drive': {
            'plugin': 'mppi::DiffDriveMotionModel',
        },
        'visualize': False,
        'critic_index_to_visualize': 0,
        'publish_optimal_trajectory': False,
        'publish_critics_stats': False,
        'open_loop': False,
        'regenerate_noises': False,
        'sgf_order': 2,
        'TrajectoryVisualizer': {
            'trajectory_step': 5,
            'time_step': 3,
        },
        'TrajectoryValidator': {
            'plugin': 'mppi::DefaultOptimalTrajectoryValidator',
            'collision_lookahead_time': 2.0,
            'consider_footprint': True,
        },
        'critics': [
            'ConstraintCritic',
            'CostCritic',
            'GoalCritic',
            'GoalAngleCritic',
            'PathAlignCritic',
            'PathFollowCritic',
            'PathAngleCritic',
            'PreferForwardCritic',
            'VelocityDeadbandCritic',
        ],
        'ConstraintCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 4.0,
        },
        'GoalCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 5.0,
            'threshold_to_consider': 1.4,
        },
        'GoalAngleCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 3.0,
            'threshold_to_consider': 0.5,
        },
        'PreferForwardCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 5.0,
            'threshold_to_consider': 0.5,
        },
        'CostCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 2.0,
            'critical_cost': 300.0,
            'near_collision_cost': 253,
            'consider_footprint': True,
            'collision_cost': 1000000.0,
            'near_goal_distance': 1.0,
            'trajectory_point_step': 2,
        },
        'PathAlignCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 10.0,
            'max_path_occupancy_ratio': 0.20,
            'trajectory_point_step': 4,
            'threshold_to_consider': 0.5,
            'offset_from_furthest': 12,
            'use_path_orientations': False,
        },
        'PathFollowCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 6.0,
            'offset_from_furthest': 5,
            'threshold_to_consider': 1.4,
        },
        'PathAngleCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 2.0,
            'offset_from_furthest': 4,
            'threshold_to_consider': 0.5,
            'max_angle_to_furthest': 1.0,
            'mode': 0,
        },
        'VelocityDeadbandCritic': {
            'enabled': True,
            'cost_power': 1,
            'cost_weight': 60.0,
            'deadband_velocities': [0.09, 0.0, 0.08],
        },
    }


def _stvl_layer_params(pointcloud_topic: str) -> dict:
    return {
        'plugin': 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer',
        'enabled': True,
        'voxel_decay': 8.0,
        'decay_model': 0,
        'voxel_size': 0.05,
        'track_unknown_space': True,
        'unknown_threshold': 15,
        'mark_threshold': 0,
        'update_footprint_enabled': True,
        'combination_method': 1,
        'origin_z': 0.0,
        'publish_voxel_map': False,
        'transform_tolerance': 0.5,
        'mapping_mode': False,
        'map_save_duration': 60.0,
        'observation_sources': 'pointcloud',
        'pointcloud': {
            'data_type': 'PointCloud2',
            'topic': pointcloud_topic,
            'marking': True,
            'clearing': True,
            'obstacle_range': 3.0,
            'min_obstacle_height': 0.05,
            'max_obstacle_height': 1.5,
            'expected_update_rate': 0.0,
            'observation_persistence': 0.0,
            'inf_is_valid': False,
            'filter': 'voxel',
            'voxel_min_points': 0,
            'clear_after_reading': True,
            'max_z': 2.0,
            'min_z': 0.05,
            'vertical_fov_angle': 1.0,
            'horizontal_fov_angle': 6.283,
            'decay_acceleration': 10.0,
            'model_type': 0,
        },
    }


def _resolve_stvl_enabled(requested: str) -> bool:
    raw = (requested or 'auto').strip().lower()
    available = _package_available('spatio_temporal_voxel_layer')
    if raw in ('auto', 'detect'):
        return available
    enabled = _bool_flag(raw, default=False)
    if enabled and not available:
        raise RuntimeError(
            'stvl_enabled was requested, but spatio_temporal_voxel_layer is not installed. '
            'Install it with: sudo apt install ros-jazzy-spatio-temporal-voxel-layer'
        )
    return enabled


def _build_semantic_nav2_params(source_path: str, scan_topic: str, pointcloud_topic: str, stvl_enabled: str) -> str:
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

    bt = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    bt['navigators'] = ['navigate_to_pose', 'navigate_through_poses']
    bt['navigate_to_pose'] = {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'}
    bt['navigate_through_poses'] = {'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'}
    bt['default_nav_to_pose_bt_xml'] = '$(find-pkg-share nav2_bt_navigator)/behavior_trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml'
    bt['default_nav_through_poses_bt_xml'] = '$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml'
    bt['follow_point_bt_xml'] = '$(find-pkg-share nav2_bt_navigator)/behavior_trees/follow_point.xml'
    bt['odometry_calibration_bt_xml'] = '$(find-pkg-share nav2_bt_navigator)/behavior_trees/odometry_calibration.xml'
    bt['error_code_names'] = ['compute_path_error_code', 'follow_path_error_code']
    bt['wait_for_service_timeout'] = 5000
    bt['bond_heartbeat_period'] = 0.10

    ctrl = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    ctrl['controller_frequency'] = 10.0
    ctrl['costmap_update_timeout'] = 1.0
    ctrl['progress_checker_plugins'] = ['progress_checker']
    ctrl['current_progress_checker'] = 'progress_checker'
    ctrl.pop('progress_checker_plugin', None)
    ctrl['goal_checker_plugins'] = ['general_goal_checker']
    ctrl['current_goal_checker'] = 'general_goal_checker'
    ctrl['controller_plugins'] = ['FollowPath']
    ctrl['use_realtime_priority'] = False
    ctrl['enable_stamped_cmd_vel'] = False

    progress = ctrl.setdefault('progress_checker', {})
    progress['plugin'] = 'nav2_controller::PoseProgressChecker'
    progress['required_movement_radius'] = 0.005
    progress['required_movement_angle'] = 0.02
    progress['movement_time_allowance'] = 600.0

    goal_checker = ctrl.setdefault('general_goal_checker', {})
    goal_checker['plugin'] = 'nav2_controller::SimpleGoalChecker'
    goal_checker['xy_goal_tolerance'] = 0.30
    goal_checker['yaw_goal_tolerance'] = 0.45
    goal_checker['stateful'] = True

    ctrl['FollowPath'] = _mppi_follow_path_params()

    local = params.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    local['update_frequency'] = 5.0
    local['publish_frequency'] = 2.0
    local['rolling_window'] = True
    local['width'] = 5
    local['height'] = 5
    local['resolution'] = 0.05
    use_stvl = _resolve_stvl_enabled(stvl_enabled)
    local['plugins'] = ['stvl_layer', 'inflation_layer'] if use_stvl else ['obstacle_layer', 'inflation_layer']

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
    local_scan['observation_persistence'] = 0.0
    local_scan['expected_update_rate'] = 0.0

    local_inflation = local.setdefault('inflation_layer', {})
    local_inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
    # Must stay larger than the footprint circumscribed radius (~0.51 m) so
    # MPPI's SE2 collision checker can use the costmap potential field.
    local_inflation['inflation_radius'] = 0.65
    local_inflation['cost_scaling_factor'] = 3.0
    local['stvl_layer'] = _stvl_layer_params(pointcloud_topic)

    global_cm = params.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    # GO2_GLOBAL_LIVE_OBSTACLES_FLAG
    # GO2_GLOBAL_LIVE_OBSTACLES=0 -> stable static-map global planning
    # GO2_GLOBAL_LIVE_OBSTACLES=1 -> global rerouting using live scan obstacles
    global_live_raw = os.environ.get('GO2_GLOBAL_LIVE_OBSTACLES', '0').strip().lower()
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
    global_scan['observation_persistence'] = 0.0
    global_scan['expected_update_rate'] = 0.0

    global_inflation = global_cm.setdefault('inflation_layer', {})
    global_inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
    global_inflation['inflation_radius'] = 0.70
    global_inflation['cost_scaling_factor'] = 3.0

    cm = params.setdefault({}).setdefault('ros__parameters', {})
    cm['enabled'] = True
    cm['enable_stamped_cmd_vel'] = False
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
    stop['points'] = '[[0.45, 0.26], [0.45, -0.26], [-0.25, -0.26], [-0.25, 0.26]]'
    stop['action_type'] = 'stop'
    stop['min_points'] = 25
    stop['visualize'] = True
    stop['polygon_pub_topic'] = 'stop_polygon'

    slow = cm.setdefault('SlowdownPolygon', {})
    slow['type'] = 'polygon'
    slow['points'] = '[[0.50, 0.30], [0.50, -0.30], [-0.30, -0.32], [-0.30, 0.32]]'
    slow['action_type'] = 'slowdown'
    slow['slowdown_ratio'] = 0.20
    slow['min_points'] = 30
    slow['visualize'] = True
    slow['polygon_pub_topic'] = 'slowdown_polygon'

    cm_scan = cm.setdefault('scan', {})
    cm_scan['type'] = 'scan'
    cm_scan['topic'] = scan_topic
    cm_scan['enabled'] = True

    behavior = params.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
    behavior['enable_stamped_cmd_vel'] = False
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
    # Do not let RViz start before /navigate_to_pose exists.
    # External wrapper will launch RViz only after Nav2 is ready.
    if os.environ.get('GO2_RESUME_INTERNAL_RVIZ', '0').strip().lower() not in ('1', 'true', 'yes', 'on'):
        rviz = False
        rviz2 = False
    restore_spawn_on_start = LaunchConfiguration('restore_spawn_on_start').perform(context).lower() in ('1', 'true', 'yes')
    nav2_start_delay_sec = float(LaunchConfiguration('nav2_start_delay_sec').perform(context))
    scan_input_topic = LaunchConfiguration('scan_input_topic').perform(context).strip() or '/scan'
    scan_nav_topic = LaunchConfiguration('scan_nav_topic').perform(context).strip() or '/scan_nav'
    pointcloud_topic = LaunchConfiguration('pointcloud_topic').perform(context).strip() or '/point_cloud2'
    stvl_enabled = LaunchConfiguration('stvl_enabled').perform(context).strip() or 'auto'
    scan_frame_id = LaunchConfiguration('scan_frame_id').perform(context).strip() or 'base_link'
    scan_stamp_offset_sec = float(LaunchConfiguration('scan_stamp_offset_sec').perform(context))
    store = SessionStore(session_root)
    if not session_name or session_name.lower() in ('auto', 'latest', 'latest_usable'):
        session_name = _latest_usable(session_root)
    elif session_name.lower() == 'default':
        try:
            default_session = store.for_name(session_name)
            if store.resolve_map_yaml(default_session) is None:
                session_name = _latest_usable(session_root)
        except Exception:
            session_name = _latest_usable(session_root)
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
        pointcloud_topic,
        stvl_enabled,
    )
    if map_yaml is None:
        configured = str(meta.get('map_yaml', '') or '').strip() or os.path.join(session_dir, 'map.yaml')
        raise RuntimeError(
            f'Resume session "{session_name}" has no usable saved map. '
            f'Checked configured path "{configured}" and session-local map files in "{session_dir}".'
        )
    print(f'[semantic_nav_resume] using session={session_name} map={map_yaml}')
    nodes = [
        Node(package='go2_semantic_nav_agent', executable='scan_retimestamp_node', name='scan_retimestamp_node', output='screen', parameters=[{
            'input_topic': scan_input_topic,
            'output_topic': scan_nav_topic,
            'frame_id': scan_frame_id,
            'stamp_offset_sec': scan_stamp_offset_sec,
            'use_latest_tf_stamp': True,
            'tf_target_frame': 'odom',
            'tf_source_frame': scan_frame_id,
        }]),
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
            'fallback_cmd_topic': '/cmd_vel_nav',
            'initialpose_stamp_backdate_sec': 0.10,
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
        DeclareLaunchArgument('nav2_start_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('scan_input_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_nav_topic', default_value='/scan_nav'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/point_cloud2'),
        DeclareLaunchArgument('stvl_enabled', default_value='auto'),
        DeclareLaunchArgument('scan_frame_id', default_value='base_link'),
        DeclareLaunchArgument('scan_stamp_offset_sec', default_value='0.25'),
        OpaqueFunction(function=launch_setup),
    ])
