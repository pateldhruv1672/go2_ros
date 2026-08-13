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
    # Default safety consumers to the retimestamped navigation scan passed into this builder.
    # GO2_SAFETY_SCAN_TOPIC may still override this explicitly.
    safety_scan_topic = os.environ.get('GO2_SAFETY_SCAN_TOPIC', scan_topic).strip() or scan_topic
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

    # SPARKY_RPP_SAFE_TOUR_V13_11
    # Regulated Pure Pursuit is the single path-tracking authority. It directly
    # regulates speed from path curvature and goal approach; downstream nodes
    # only arbitrate/clamp and do not reinterpret the trajectory.
    ctrl = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    ctrl['controller_frequency'] = float(os.environ.get('GO2_RPP_CONTROLLER_HZ', '20.0'))
    ctrl['costmap_update_timeout'] = 1.0
    ctrl['min_x_velocity_threshold'] = 0.01
    ctrl['min_y_velocity_threshold'] = 0.01
    ctrl['min_theta_velocity_threshold'] = 0.01
    ctrl['failure_tolerance'] = 0.3
    ctrl['speed_limit_topic'] = 'speed_limit'
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
    progress['required_movement_radius'] = float(os.environ.get('GO2_RPP_PROGRESS_RADIUS_M', '0.04'))
    progress['required_movement_angle'] = float(os.environ.get('GO2_RPP_PROGRESS_ANGLE_RAD', '0.06'))
    progress['movement_time_allowance'] = float(os.environ.get('GO2_RPP_PROGRESS_TIMEOUT_SEC', '8.0'))

    goal_checker = ctrl.setdefault('general_goal_checker', {})
    goal_checker['plugin'] = 'nav2_controller::SimpleGoalChecker'
    goal_checker['xy_goal_tolerance'] = float(os.environ.get('GO2_RPP_XY_GOAL_TOLERANCE', '0.18'))
    goal_checker['yaw_goal_tolerance'] = float(os.environ.get('GO2_RPP_YAW_GOAL_TOLERANCE', '0.20'))
    goal_checker['stateful'] = True

    follow = ctrl.setdefault('FollowPath', {})
    follow.clear()
    # SPARKY_RPP_FLAT_NAMESPACE_V13_14
    # RotationShim's primary_controller is a plugin-class STRING. The child
    # controller is configured under the SAME FollowPath namespace in Jazzy.
    follow['plugin'] = 'nav2_rotation_shim_controller::RotationShimController'
    follow['primary_controller'] = 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'

    # Rotation shim: decisive initial/U-turn/final heading alignment.
    follow['angular_dist_threshold'] = float(os.environ.get('GO2_SHIM_ANGULAR_THRESHOLD', '0.35'))
    follow['angular_disengage_threshold'] = float(os.environ.get('GO2_SHIM_DISENGAGE_THRESHOLD', '0.12'))
    follow['forward_sampling_distance'] = float(os.environ.get('GO2_SHIM_FORWARD_SAMPLE_M', '0.25'))
    follow['rotate_to_heading_angular_vel'] = float(os.environ.get('GO2_SHIM_ROTATE_WZ', '0.75'))
    follow['max_angular_accel'] = float(os.environ.get('GO2_SHIM_MAX_ANGULAR_ACCEL', '2.20'))
    follow['simulate_ahead_time'] = float(os.environ.get('GO2_SHIM_SIM_AHEAD_SEC', '0.70'))
    follow['rotate_to_goal_heading'] = True
    follow['rotate_to_heading_once'] = False
    follow['closed_loop'] = True
    follow['use_path_orientations'] = False

    # RPP child controller parameters. These MUST be FollowPath.* (flat), not
    # FollowPath.primary_controller.*. 0.45 m/s is physically proven on Go2.
    follow['desired_linear_vel'] = float(os.environ.get('GO2_RPP_CRUISE_MPS', '0.48'))
    follow['lookahead_dist'] = float(os.environ.get('GO2_RPP_LOOKAHEAD_M', '0.35'))
    follow['min_lookahead_dist'] = float(os.environ.get('GO2_RPP_MIN_LOOKAHEAD_M', '0.20'))
    follow['max_lookahead_dist'] = float(os.environ.get('GO2_RPP_MAX_LOOKAHEAD_M', '0.55'))
    follow['lookahead_time'] = float(os.environ.get('GO2_RPP_LOOKAHEAD_TIME', '0.70'))
    follow['use_velocity_scaled_lookahead_dist'] = True
    follow['transform_tolerance'] = 0.50
    follow['min_approach_linear_velocity'] = float(os.environ.get('GO2_RPP_MIN_APPROACH_MPS', '0.30'))
    follow['approach_velocity_scaling_dist'] = float(os.environ.get('GO2_RPP_APPROACH_DIST_M', '0.65'))
    follow['use_regulated_linear_velocity_scaling'] = True
    follow['regulated_linear_scaling_min_radius'] = float(os.environ.get('GO2_RPP_MIN_RADIUS_M', '0.55'))
    follow['regulated_linear_scaling_min_speed'] = float(os.environ.get('GO2_RPP_MIN_REGULATED_MPS', '0.36'))
    follow['use_cost_regulated_linear_velocity_scaling'] = False
    follow['cost_scaling_dist'] = 0.30
    follow['cost_scaling_gain'] = 1.0
    follow['inflation_cost_scaling_factor'] = 3.0
    follow['use_rotate_to_heading'] = True
    follow['rotate_to_heading_min_angle'] = float(os.environ.get('GO2_RPP_ROTATE_MIN_ANGLE', '1.05'))
    # rotate_to_heading_angular_vel and max_angular_accel are shared with shim
    # because both controllers use the same FollowPath namespace.
    follow['allow_reversing'] = False
    follow['use_collision_detection'] = True
    follow['max_allowed_time_to_collision_up_to_carrot'] = float(os.environ.get('GO2_RPP_COLLISION_HORIZON_SEC', '0.60'))
    follow['max_robot_pose_search_dist'] = 10.0
    follow['use_fixed_curvature_lookahead'] = False
    follow['interpolate_curvature_after_goal'] = True

    # SPARKY_REACTIVE_DWB_V13_15
    # RPP was too slow/wide for the Go2 in this lab. Use the controller family
    # that previously moved the robot (DWB), but put RotationShim in front so
    # large initial/final heading errors become decisive in-place rotations.
    # Shorter rollout + higher control rate makes corrections more reactive.
    ctrl['controller_frequency'] = float(os.environ.get('GO2_NAV_CONTROLLER_HZ', '25.0'))
    ctrl['costmap_update_timeout'] = float(os.environ.get('GO2_NAV_COSTMAP_TIMEOUT_SEC', '0.50'))
    ctrl['min_x_velocity_threshold'] = 0.001
    ctrl['min_y_velocity_threshold'] = 0.001
    ctrl['min_theta_velocity_threshold'] = 0.001

    progress['required_movement_radius'] = float(os.environ.get('GO2_NAV_PROGRESS_RADIUS_M', '0.05'))
    progress['required_movement_angle'] = float(os.environ.get('GO2_NAV_PROGRESS_ANGLE_RAD', '0.08'))
    progress['movement_time_allowance'] = float(os.environ.get('GO2_NAV_PROGRESS_TIMEOUT_SEC', '8.0'))
    goal_checker['xy_goal_tolerance'] = float(os.environ.get('GO2_NAV_XY_TOLERANCE', '0.18'))
    goal_checker['yaw_goal_tolerance'] = float(os.environ.get('GO2_NAV_YAW_TOLERANCE', '0.20'))

    follow.clear()
    follow['plugin'] = 'nav2_rotation_shim_controller::RotationShimController'
    follow['primary_controller'] = 'dwb_core::DWBLocalPlanner'
    follow['angular_dist_threshold'] = float(os.environ.get('GO2_NAV_SHIM_TRIGGER_RAD', '0.30'))
    follow['angular_disengage_threshold'] = float(os.environ.get('GO2_NAV_SHIM_RELEASE_RAD', '0.08'))
    follow['forward_sampling_distance'] = float(os.environ.get('GO2_NAV_SHIM_SAMPLE_M', '0.20'))
    follow['rotate_to_heading_angular_vel'] = float(os.environ.get('GO2_NAV_SHIM_WZ', '0.80'))
    follow['max_angular_accel'] = float(os.environ.get('GO2_NAV_SHIM_ACC_THETA', '2.40'))
    follow['simulate_ahead_time'] = float(os.environ.get('GO2_NAV_SHIM_SIM_SEC', '0.45'))
    follow['rotate_to_goal_heading'] = True
    follow['rotate_to_heading_once'] = False
    follow['closed_loop'] = True
    follow['use_path_orientations'] = False

    # DWB values are placed in the same FollowPath namespace because Jazzy's
    # RotationShim configures its primary controller with the FollowPath name.
    follow['debug_trajectory_details'] = False
    follow['min_vel_x'] = 0.0
    follow['max_vel_x'] = float(os.environ.get('GO2_NAV_MAX_X', '0.50'))
    follow['min_vel_y'] = 0.0
    follow['max_vel_y'] = 0.0
    follow['max_vel_theta'] = float(os.environ.get('GO2_NAV_MAX_THETA', '0.90'))
    follow['min_speed_xy'] = float(os.environ.get('GO2_NAV_MIN_SPEED_XY', '0.30'))
    follow['max_speed_xy'] = follow['max_vel_x']
    follow['min_speed_theta'] = float(os.environ.get('GO2_NAV_MIN_SPEED_THETA', '0.25'))
    # High enough acceleration that DWB does not spend seconds commanding the
    # physically useless twitch range before reaching a usable Go2 stride.
    follow['acc_lim_x'] = float(os.environ.get('GO2_NAV_ACC_X', '2.00'))
    follow['acc_lim_y'] = 0.0
    follow['acc_lim_theta'] = float(os.environ.get('GO2_NAV_ACC_THETA', '3.00'))
    follow['decel_lim_x'] = -abs(float(os.environ.get('GO2_NAV_DECEL_X', '2.50')))
    follow['decel_lim_y'] = 0.0
    follow['decel_lim_theta'] = -abs(float(os.environ.get('GO2_NAV_DECEL_THETA', '3.20')))
    follow['vx_samples'] = int(os.environ.get('GO2_NAV_VX_SAMPLES', '10'))
    follow['vy_samples'] = 1
    follow['vtheta_samples'] = int(os.environ.get('GO2_NAV_VTHETA_SAMPLES', '24'))
    follow['sim_time'] = float(os.environ.get('GO2_NAV_SIM_TIME', '0.80'))
    follow['linear_granularity'] = 0.03
    follow['angular_granularity'] = 0.02
    follow['transform_tolerance'] = 0.5
    follow['trans_stopped_velocity'] = 0.05
    follow['theta_stopped_velocity'] = 0.08
    follow['short_circuit_trajectory_evaluation'] = True
    follow['stateful'] = True
    follow['critics'] = ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']
    follow['BaseObstacle.scale'] = 0.8
    follow['PathAlign.scale'] = 5.0
    follow['PathAlign.forward_point_distance'] = 0.22
    follow['PathDist.scale'] = 6.0
    follow['GoalAlign.scale'] = 4.0
    follow['GoalAlign.forward_point_distance'] = 0.22
    follow['GoalDist.scale'] = 8.0
    follow['RotateToGoal.scale'] = 20.0
    follow['RotateToGoal.slowing_factor'] = 4.0
    follow['RotateToGoal.lookahead_time'] = -1.0
    follow['Oscillation.scale'] = 1.0
    # END_SPARKY_REACTIVE_DWB_V13_15
    # SPARKY_STRICT_PATH_V13_16
    # SPARKY_JAZZY_ROTATION_SHIM_DWB_V13_16_5
    # ROS 2 Jazzy RotationShim configures its child controller with the SAME
    # plugin namespace ("FollowPath"). Thus primary_controller is a scalar
    # plugin class and DWB parameters live directly under FollowPath.
    ctrl['controller_frequency'] = 20.0
    ctrl['costmap_update_timeout'] = 0.5
    ctrl['min_x_velocity_threshold'] = 0.01
    ctrl['min_y_velocity_threshold'] = 0.01
    ctrl['min_theta_velocity_threshold'] = 0.01

    progress = ctrl.setdefault('progress_checker', {})
    progress['plugin'] = 'nav2_controller::PoseProgressChecker'
    progress['required_movement_radius'] = 0.05
    progress['required_movement_angle'] = 0.08
    progress['movement_time_allowance'] = 10.0

    goal_checker = ctrl.setdefault('general_goal_checker', {})
    goal_checker['plugin'] = 'nav2_controller::SimpleGoalChecker'
    goal_checker['xy_goal_tolerance'] = 0.18
    goal_checker['yaw_goal_tolerance'] = 0.20
    goal_checker['stateful'] = True

    strict_max_x = float(os.environ.get('SPARKY_NAV_STRICT_MAX_X', '0.375'))
    strict_max_theta = float(os.environ.get('SPARKY_NAV_STRICT_MAX_THETA', '0.80'))

    follow = ctrl.setdefault('FollowPath', {})
    follow.clear()

    # Rotation Shim
    follow['plugin'] = 'nav2_rotation_shim_controller::RotationShimController'
    follow['primary_controller'] = 'dwb_core::DWBLocalPlanner'
    follow['angular_dist_threshold'] = 0.30
    follow['angular_disengage_threshold'] = 0.08
    follow['forward_sampling_distance'] = 0.18
    follow['rotate_to_heading_angular_vel'] = 0.75
    follow['max_angular_accel'] = 2.40
    follow['max_cost_threshold'] = 254.0
    follow['simulate_ahead_time'] = 0.45
    follow['rotate_to_goal_heading'] = True
    follow['rotate_to_heading_once'] = False
    follow['closed_loop'] = True
    follow['use_path_orientations'] = False

    # DWB - SAME FollowPath namespace on ROS 2 Jazzy.
    follow['debug_trajectory_details'] = False
    follow['trajectory_generator_name'] = 'dwb_plugins::StandardTrajectoryGenerator'
    follow['min_vel_x'] = 0.0
    follow['max_vel_x'] = strict_max_x
    follow['min_vel_y'] = 0.0
    follow['max_vel_y'] = 0.0
    follow['max_vel_theta'] = strict_max_theta
    follow['min_speed_xy'] = 0.30
    follow['max_speed_xy'] = strict_max_x
    follow['min_speed_theta'] = 0.22

    follow['acc_lim_x'] = 1.20
    follow['acc_lim_y'] = 0.0
    follow['acc_lim_theta'] = 2.20
    follow['decel_lim_x'] = -1.50
    follow['decel_lim_y'] = 0.0
    follow['decel_lim_theta'] = -2.50

    follow['vx_samples'] = 14
    follow['vy_samples'] = 1
    follow['vtheta_samples'] = 28
    follow['sim_time'] = 0.90
    follow['linear_granularity'] = 0.03
    follow['angular_granularity'] = 0.02
    follow['transform_tolerance'] = 0.5
    follow['trans_stopped_velocity'] = 0.05
    follow['short_circuit_trajectory_evaluation'] = True
    follow['stateful'] = True

    follow['critics'] = [
        'RotateToGoal', 'Oscillation', 'BaseObstacle',
        'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'
    ]

    # Strong path adherence.
    follow['BaseObstacle.scale'] = 1.0
    follow['PathAlign.scale'] = 32.0
    follow['PathAlign.forward_point_distance'] = 0.15
    follow['PathDist.scale'] = 40.0
    follow['GoalAlign.scale'] = 16.0
    follow['GoalAlign.forward_point_distance'] = 0.15
    follow['GoalDist.scale'] = 18.0
    follow['RotateToGoal.scale'] = 32.0
    follow['RotateToGoal.slowing_factor'] = 5.0
    follow['RotateToGoal.lookahead_time'] = -1.0
    follow['Oscillation.scale'] = 1.0

    local = params.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    local['update_frequency'] = float(os.environ.get('GO2_LOCAL_COSTMAP_HZ', '10.0'))
    local['publish_frequency'] = float(os.environ.get('GO2_LOCAL_COSTMAP_PUB_HZ', '5.0'))
    local['rolling_window'] = True
    local['width'] = 5
    local['height'] = 5
    local['resolution'] = 0.05
    use_stvl = _resolve_stvl_enabled(stvl_enabled)
    local['plugins'] = ['obstacle_layer', 'inflation_layer']

    local_obstacle = local.setdefault('obstacle_layer', {})
    local_obstacle['plugin'] = 'nav2_costmap_2d::ObstacleLayer'
    local_obstacle['enabled'] = True
    local_obstacle['footprint_clearing_enabled'] = True
    local_obstacle['observation_sources'] = 'scan'

    local_scan = local_obstacle.setdefault('scan', {})
    local_scan['topic'] = safety_scan_topic
    local_scan['data_type'] = 'LaserScan'
    local_scan['clearing'] = True
    local_scan['marking'] = True
    local_scan['obstacle_min_range'] = 0.05
    local_scan['obstacle_max_range'] = 2.5
    local_scan['raytrace_min_range'] = 0.05
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
    global_scan['topic'] = safety_scan_topic
    global_scan['data_type'] = 'LaserScan'
    global_scan['clearing'] = True
    global_scan['marking'] = True
    global_scan['obstacle_min_range'] = 0.05
    global_scan['obstacle_max_range'] = 2.5
    global_scan['raytrace_min_range'] = 0.05
    global_scan['raytrace_max_range'] = 3.5
    global_scan['max_obstacle_height'] = 2.0
    global_scan['inf_is_valid'] = True
    global_scan['observation_persistence'] = 1.0
    global_scan['expected_update_rate'] = 0.0

    global_inflation = global_cm.setdefault('inflation_layer', {})
    global_inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
    global_inflation['inflation_radius'] = 0.70
    global_inflation['cost_scaling_factor'] = 3.0

    # GO2_V11_6_2_SAFE_RECOVERY
    behavior = params.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
    behavior['cycle_frequency'] = 5.0
    behavior['simulate_ahead_time'] = 1.0
    behavior['max_rotational_vel'] = 0.20
    behavior['min_rotational_vel'] = 0.05
    behavior['rotational_acc_lim'] = 0.35
    behavior['transform_tolerance'] = 0.5
    cm = params.setdefault('collision_monitor', {}).setdefault('ros__parameters', {})
    cm['enabled'] = True
    cm['enable_stamped_cmd_vel'] = False
    cm['base_frame_id'] = 'base_link'
    cm['odom_frame_id'] = 'odom'
    cm['cmd_vel_in_topic'] = 'cmd_vel_nav'
    cm['cmd_vel_out_topic'] = 'cmd_vel_out'
    cm['state_topic'] = 'collision_monitor_state'
    cm['transform_tolerance'] = 0.5
    # Keep the fail-safe watchdog, but use the validated Go2/WebRTC jitter budget.
    # This remains configurable without editing source.
    try:
        collision_source_timeout = float(os.environ.get('GO2_COLLISION_SOURCE_TIMEOUT_SEC', '1.6'))
    except ValueError:
        collision_source_timeout = 1.6
    collision_source_timeout = max(0.5, min(collision_source_timeout, 5.0))
    cm['source_timeout'] = float(os.environ.get('GO2_COLLISION_SOURCE_TIMEOUT_SEC', '1.6'))
    cm['stop_pub_timeout'] = 1.0
    cm['polygons'] = ['StopPolygon', 'SlowdownPolygon']
    collision_source = os.environ.get('GO2_COLLISION_SOURCE', 'pointcloud').strip().lower()
    if collision_source == 'scan':
        cm['observation_sources'] = ['scan']
    else:
        cm['observation_sources'] = ['pointcloud']
    stop = cm.setdefault('StopPolygon', {})
    stop['type'] = 'polygon'
    stop['points'] = '[[0.45, 0.28], [0.45, -0.28], [-0.20, -0.28], [-0.20, 0.28]]'
    stop['action_type'] = 'stop'
    stop['min_points'] = 4
    stop['visualize'] = True
    stop['polygon_pub_topic'] = 'stop_polygon'

    slow = cm.setdefault('SlowdownPolygon', {})
    slow['type'] = 'polygon'
    slow['points'] = '[[1.20, 0.45], [1.20, -0.45], [-0.25, -0.45], [-0.25, 0.45]]'
    slow['action_type'] = 'slowdown'
    slow['slowdown_ratio'] = 0.35
    slow['min_points'] = 4
    slow['visualize'] = True
    slow['polygon_pub_topic'] = 'slowdown_polygon'

    cm_scan = cm.setdefault('scan', {})
    cm_scan['type'] = 'scan'
    cm_scan['topic'] = safety_scan_topic
    cm_scan['enabled'] = collision_source == 'scan'
    cm_scan['source_timeout'] = cm['source_timeout']

    cm_pointcloud = cm.setdefault('pointcloud', {})
    cm_pointcloud['type'] = 'pointcloud'
    cm_pointcloud['topic'] = pointcloud_topic
    cm_pointcloud['transport_type'] = 'raw'
    cm_pointcloud['min_height'] = 0.05
    cm_pointcloud['max_height'] = 0.65
    cm_pointcloud['min_range'] = 0.18
    cm_pointcloud['enabled'] = collision_source != 'scan'
    cm_pointcloud['source_timeout'] = cm['source_timeout']

    behavior = params.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
    behavior['enable_stamped_cmd_vel'] = False
    try:
        params.setdefault('amcl', {}).setdefault('ros__parameters', {})['scan_topic'] = scan_topic
    except Exception:
        pass

    try:
        params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = safety_scan_topic
    except Exception:
        pass
    try:
        params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = safety_scan_topic
    except Exception:
        pass
    try:
        params['collision_monitor']['ros__parameters']['scan']['topic'] = safety_scan_topic
    except Exception:
        pass

    print(f'[semantic_nav_resume] safety_scan_topic={safety_scan_topic} collision_source_timeout={cm["source_timeout"]:.2f}s')
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
    # V11.5: localization must use the measurement-time scan, not the retimestamped safety scan.
    amcl_scan_topic = LaunchConfiguration('amcl_scan_topic').perform(context).strip() or scan_input_topic
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
    print(f'[semantic_nav_resume] AMCL measurement scan={amcl_scan_topic}; retimed safety scan={scan_nav_topic}')
    print(f"[semantic_nav_resume] collision_source={os.environ.get('GO2_COLLISION_SOURCE','pointcloud')} safety_scan={os.environ.get('GO2_SAFETY_SCAN_TOPIC', scan_nav_topic)} timeout={os.environ.get('GO2_COLLISION_SOURCE_TIMEOUT_SEC','2.0')}s")
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
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl_cfg, {'scan_topic': amcl_scan_topic}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='resume_map_lifecycle_manager', output='screen', parameters=[{
            'autostart': True,
            'node_names': ['resume_map_server', 'amcl'],
        }]),
        Node(package='go2_semantic_nav_agent', executable='semantic_nav_node', name='semantic_nav_node', output='screen', respawn=True, respawn_delay=2.0, parameters=[{
            'mode': 'resume',
            'session_root': session_root,
            'session_name': session_name,
            'auto_save_places': False,
            'auto_save_use_vlm': False,
            'restore_spawn_on_start': restore_spawn_on_start,
            'allow_manual_initialpose_override': True,
            'fallback_cmd_topic': '/cmd_vel_omi',
            'fallback_enable': _bool_flag(os.environ.get('GO2_SEMANTIC_FALLBACK_ENABLE', '0'), default=False),
            'initialpose_stamp_backdate_sec': 0.10,
            'scan_topic': scan_nav_topic,
            'tour_auto_advance': _bool_flag(os.environ.get('GO2_TOUR_AUTO_ADVANCE', '1'), default=True),
            'tour_default_pause_sec': float(os.environ.get('GO2_TOUR_PAUSE_SEC', '2.0')),
        }]),
    ]
    print('[semantic_nav_resume] semantic_nav_node immediate+respawn')
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
    ]
    delayed_resume_actions.insert(0, Node(
        package='go2_sysnav_vln',
        executable='registered_cloud_object_projector',
        name='go2_registered_lidar_object_projector',
        output='screen',
        parameters=[{
            'detections_2d_topic': '/object_explorer/sam2_detections',
            'pointcloud_topic': pointcloud_topic,
            'camera_info_topic': '/camera/camera_info',
            'detections_3d_topic': '/go2_vln/target_detections_3d',
            'map_frame': 'map',
            'camera_frame': 'front_camera',
            'max_cloud_age_sec': 0.75,
            'min_points': 5,
            'max_linear_speed_mps': 0.22,
            'max_angular_speed_rps': 0.45,
        }],
    ))
    if rviz or rviz2:
        delayed_resume_actions.append(
            Node(package='rviz2', executable='rviz2', name='semantic_nav_rviz2', output='screen', arguments=['-d', rviz_cfg], additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'})
        )
    nodes.append(TimerAction(period=nav2_start_delay_sec, actions=delayed_resume_actions))
    return nodes


def generate_launch_description():

    motion_arbiter_node = Node(
        package='go2_nav_tools',
        executable='motion_arbiter',
        name='go2_motion_arbiter',
        output='screen',
        parameters=[{
            'nav2_topic': '/cmd_vel_nav2',
            'omi_topic': '/cmd_vel_omi',
            'escape_topic': '/cmd_vel_escape',
            'output_topic': '/cmd_vel_nav',
            'source_timeout_sec': 0.40,
            'nav2_max_x': float(os.environ.get('SPARKY_NAV_STRICT_MAX_X', '0.375')),
            'nav2_max_y': 0.0,
            'nav2_max_theta': float(os.environ.get('SPARKY_NAV_STRICT_MAX_THETA', '0.80')),
            'nav2_zero_subfloor_x': False,
            'omi_max_x': 0.25,
            'omi_max_y': 0.20,
            'omi_max_theta': 0.60,
            'escape_max_x': 0.30,
            'escape_max_y': 0.25,
            'escape_max_theta': 0.70,
        }],
    )

    return LaunchDescription([
        motion_arbiter_node,
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz2', default_value='false'),
        DeclareLaunchArgument('restore_spawn_on_start', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('scan_input_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_nav_topic', default_value='/scan_nav'),
        DeclareLaunchArgument('amcl_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/point_cloud2'),
        DeclareLaunchArgument('stvl_enabled', default_value='auto'),
        DeclareLaunchArgument('scan_frame_id', default_value='base_link'),
        DeclareLaunchArgument('scan_stamp_offset_sec', default_value='0.25'),
        OpaqueFunction(function=launch_setup),
    ])
