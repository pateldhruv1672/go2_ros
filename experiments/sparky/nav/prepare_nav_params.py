#!/usr/bin/env python3
"""Exported NavFn+DWB candidate config -> launchable params, with planner/controller variants. Records every change.
usage: prepare_nav_params.py src.yaml map.yaml scenario.json out.yaml [planner] [controller]"""
import sys, json, re, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
src, map_yaml, scn_path, out = sys.argv[1:5]
planner = sys.argv[5] if len(sys.argv) > 5 else 'navfn'
controller = sys.argv[6] if len(sys.argv) > 6 else 'dwb'
scn = json.loads(Path(scn_path).read_text())
p = yaml.safe_load(Path(src).read_text())
changes = []
bt = p['bt_navigator']['ros__parameters']
base = share('nav2_bt_navigator')
for k, v in list(bt.items()):
    if isinstance(v, str) and '$(find-pkg-share nav2_bt_navigator)' in v:
        bt[k] = v.replace('$(find-pkg-share nav2_bt_navigator)', base); changes.append(f'bt_navigator.{k}: resolved find-pkg-share')
a = p['amcl']['ros__parameters']
a['set_initial_pose'] = True
a['initial_pose'] = {'x': scn['start']['x'], 'y': scn['start']['y'], 'z': 0.0, 'yaw': scn['start']['yaw']}
changes.append('amcl.set_initial_pose=true at scenario start')
p['map_server']['ros__parameters']['yaml_filename'] = map_yaml
changes.append('map_server.yaml_filename=' + map_yaml)

PLANNERS = {
  'navfn': {'plugin': 'nav2_navfn_planner::NavfnPlanner', 'tolerance': 0.5, 'use_astar': False, 'allow_unknown': True, 'use_final_approach_orientation': False},
  'smac2d': {'plugin': 'nav2_smac_planner::SmacPlanner2D', 'tolerance': 0.25, 'downsample_costmap': False, 'downsampling_factor': 1,
             'allow_unknown': True, 'max_iterations': 1000000, 'max_on_approach_iterations': 1000, 'max_planning_time': 2.0,
             'cost_travel_multiplier': 2.0, 'use_final_approach_orientation': False,
             'smoother': {'max_iterations': 1000, 'w_smooth': 0.3, 'w_data': 0.2, 'tolerance': 1.0e-10}},
}
PLANNERS['smac2d_smoothed'] = dict(PLANNERS['smac2d'])  # same planner; SimpleSmoother is added through the behavior tree below
CONTROLLERS = {
  'dwb': None,  # keep the exported repository DWB block untouched
  'rpp': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
          'desired_linear_vel': 0.3, 'lookahead_dist': 0.6, 'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.9, 'lookahead_time': 1.5,
          'rotate_to_heading_angular_vel': 0.6, 'transform_tolerance': 0.1, 'use_velocity_scaled_lookahead_dist': False,
          'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 0.6, 'use_collision_detection': True,
          'max_allowed_time_to_collision_up_to_carrot': 1.0, 'use_regulated_linear_velocity_scaling': True,
          'use_cost_regulated_linear_velocity_scaling': False, 'regulated_linear_scaling_min_radius': 0.9,
          'regulated_linear_scaling_min_speed': 0.1, 'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.785,
          'max_angular_accel': 1.0, 'allow_reversing': False},
  # Hand-chosen second parameter set (shorter lookahead, tighter curvature slowdown, cost regulation). NOT optimized on data.
  'rpp_retuned': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
          'desired_linear_vel': 0.3, 'lookahead_dist': 0.45, 'min_lookahead_dist': 0.25, 'max_lookahead_dist': 0.75, 'lookahead_time': 1.2,
          'rotate_to_heading_angular_vel': 0.6, 'transform_tolerance': 0.1, 'use_velocity_scaled_lookahead_dist': False,
          'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 0.5, 'use_collision_detection': True,
          'max_allowed_time_to_collision_up_to_carrot': 1.0, 'use_regulated_linear_velocity_scaling': True,
          'use_cost_regulated_linear_velocity_scaling': True, 'cost_scaling_dist': 0.6, 'cost_scaling_gain': 1.0,
          'regulated_linear_scaling_min_radius': 0.6, 'regulated_linear_scaling_min_speed': 0.1,
          'use_fixed_curvature_lookahead': True, 'curvature_lookahead_dist': 0.6,
          'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.6, 'max_angular_accel': 1.0, 'allow_reversing': False},
  'mppi': {'plugin': 'nav2_mppi_controller::MPPIController', 'time_steps': 56, 'model_dt': 0.05, 'batch_size': 2000,
           'ax_max': 0.45, 'ax_min': -0.6, 'ay_max': 0.0, 'az_max': 1.0, 'vx_std': 0.2, 'vy_std': 0.0, 'wz_std': 0.4,
           'vx_max': 0.3, 'vx_min': 0.0, 'vy_max': 0.0, 'wz_max': 0.6, 'iteration_count': 1, 'prune_distance': 1.7,
           'transform_tolerance': 0.1, 'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'DiffDrive', 'visualize': False,
           'regenerate_noises': True,
           'TrajectoryValidator': {'plugin': 'mppi::DefaultOptimalTrajectoryValidator'} if False else {},
           'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
           'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
           'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4},
           'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0, 'threshold_to_consider': 0.5},
           'PreferForwardCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 0.5},
           'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.81, 'critical_cost': 300.0, 'consider_footprint': True,
                          'collision_cost': 1000000.0, 'near_goal_distance': 0.5, 'trajectory_point_step': 2},
           'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0, 'max_path_occupancy_ratio': 0.05,
                               'trajectory_point_step': 4, 'threshold_to_consider': 0.5, 'offset_from_furthest': 20, 'use_path_orientations': False},
           'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'offset_from_furthest': 5, 'threshold_to_consider': 1.4},
           'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0, 'offset_from_furthest': 4, 'threshold_to_consider': 0.5,
                               'max_angle_to_furthest': 1.0, 'mode': 0}},
}
CONTROLLERS['mppi'].pop('TrajectoryValidator')
pl = p['planner_server']['ros__parameters']
pl['GridBased'] = PLANNERS[planner]; changes.append('planner_server.GridBased := ' + planner)
cs = p['controller_server']['ros__parameters']
if CONTROLLERS[controller] is not None:
    cs['FollowPath'] = CONTROLLERS[controller]; changes.append('controller_server.FollowPath := ' + controller)
else:
    changes.append('controller_server.FollowPath kept as exported repository DWB')
# smoother server is always launched; only the *_smoothed planner arm inserts SmoothPath into the BT
p['smoother_server'] = {'ros__parameters': {'use_sim_time': True, 'smoother_plugins': ['simple_smoother'],
    'simple_smoother': {'plugin': 'nav2_smoother::SimpleSmoother', 'tolerance': 1.0e-10, 'max_its': 1000, 'do_refinement': True}}}
bt_key = 'default_nav_to_pose_bt_xml'
xml = Path(bt[bt_key]).read_text()
if planner == 'smac2d_smoothed':
    new, n = re.subn(r'<ComputePathToPose[^>]*/>', lambda m: '<Sequence>' + m.group(0) + '<SmoothPath unsmoothed_path="{path}" smoothed_path="{path}" smoother_id="simple_smoother" error_code_id="{smoother_error_code}"/></Sequence>', xml)
    if n < 1: raise SystemExit('BT patch pattern not found')
    bt_path = str(Path(out).with_name('bt_smoothed.xml')); Path(bt_path).write_text(new); bt[bt_key] = bt_path
    changes.append(f'BT: wrapped {n} ComputePathToPose with SmoothPath(simple_smoother) -> {bt_path}')
Path(out).write_text(yaml.safe_dump(p))
Path(out + '.changes.json').write_text(json.dumps({'planner': planner, 'controller': controller, 'changes': changes}, indent=1))
print(changes)
