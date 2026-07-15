# Codex Task: Stabilize Go2 Nav2 MPPI Navigation

Owner: Dhruv / Go2 ROS 2 project
Workspace: `/home/digital-twin-admin/Dhruv/sparky/ros2_ws`
Target branch: `mppi-nav2`
Primary package: `src/go2_robot_sdk`
Primary config file: `src/go2_robot_sdk/config/nav2_params.yaml`
Primary launch files:
- `src/go2_robot_sdk/launch/robot.launch.py`
- `src/go2_robot_sdk/launch/navigation_no_docking.launch.py`

## Goal

Make the Go2 robot navigate reliably with:

- MPPI as the only local controller.
- A proper global planner for long routes.
- A consistent motion model across AMCL, planner, MPPI, collision monitor, and Go2 driver.
- Short, useful failure detection instead of waiting forever.
- Full logs and evidence so we can tune from measured behavior.

The current symptom is:

- Every Nav2 goal prints `Passing new path to controller` repeatedly.
- The robot turns or rotates sideways.
- The robot continues moving forward or drifts without reaching the goal.
- It takes forever and does not fail quickly.

Important: `Passing new path to controller` is not itself an error. It only means the controller received a newly computed path. The root cause is that the controller stack is not making meaningful progress toward the goal and the progress checker is too permissive.

## Root Cause Summary

The current implementation has an inconsistent navigation contract:

1. MPPI is the local controller, but the global planner is/was Smac Hybrid with Dubin/Reeds-Shepp style heading constraints. That is too car-like for the current forward-only Go2 MPPI baseline.
2. AMCL uses `OmniMotionModel`, but MPPI is configured as diff-drive / no lateral motion (`vy_max: 0.0`, `vy_std: 0.0`).
3. The progress checker allows about 600 seconds, so Nav2 can appear to run forever while the robot is not really reaching the goal.
4. Global costmap uses live `/scan`, which can make the global path unstable. Live scan obstacles should be local-costmap/collision-monitor only for this baseline.
5. Collision monitor slowdown is too aggressive for debugging and can shrink valid MPPI commands into crawl commands.
6. Driver gain/clamping can distort MPPI output; while tuning, driver velocity scaling should be close to identity.
7. MPPI motion model syntax should use the current Nav2 plugin form: `motion_model: "diff_drive"` and `diff_drive.plugin: "mppi::DiffDriveMotionModel"`.
8. The BT XML may only refresh paths every 10 seconds due to `PathExpiringTimer seconds="10"`; change this to 2 seconds for faster path updates.

## Non-Goals / Do Not Do

Do not do any of the following:

- Do not switch the local controller to DWB.
- Do not try to use MPPI as the global planner. Nav2 MPPI is a controller plugin, not a global planner plugin.
- Do not use Smac Hybrid with `DUBIN` or `REEDS_SHEPP` for this baseline.
- Do not increase `iteration_count` above 1.
- Do not use huge `cmd_vel_linear_gain` values to hide poor MPPI output.
- Do not change semantic navigation packages in this task.
- Do not modify files under `install/` directly. Modify `src/`, rebuild, then source `install/setup.bash`.
- Do not add dotted controller params to `param_substitutions` in `navigation_no_docking.launch.py`. Example of what NOT to do:
  - `controller_server.ros__parameters.current_goal_checker`
  - `controller_server.ros__parameters.current_progress_checker`
  This previously caused startup issues. Keep `param_substitutions = {'autostart': autostart}`.

## Required Final Architecture

Use this architecture:

```text
Long route global planner: SmacPlanner2D
Local controller: MPPI diff_drive
AMCL motion model: DifferentialMotionModel
Global costmap: static map + inflation only
Local costmap: live scan obstacle handling
Collision monitor: active but not overly aggressive while debugging
Driver velocity gains: identity-ish while tuning
```

## Step 0: Create a Work Branch and Log Directory

Run this before making changes:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh || true
source install/setup.bash || true

git -C src/go2_robot_sdk status --short || true

git -C src/go2_robot_sdk checkout mppi-nav2
git -C src/go2_robot_sdk checkout -b fix/go2-mppi-stable-baseline

TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/go2_nav2_mppi_fix_logs/$TS"
mkdir -p "$LOG_DIR"

echo "$LOG_DIR" | tee /tmp/go2_nav2_mppi_latest_log_dir.txt

git -C src/go2_robot_sdk branch --show-current | tee "$LOG_DIR/branch.txt"
git -C src/go2_robot_sdk status --short | tee "$LOG_DIR/git_status_before.txt"
cp src/go2_robot_sdk/config/nav2_params.yaml "$LOG_DIR/nav2_params.before.yaml"
cp src/go2_robot_sdk/launch/robot.launch.py "$LOG_DIR/robot.launch.before.py"
cp src/go2_robot_sdk/launch/navigation_no_docking.launch.py "$LOG_DIR/navigation_no_docking.before.py"
```

If the repo root is actually `src/go2_ros` and `go2_robot_sdk` is below it, adjust the git commands accordingly. Do not guess silently. Print `pwd`, `git rev-parse --show-toplevel`, and the file paths you are editing.

## Step 1: Update AMCL Motion Model

File: `src/go2_robot_sdk/config/nav2_params.yaml`

Find:

```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::OmniMotionModel"
```

Change to:

```yaml
amcl:
  ros__parameters:
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

Reason: The current MPPI baseline is diff-drive / no lateral velocity. AMCL should not assume omni motion until we intentionally enable true omni MPPI.

## Step 2: Replace Global Planner With SmacPlanner2D

File: `src/go2_robot_sdk/config/nav2_params.yaml`

Replace the current `planner_server` block with this exact block unless there are unrelated parameters that must be preserved. Keep indentation exactly as shown.

```yaml
planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.5
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 500000
      max_on_approach_iterations: 1000
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smooth_path: true
```

Remove or ignore any old Smac Hybrid-only parameters under `GridBased`, including:

```yaml
motion_model_for_search
angle_quantization_bins
minimum_turning_radius
analytic_expansion_ratio
analytic_expansion_max_length
analytic_expansion_max_cost
reverse_penalty
change_penalty
non_straight_penalty
cost_penalty
retrospective_penalty
lookup_table_size
cache_obstacle_heuristic
downsample_obstacle_heuristic
max_planning_time
```

Reason: We still need a global planner for long routes, but it should be a 2D grid route for this Go2 baseline. MPPI remains the only local controller.

## Step 3: Replace Controller Server / MPPI Block

File: `src/go2_robot_sdk/config/nav2_params.yaml`

Under `controller_server.ros__parameters`, ensure the following complete baseline exists. Replace the existing progress checker, goal checker, controller plugin list, and `FollowPath` MPPI block with this.

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 15.0
    costmap_update_timeout: 0.80

    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    failure_tolerance: 0.3

    progress_checker_plugins: ["progress_checker"]
    current_progress_checker: "progress_checker"

    goal_checker_plugins: ["general_goal_checker"]
    current_goal_checker: "general_goal_checker"

    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::PoseProgressChecker"
      required_movement_radius: 0.05
      required_movement_angle: 0.10
      movement_time_allowance: 15.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.30
      yaw_goal_tolerance: 0.60

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      motion_model: "diff_drive"
      diff_drive:
        plugin: "mppi::DiffDriveMotionModel"

      time_steps: 32
      model_dt: 0.066
      batch_size: 800
      iteration_count: 1

      vx_max: 0.25
      vx_min: 0.0
      vy_max: 0.0
      wz_max: 0.22

      ax_max: 0.20
      ax_min: -0.25
      ay_max: 0.0
      ay_min: 0.0
      az_max: 0.25

      vx_std: 0.12
      vy_std: 0.0
      wz_std: 0.08

      prune_distance: 1.0
      transform_tolerance: 1.0
      temperature: 0.30
      gamma: 0.015

      visualize: false
      regenerate_noises: false
      open_loop: true

      TrajectoryValidator:
        plugin: "mppi::DefaultOptimalTrajectoryValidator"
        collision_lookahead_time: 1.2
        consider_footprint: false

      critics:
        - ConstraintCritic
        - CostCritic
        - GoalCritic
        - GoalAngleCritic
        - PathAlignCritic
        - PathFollowCritic
        - PathAngleCritic
        - PreferForwardCritic
        - VelocityDeadbandCritic

      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0

      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 1.2
        critical_cost: 300.0
        collision_cost: 1000000.0
        consider_footprint: false
        near_goal_distance: 0.7
        trajectory_point_step: 3

      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 8.0
        threshold_to_consider: 1.2

      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 1.0
        threshold_to_consider: 0.35

      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
        threshold_to_consider: 0.5

      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        max_path_occupancy_ratio: 0.30
        trajectory_point_step: 6
        threshold_to_consider: 0.6
        offset_from_furthest: 6
        use_path_orientations: false

      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        offset_from_furthest: 5
        threshold_to_consider: 1.2

      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 0.5
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0

      VelocityDeadbandCritic:
        enabled: true
        cost_power: 1
        cost_weight: 25.0
        deadband_velocities: [0.06, 0.0, 0.04]
```

Important notes:

- Keep `iteration_count: 1`.
- `controller_frequency: 15.0` and `model_dt: 0.066` are paired. Do not set controller frequency to 15 Hz while leaving `model_dt` at 0.1 or 0.125.
- This is intentionally less spin-aggressive than the current branch. The robot was rotating/turning too much and not settling at the goal.
- `yaw_goal_tolerance: 0.60` is intentional. Exact final yaw is less important while stabilizing walking.
- `open_loop: true` is intentional for now because the robot stack previously had odom/TF timing instability. We can revisit after logs show odom is stable.

## Step 4: Make Global Costmap Static-Only

File: `src/go2_robot_sdk/config/nav2_params.yaml`

In `global_costmap.global_costmap.ros__parameters`, remove `obstacle_layer` from the plugins list.

Change this kind of block:

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

to:

```yaml
plugins: ["static_layer", "inflation_layer"]
```

Make sure the global costmap has this structure:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 2.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      resolution: 0.05
      track_unknown_space: true
      footprint: "[ [0.360, 0.200], [0.360, -0.200], [-0.450, -0.200], [-0.450, 0.200] ]"

      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.70

      always_send_full_costmap: true
```

If the old `obstacle_layer:` section remains under `global_costmap`, it is harmless only if it is not listed in `plugins`, but prefer removing or commenting it to avoid confusion.

Reason: The global planner should plan over the static map. Live scan obstacles should affect local avoidance, not constantly rewrite the global route.

## Step 5: Keep Local Costmap Live Scan

File: `src/go2_robot_sdk/config/nav2_params.yaml`

Do not remove the local costmap obstacle layer. It should keep live scan obstacle handling.

Expected local costmap shape:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      plugins: ["obstacle_layer", "inflation_layer"]
```

For scan topic:

- Use `/scan` if that is the only scan topic available.
- Use `/scan_nav` only if a retimestamped scan publisher exists and is launched.
- Do not blindly change to `/scan_nav` unless `ros2 topic list | grep '^/scan_nav$'` confirms it exists.

Add this runtime check to the log file later:

```bash
ros2 topic list | grep -E '^/scan$|^/scan_nav$' | tee "$LOG_DIR/scan_topics.txt"
ros2 topic hz /scan > "$LOG_DIR/scan_hz.txt" 2>&1 & echo $! > "$LOG_DIR/scan_hz.pid"
sleep 8
kill $(cat "$LOG_DIR/scan_hz.pid") || true
```

## Step 6: Make Collision Monitor Less Aggressive While Debugging

File: `src/go2_robot_sdk/config/nav2_params.yaml`

Find collision monitor slowdown polygon:

```yaml
SlowdownPolygon:
  slowdown_ratio: 0.25
```

Change to:

```yaml
SlowdownPolygon:
  slowdown_ratio: 0.70
```

Keep the routing:

```yaml
cmd_vel_in_topic: "cmd_vel_nav"
cmd_vel_out_topic: "cmd_vel_out"
```

Reason: A slowdown ratio of 0.25 can reduce a usable MPPI command into an unwalkable crawl command.

## Step 7: Make Go2 Driver Velocity Scaling Identity-ish

File: `src/go2_robot_sdk/launch/robot.launch.py`

Find these Go2 driver parameters if present:

```python
'cmd_vel_linear_gain': 2.5,
'cmd_vel_angular_gain': 0.8,
```

Change to:

```python
'cmd_vel_linear_gain': 1.0,
'cmd_vel_angular_gain': 1.0,
```

Ensure these are safe:

```python
'cmd_vel_min_linear_x': 0.0,
'cmd_vel_min_angular_z': 0.0,
'cmd_vel_max_linear_x': 0.30,
'cmd_vel_max_angular_z': 0.35,
```

If the file currently has a minimum threshold like `cmd_vel_min_linear_x: 0.10`, do not leave it there for this MPPI baseline. It can hide small-but-valid MPPI output. The `VelocityDeadbandCritic` will do the controller-side deadband work instead.

Reason: Tune MPPI output directly. Do not distort it with hidden driver-side scaling.

## Step 8: Add Fast Replanning BT XML

The current tree likely only refreshes a valid path every 10 seconds due to `PathExpiringTimer seconds="10"`. Create a package-local BT XML and change the timer to 2 seconds.

Commands:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
mkdir -p src/go2_robot_sdk/config/behavior_trees

cp /opt/ros/$ROS_DISTRO/share/nav2_bt_navigator/behavior_trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml \
  src/go2_robot_sdk/config/behavior_trees/go2_fast_replanning.xml

sed -i 's/PathExpiringTimer seconds="10"/PathExpiringTimer seconds="2.0"/g' \
  src/go2_robot_sdk/config/behavior_trees/go2_fast_replanning.xml

grep -n "RateController\|PathExpiringTimer\|ComputePathToPose" \
  src/go2_robot_sdk/config/behavior_trees/go2_fast_replanning.xml
```

Then update `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` in `nav2_params.yaml` to this exact absolute source path:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/home/digital-twin-admin/Dhruv/sparky/ros2_ws/src/go2_robot_sdk/config/behavior_trees/go2_fast_replanning.xml"
```

This path is acceptable for the current development machine. If you want install-space portability, ensure `config/behavior_trees/*.xml` is installed by the package and use the install-space path instead.

Reason: Path refresh every 10 seconds made debugging feel slow. Use 2 seconds. This does not fix poor movement by itself; it only makes route updates more responsive.

## Step 9: Verify navigation_no_docking.launch.py Is Safe

File: `src/go2_robot_sdk/launch/navigation_no_docking.launch.py`

Ensure this remains simple:

```python
param_substitutions = {'autostart': autostart}
```

Do not add dotted controller parameter rewrites here.

Optional safe improvement: ensure the controller node has an explicit name:

```python
Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[configured_params],
    arguments=['--ros-args', '--log-level', log_level],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
),
```

Also apply the explicit name in the composable controller if needed:

```python
ComposableNode(
    package='nav2_controller',
    plugin='nav2_controller::ControllerServer',
    name='controller_server',
    parameters=[configured_params],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
),
```

Do not add extra direct parameter overrides here unless the YAML still fails to provide `current_goal_checker` and `current_progress_checker` after verification.

## Step 10: Validate Syntax Before Building

Run:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

python - <<'PY' | tee "$LOG_DIR/yaml_validation.txt"
from pathlib import Path
import sys
try:
    import yaml
except Exception as e:
    print(f"PyYAML unavailable: {e}")
    sys.exit(2)
path = Path('src/go2_robot_sdk/config/nav2_params.yaml')
with path.open() as f:
    data = yaml.safe_load(f)
print('YAML OK')
print('top-level keys:', sorted(data.keys()))
PY

python -m py_compile src/go2_robot_sdk/launch/robot.launch.py | tee "$LOG_DIR/robot_launch_py_compile.txt"
python -m py_compile src/go2_robot_sdk/launch/navigation_no_docking.launch.py | tee "$LOG_DIR/navigation_no_docking_py_compile.txt"
```

If PyYAML is missing, install only if acceptable in the active venv:

```bash
python -m pip install PyYAML
```

## Step 11: Build

Use Dhruv's standard build command:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

python -m colcon build --symlink-install \
  --packages-select go2_robot_sdk \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev \
  2>&1 | tee "$LOG_DIR/build_go2_robot_sdk.log"

source install/setup.bash
```

If the build fails, stop and record the failure. Do not continue to runtime testing.

## Step 12: Save Diff

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

git -C src/go2_robot_sdk diff -- src/go2_robot_sdk/config/nav2_params.yaml src/go2_robot_sdk/launch/robot.launch.py src/go2_robot_sdk/launch/navigation_no_docking.launch.py \
  > "$LOG_DIR/go2_robot_sdk.diff" || true

git -C src/go2_robot_sdk status --short | tee "$LOG_DIR/git_status_after.txt"
```

If the git root is not `src/go2_robot_sdk`, use the correct root. The important thing is to save a patch/diff file.

## Step 13: Clean Runtime Start

Before launching:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

source ./go2_env.sh
source install/setup.bash

pkill -f "controller_server|planner_server|bt_navigator|collision_monitor|lifecycle_manager|rviz2|nav2|amcl|map_server|slam_toolbox" || true
ros2 daemon stop || true
ros2 daemon start
```

Launch without joystick/teleop noise:

```bash
ros2 launch go2_robot_sdk robot.launch.py \
  nav2:=true \
  slam:=true \
  joystick:=false \
  teleop:=false \
  foxglove:=false \
  log_level:=info \
  2>&1 | tee "$LOG_DIR/robot_launch_runtime.log"
```

If using saved-map navigation, do not use `slam:=true`; use the resume/localization launch path instead. For this task, the above command is acceptable for live SLAM + Nav2 testing.

## Step 14: Runtime Verification Commands

Open a second terminal and run:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

ros2 node list | sort | tee "$LOG_DIR/ros2_node_list.txt"

ros2 lifecycle get /controller_server 2>&1 | tee "$LOG_DIR/lifecycle_controller_server.txt"
ros2 lifecycle get /planner_server 2>&1 | tee "$LOG_DIR/lifecycle_planner_server.txt"
ros2 lifecycle get /bt_navigator 2>&1 | tee "$LOG_DIR/lifecycle_bt_navigator.txt"
ros2 lifecycle get /collision_monitor 2>&1 | tee "$LOG_DIR/lifecycle_collision_monitor.txt"

ros2 param get /controller_server FollowPath.plugin 2>&1 | tee "$LOG_DIR/param_followpath_plugin.txt"
ros2 param get /controller_server FollowPath.motion_model 2>&1 | tee "$LOG_DIR/param_followpath_motion_model.txt"
ros2 param get /planner_server GridBased.plugin 2>&1 | tee "$LOG_DIR/param_planner_plugin.txt"
ros2 param get /amcl robot_model_type 2>&1 | tee "$LOG_DIR/param_amcl_motion_model.txt"
ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>&1 | tee "$LOG_DIR/param_bt_xml.txt"
ros2 param get /controller_server current_goal_checker 2>&1 | tee "$LOG_DIR/param_current_goal_checker.txt"
ros2 param get /controller_server current_progress_checker 2>&1 | tee "$LOG_DIR/param_current_progress_checker.txt"
```

Expected:

```text
/controller_server: active
/planner_server: active
/bt_navigator: active
/collision_monitor: active
FollowPath.plugin: nav2_mppi_controller::MPPIController
FollowPath.motion_model: diff_drive
GridBased.plugin: nav2_smac_planner::SmacPlanner2D
robot_model_type: nav2_amcl::DifferentialMotionModel
current_goal_checker: general_goal_checker
current_progress_checker: progress_checker
```

## Step 15: Command Chain Verification During a Goal

After sending a small Nav2 goal in RViz, run these in another terminal.

Record topic frequencies:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

(timeout 12 ros2 topic hz /cmd_vel_nav) 2>&1 | tee "$LOG_DIR/cmd_vel_nav_hz.txt"
(timeout 12 ros2 topic hz /cmd_vel_out) 2>&1 | tee "$LOG_DIR/cmd_vel_out_hz.txt"
```

Record command samples:

```bash
ros2 topic echo --once /cmd_vel_nav 2>&1 | tee "$LOG_DIR/cmd_vel_nav_once_1.txt"
ros2 topic echo --once /cmd_vel_out 2>&1 | tee "$LOG_DIR/cmd_vel_out_once_1.txt"
ros2 topic echo --once /collision_monitor_state 2>&1 | tee "$LOG_DIR/collision_monitor_state_once_1.txt"
```

Expected during a normal forward-ish goal:

```text
/cmd_vel_nav frequency: about 10-15 Hz
/cmd_vel_out frequency: about 10-15 Hz
linear.x: usually about 0.06 to 0.18
angular.z: usually below about +/-0.20 except during turns
/cmd_vel_out should be similar to /cmd_vel_nav, not zero or drastically smaller
```

If `/cmd_vel_nav` is reasonable but `/cmd_vel_out` is much smaller or zero, collision monitor or stale scan is the next root cause.

If `/cmd_vel_nav` has near-zero linear velocity and high angular velocity, MPPI tuning or path/controller mismatch is still wrong.

If both `/cmd_vel_nav` and `/cmd_vel_out` look good but the robot does not move correctly, the Go2 driver or robot walking mode is the next root cause.

## Step 16: Sensor and TF Health Logs

Run:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

ros2 topic list | grep -E '^/scan$|^/scan_nav$|^/odom$|^/tf$|^/tf_static$|^/map$' | tee "$LOG_DIR/core_topics.txt"

(timeout 8 ros2 topic hz /scan) 2>&1 | tee "$LOG_DIR/scan_hz.txt"
(timeout 8 ros2 topic hz /odom) 2>&1 | tee "$LOG_DIR/odom_hz.txt"
ros2 topic echo --once /scan/header 2>&1 | tee "$LOG_DIR/scan_header_once.txt"
ros2 topic echo --once /odom/header 2>&1 | tee "$LOG_DIR/odom_header_once.txt"

ros2 run tf2_ros tf2_echo map base_link --timeout 2 2>&1 | tee "$LOG_DIR/tf_map_base_link.txt" || true
ros2 run tf2_ros tf2_echo odom base_link --timeout 2 2>&1 | tee "$LOG_DIR/tf_odom_base_link.txt" || true
```

If logs show stale scan errors, timestamp mismatch, or transform extrapolation, record them and do not tune MPPI blindly.

## Step 17: Optional Short Rosbag for Evidence

Record a 30 second bag while sending a small goal:

```bash
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
source ./go2_env.sh
source install/setup.bash
LOG_DIR=$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)

mkdir -p "$LOG_DIR/bags"
timeout 30 ros2 bag record \
  -o "$LOG_DIR/bags/nav2_mppi_goal_test" \
  /tf /tf_static /odom /scan /map /cmd_vel_nav /cmd_vel_out /collision_monitor_state /plan /local_plan \
  2>&1 | tee "$LOG_DIR/rosbag_record.log"
```

If `/plan` or `/local_plan` topics do not exist, record the bag anyway with the topics that exist.

## Step 18: Success Criteria

The change is successful only if these are true:

1. Nav2 lifecycle nodes are active.
2. `FollowPath.plugin` is `nav2_mppi_controller::MPPIController`.
3. `FollowPath.motion_model` is `diff_drive`.
4. Planner plugin is `nav2_smac_planner::SmacPlanner2D`.
5. AMCL model is `nav2_amcl::DifferentialMotionModel`.
6. `/cmd_vel_nav` publishes at about 10-15 Hz during a goal.
7. `/cmd_vel_out` publishes at about the same rate and is not zeroed by collision monitor.
8. The robot makes visible progress within 15 seconds.
9. If it does not make progress, Nav2 fails or triggers recovery within about 15 seconds, not 600 seconds.
10. `Passing new path to controller` may still appear, but it should not be the only observable behavior. The robot must make measurable pose progress.

## If Robot Still Goes Crazy After This

Use the logs to classify the failure:

### Case A: `/cmd_vel_nav` is small or angular-heavy

Example:

```text
linear.x < 0.03 most of the time
angular.z constantly near max
```

Next tuning:

- Lower `wz_max` to `0.18`.
- Lower `wz_std` to `0.06`.
- Increase `PathFollowCritic.cost_weight` from `14.0` to `18.0`.
- Increase `GoalCritic.cost_weight` from `8.0` to `10.0`.
- Keep `yaw_goal_tolerance` at `0.60`.

### Case B: `/cmd_vel_nav` is good, `/cmd_vel_out` is tiny or zero

Next tuning:

- Collision monitor is slowing/stopping.
- Inspect `collision_monitor_state`.
- Temporarily deactivate collision monitor for one open-area test:

```bash
ros2 lifecycle deactivate /collision_monitor
```

If the robot improves, tune collision polygons or scan timing.

### Case C: `/cmd_vel_nav` and `/cmd_vel_out` are good, robot does not move correctly

Next checks:

- Direct driver test:

```bash
timeout 2 ros2 topic pub --rate 10 /cmd_vel_out geometry_msgs/msg/Twist \
"{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /cmd_vel_out geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

If this does not move the robot forward, the issue is not Nav2/MPPI. It is driver mode, WebRTC command path, or Go2 gait/walking state.

### Case D: TF/scan stale warnings appear

Next checks:

- Use a retimestamped `/scan_nav` only if the retimestamp node exists.
- Make AMCL, local costmap, and collision monitor all use the same fresh scan topic.
- Do not tune MPPI while TF/scan timestamps are broken.

## Final Deliverables From Codex

Codex must produce:

1. A concise summary of files changed.
2. A saved diff/patch path.
3. Build log path.
4. Runtime launch log path.
5. Verification command outputs path.
6. A short diagnosis based on `/cmd_vel_nav`, `/cmd_vel_out`, and `collision_monitor_state`.
7. Any deviations from this plan must be explicitly explained.

Do not claim success unless the logs prove it.
