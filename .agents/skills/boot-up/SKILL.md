# SKILL.md — Booting the Go2 ROS 2 Jazzy Stack Correctly

This guide boots the Unitree Go2 ROS 2 stack from a clean terminal, with Nav2, SLAM Toolbox, RViz, Foxglove, collision monitor, and the Go2 WebRTC driver.

The expected workspace root is:

```bash
~/Dhruv/sparky/ros2_ws
```

The main launch command is:

```bash
ros2 launch go2_robot_sdk robot.launch.py
```

---

## 1. Known-good environment

Use ROS 2 Jazzy with the workspace virtual environment.

Recommended runtime environment:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
```

Use the same `ROS_DOMAIN_ID` in every terminal. If one terminal uses `0` and another uses `7`, nodes will not see each other.

Avoid mixing Conda Python with ROS Python. If needed, deactivate Conda first:

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
```

Then source ROS and the workspace venv:

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
```

---

## 2. Clean stale ROS processes before boot

Before launching, kill old ROS/Nav2/RViz processes and restart the ROS daemon:

```bash
pkill -9 -f "rviz2|go2_rviz2|robot.launch.py|go2_driver_node|bt_navigator|planner_server|controller_server|behavior_server|lifecycle_manager|nav2|collision_monitor|docking_server|slam_toolbox|foxglove_bridge|pointcloud|lidar" || true

ros2 daemon stop || true
ros2 daemon start
```

---

## 3. Build the workspace

For normal builds:

```bash
cd ~/Dhruv/sparky/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate

python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev

source install/setup.bash
```

For rebuilding only `go2_robot_sdk`:

```bash
python -m colcon build --symlink-install --packages-select go2_robot_sdk \
  --cmake-args -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python -Wno-dev

source install/setup.bash
```

---

## 4. Required local Nav2 fixes

This stack targets ROS 2 Jazzy. The following fixes are required.

### 4.1 Use Jazzy plugin names

Nav2 plugin strings must use `::`, not old `/` syntax.

Examples:

```yaml
plugin: "nav2_smac_planner::SmacPlannerHybrid"
plugin: "nav2_behaviors::Spin"
plugin: "nav2_behaviors::BackUp"
plugin: "nav2_behaviors::DriveOnHeading"
plugin: "nav2_behaviors::Wait"
plugin: "nav2_behaviors::AssistedTeleop"
plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
```

Check for old plugin names:

```bash
grep -RIn 'plugin: "nav2_.*/' src/go2_robot_sdk/config src/go2_robot_sdk/launch || true
```

Expected: no output.

### 4.2 Remove `plugin_lib_names` from `bt_navigator`

In Jazzy, BT plugins are loaded by default. A copied old list can cause:

```text
ID [ComputePathToPose] already registered
```

Check:

```bash
grep -n "plugin_lib_names\|ComputePathToPose\|compute_path" \
  src/go2_robot_sdk/config/nav2_params.yaml || true
```

Expected: no `plugin_lib_names` block.

### 4.3 Keep collision monitor, but configure it

Do not remove `collision_monitor`. It is the safety layer that can stop or slow the robot if `/scan` detects obstacles.

A minimal valid block should exist in `nav2_params.yaml`:

```yaml
collision_monitor:
  ros__parameters:
    use_sim_time: false

    base_frame_id: "base_link"
    odom_frame_id: "odom"
    transform_tolerance: 2.0
    source_timeout: 3.0
    stop_pub_timeout: 2.0

    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel_out"
    state_topic: "collision_monitor_state"

    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      enabled: true

    polygons: ["StopPolygon", "SlowdownPolygon"]

    StopPolygon:
      type: "polygon"
      points: "[[0.45, 0.35], [0.45, -0.35], [-0.30, -0.35], [-0.30, 0.35]]"
      action_type: "stop"
      min_points: 3
      visualize: true
      polygon_pub_topic: "stop_polygon"

    SlowdownPolygon:
      type: "polygon"
      points: "[[0.85, 0.50], [0.85, -0.50], [-0.45, -0.50], [-0.45, 0.50]]"
      action_type: "slowdown"
      slowdown_ratio: 0.4
      min_points: 3
      visualize: true
      polygon_pub_topic: "slowdown_polygon"
```

### 4.4 Do not lifecycle-manage docking unless dock plugins are configured

If docking is lifecycle-managed without dock plugin config, Nav2 aborts with:

```text
Charging dock plugins not given!
Failed to bring up all requested nodes. Aborting bringup.
```

Use a copied Nav2 launch file that excludes `docking_server` from the lifecycle manager list.

The custom launch file should be named:

```text
src/go2_robot_sdk/launch/navigation_no_docking.launch.py
```

It must match this package install pattern:

```python
glob(os.path.join('launch', '*launch.[pxy][yma]*'))
```

So avoid names like:

```text
navigation_launch_no_docking.py
```

That name will not be installed.

`robot.launch.py` should include:

```python
os.path.join(
    get_package_share_directory('go2_robot_sdk'),
    'launch',
    'navigation_no_docking.launch.py'
)
```

Verify it installs correctly:

```bash
ls -l install/go2_robot_sdk/share/go2_robot_sdk/launch/navigation_no_docking.launch.py
```

---

## 5. Boot command

Use this full boot sequence:

```bash
cd ~/Dhruv/sparky/ros2_ws

conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

source /opt/ros/jazzy/setup.bash
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
pkill -9 -f "rviz2|go2_rviz2|robot.launch.py|go2_driver_node|bt_navigator|planner_server|controller_server|behavior_server|lifecycle_manager|nav2|collision_monitor|docking_server|slam_toolbox|foxglove_bridge|pointcloud|lidar" || true

ros2 daemon stop || true
ros2 daemon start

ros2 launch go2_robot_sdk robot.launch.py 2>&1 | tee /tmp/go2_nav2.log
```

---

## 6. Validate Nav2 is healthy

Open a second terminal with the same environment:

```bash
cd ~/Dhruv/sparky/ros2_ws

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
```

Check lifecycle states:

```bash
for n in \
  /controller_server \
  /smoother_server \
  /planner_server \
  /route_server \
  /behavior_server \
  /velocity_smoother \
  /collision_monitor \
  /bt_navigator \
  /waypoint_follower
do
  echo "---- $n ----"
  ros2 lifecycle get $n || true
done
```

Expected:

```text
active [3]
```

Check the NavigateToPose action server:

```bash
ros2 action info /navigate_to_pose
```

Expected:

```text
Action servers: 1
    /bt_navigator
```

Check lifecycle manager node list:

```bash
ros2 param get /lifecycle_manager_navigation node_names
```

Expected:

```text
collision_monitor present
docking_server absent
```

---

## 7. Validate map, scan, odom, and TF

Check required topics:

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /map --once
```

If `/map` does not echo normally, try transient local QoS:

```bash
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Check SLAM:

```bash
ros2 lifecycle get /slam_toolbox
ros2 node list | grep slam
```

Expected:

```text
active [3]
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map odom
```

If navigation aborts with:

```text
Transform data too old when converting from odom to map
```

increase Nav2 `transform_tolerance` values in `nav2_params.yaml` to `2.0`, and use:

```yaml
source_timeout: 3.0
```

for collision monitor.

---

## 8. RViz navigation flow

1. Wait until `/map` appears in RViz.
2. Set RViz fixed frame to:

```text
map
```

3. Use **2D Pose Estimate** to set the robot pose.
4. Use **Nav2 Goal** or **2D Goal Pose**.
5. Watch logs for:

```text
Begin navigating from current location
Received a goal, begin computing control effort
Passing new path to controller
```

That means Nav2 is actually planning and controlling.

---

## 9. Common errors and fixes

### Error: `navigate_to_pose action server is not available`

Check:

```bash
ros2 action info /navigate_to_pose
ros2 lifecycle get /bt_navigator
```

If there are zero action servers, BT Navigator is not active or crashed.

### Error: `Action server is inactive. Rejecting the goal`

Nav2 configured but did not activate.

Check lifecycle:

```bash
ros2 lifecycle get /bt_navigator
```

Activate with lifecycle manager:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"
```

### Error: `Charging dock plugins not given`

`docking_server` is still being lifecycle-managed. Remove `docking_server` from the Nav2 lifecycle list or use `navigation_no_docking.launch.py`.

### Error: `ID [ComputePathToPose] already registered`

Remove `plugin_lib_names` from `bt_navigator` in `nav2_params.yaml`.

### Error: `parameter 'observation_sources' is not initialized`

`collision_monitor` is missing its config. Add the `collision_monitor` block with:

```yaml
observation_sources: ["scan"]
```

### Error: `Transform data too old when converting from odom to map`

Increase transform tolerances:

```yaml
transform_tolerance: 2.0
```

Also check:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /tf
ros2 topic hz /odom
ros2 topic hz /scan
```

### Warning: `/scan` incompatible QoS

For RViz LaserScan display, set:

```text
Reliability Policy: Best Effort
```

For Nav2 costmaps, make sure `/scan` is being received by costmap and collision monitor.

### Warning: `inflation radius is smaller than circumscribed radius`

Set local and global inflation layer radius to at least:

```yaml
inflation_radius: 0.75
```

This improves Smac Hybrid planning behavior around obstacles.

### H264 warnings from `go2_driver_node`

These logs are usually camera-stream noise and not a Nav2 blocker:

```text
H264Decoder() failed to decode
non-existing PPS 0 referenced
```

Ignore unless camera streaming is the target being debugged.

---

## 10. Golden-state checklist

Before navigating, this should pass:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /collision_monitor
ros2 action info /navigate_to_pose
ros2 topic echo /map --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

Expected:

```text
/bt_navigator active [3]
/planner_server active [3]
/controller_server active [3]
/collision_monitor active [3]
Action servers: 1
/map publishes
/scan publishes
/odom publishes
```

Only then send a goal from RViz.
