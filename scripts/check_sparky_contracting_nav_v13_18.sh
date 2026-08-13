#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== LIFECYCLE ==="
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done

echo
echo "=== REFERENCE PATH / BT ==="
ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>/dev/null || true

echo
echo "=== CONTINUOUS LOCAL FRAME ==="
printf '%-38s ' 'local_costmap.global_frame'
ros2 param get /local_costmap/local_costmap global_frame 2>/dev/null || true
printf '%-38s ' 'controller_server.odom_topic'
ros2 param get /controller_server odom_topic 2>/dev/null || true
printf '%-38s ' 'behavior_server.local_frame'
ros2 param get /behavior_server local_frame 2>/dev/null || true
printf '%-38s ' 'behavior_server.global_frame'
ros2 param get /behavior_server global_frame 2>/dev/null || true

echo
echo "=== DIRECT DWB ==="
for p in \
  FollowPath.plugin \
  FollowPath.min_speed_xy \
  FollowPath.min_speed_theta \
  FollowPath.max_vel_x \
  FollowPath.max_vel_theta \
  FollowPath.acc_lim_x \
  FollowPath.acc_lim_theta \
  FollowPath.sim_time \
  FollowPath.discretize_by_time \
  FollowPath.time_granularity \
  FollowPath.limit_vel_cmd_in_traj \
  FollowPath.prune_plan \
  FollowPath.prune_distance \
  FollowPath.forward_prune_distance \
  FollowPath.PathDist.scale \
  FollowPath.PathAlign.scale \
  FollowPath.GoalDist.scale \
  FollowPath.GoalAlign.scale
 do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== COMMAND OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "-- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

echo
echo "=== ODOM RATE ==="
timeout 4 ros2 topic hz /odom 2>/dev/null || true

echo
echo "EXPECTED CORE:"
echo "  FollowPath.plugin = dwb_core::DWBLocalPlanner"
echo "  local_costmap.global_frame = odom"
echo "  min_speed_xy = 0.0, min_speed_theta = 0.0"
echo "  limit_vel_cmd_in_traj = True"
echo "  max_vel_x = 0.375"
echo "  PathDist 32 > PathAlign 18 > GoalDist 12"
echo "  BT = go2_error_contracting_nav.xml"
