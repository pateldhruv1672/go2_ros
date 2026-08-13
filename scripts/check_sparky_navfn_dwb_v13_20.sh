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
for n in controller_server planner_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done

echo
echo "=== PLANNER ==="
for p in planner_plugins GridBased.plugin GridBased.tolerance GridBased.use_astar; do
  printf '%-42s ' "$p"
  ros2 param get /planner_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== CONTROLLER ==="
for p in \
  controller_frequency \
  FollowPath.plugin \
  FollowPath.max_vel_x \
  FollowPath.max_vel_theta \
  FollowPath.min_speed_xy \
  FollowPath.min_speed_theta \
  FollowPath.acc_lim_x \
  FollowPath.acc_lim_theta \
  FollowPath.sim_time \
  FollowPath.PathAlign.scale \
  FollowPath.PathDist.scale \
  FollowPath.GoalDist.scale
do
  printf '%-42s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== NO RPP / ROTATION SHIM ==="
all_params="$(ros2 param dump /controller_server 2>/dev/null || true)"
if grep -Eq 'RegulatedPurePursuit|RotationShim' <<<"$all_params"; then
  echo "FAIL: experimental controller still visible live"
else
  echo "PASS: no RPP / RotationShim in controller_server"
fi

echo
echo "=== NAVIGATE ACTION ==="
ros2 action info /navigate_to_pose 2>/dev/null || true

echo
echo "Expected:"
echo "  GridBased.plugin = nav2_navfn_planner::NavfnPlanner"
echo "  FollowPath.plugin = dwb_core::DWBLocalPlanner"
echo "  max_vel_x = 0.30"
echo "  max_vel_theta = 0.60"
echo "  sim_time = 1.40"
