#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== NAV2 LIFECYCLE ==="
ok=1
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  state="$(timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || true)"
  echo "${state:-NOT_AVAILABLE}"
  [[ "$state" == "active [3]" ]] || ok=0
done

echo
echo "=== JAZZY ROTATION SHIM + DWB ==="
for p in \
  FollowPath.plugin \
  FollowPath.primary_controller \
  FollowPath.angular_dist_threshold \
  FollowPath.angular_disengage_threshold \
  FollowPath.rotate_to_heading_angular_vel \
  FollowPath.max_angular_accel \
  FollowPath.rotate_to_goal_heading \
  FollowPath.max_vel_x \
  FollowPath.max_vel_theta \
  FollowPath.min_speed_xy \
  FollowPath.sim_time \
  FollowPath.PathAlign.scale \
  FollowPath.PathDist.scale \
  FollowPath.GoalAlign.scale \
  FollowPath.GoalDist.scale
do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== WRONG NAMESPACE MUST BE ABSENT ==="
for p in \
  FollowPath.primary_controller.max_vel_x \
  FollowPath.primary_controller.PathAlign.scale
do
  printf '%-52s ' "$p"
  out="$(ros2 param get /controller_server "$p" 2>/dev/null || true)"
  if [[ "$out" == *"Parameter not set"* || -z "$out" ]]; then
    echo "ABSENT (correct for Jazzy)"
  else
    echo "$out"
  fi
done

echo
echo "=== ACTION SERVER ==="
ros2 action info /navigate_to_pose 2>/dev/null || true

echo
echo "=== RVIZ ==="
if ros2 node list 2>/dev/null | grep -qx /semantic_nav_rviz2; then
  echo "/semantic_nav_rviz2 PRESENT"
else
  echo "/semantic_nav_rviz2 NOT PRESENT"
fi

echo
if [[ "$ok" -eq 1 ]]; then
  echo "RESULT=NAV2_JAZZY_ACTIVE"
else
  echo "RESULT=NAV2_NOT_ACTIVE"
fi
