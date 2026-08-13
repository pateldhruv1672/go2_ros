#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
[ -f "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/src/.venv/bin/activate" ] && source "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/src/.venv/bin/activate"
[ -f "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/install/setup.bash" ] && source "${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}/install/setup.bash"
set -u

echo '=== controller effective params ==='
for p in \
  current_goal_checker current_progress_checker controller_frequency \
  FollowPath.max_vel_x FollowPath.max_vel_theta FollowPath.acc_lim_x FollowPath.acc_lim_theta \
  FollowPath.decel_lim_x FollowPath.decel_lim_theta FollowPath.sim_time \
  FollowPath.xy_goal_tolerance FollowPath.yaw_goal_tolerance \
  general_goal_checker.xy_goal_tolerance general_goal_checker.yaw_goal_tolerance \
  progress_checker.required_movement_radius progress_checker.required_movement_angle progress_checker.movement_time_allowance
do
  printf '%-48s ' "$p"; ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo; echo '=== scan + localization ==='
printf 'AMCL scan_topic: '; ros2 param get /amcl scan_topic 2>/dev/null || true
printf 'scan node: '; ros2 node list 2>/dev/null | grep -E '^/scan_retimestamp_node$' || echo MISSING

echo; echo '=== command ownership ==='
for t in /cmd_vel_nav2 /cmd_vel_escape /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t"; ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
done

echo; echo '=== Tour/agent state ==='
ros2 topic echo /semantic_nav/status --once 2>/dev/null || true
ros2 topic echo /go2_agent/status --once 2>/dev/null || true
