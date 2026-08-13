#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== ACTUATOR-MATCHED DWB ==="
for p in \
 controller_frequency \
 FollowPath.trajectory_generator_name \
 FollowPath.min_vel_x \
 FollowPath.min_speed_xy \
 FollowPath.max_vel_x \
 FollowPath.max_speed_xy \
 FollowPath.max_vel_theta \
 FollowPath.acc_lim_x \
 FollowPath.acc_lim_theta \
 FollowPath.decel_lim_x \
 FollowPath.decel_lim_theta \
 FollowPath.vx_samples \
 FollowPath.vtheta_samples \
 FollowPath.sim_time \
 general_goal_checker.xy_goal_tolerance \
 general_goal_checker.yaw_goal_tolerance
 do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== ACTUATOR FLOOR ==="
for p in nav2_max_x nav2_max_theta nav2_actuator_floor_x nav2_tiny_x_deadband nav2_rotate_curvature_threshold; do
  printf '%-38s ' "$p"
  ros2 param get /go2_motion_arbiter "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
done

echo
echo "Expected: max_x=0.45, max_theta=0.85, floor_x=0.35, tiny_x=0.03."
echo "Run scripts/trace_sparky_nav_cmds_v13_10.py while setting a goal."
