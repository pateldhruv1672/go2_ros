#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== RPP PACKAGE ==="
ros2 pkg prefix nav2_regulated_pure_pursuit_controller 2>/dev/null || echo 'MISSING nav2_regulated_pure_pursuit_controller'

echo
echo "=== CONTROLLER RUNTIME ==="
for p in \
 controller_frequency \
 speed_limit_topic \
 FollowPath.plugin \
 FollowPath.desired_linear_vel \
 FollowPath.lookahead_dist \
 FollowPath.min_lookahead_dist \
 FollowPath.max_lookahead_dist \
 FollowPath.lookahead_time \
 FollowPath.use_velocity_scaled_lookahead_dist \
 FollowPath.min_approach_linear_velocity \
 FollowPath.approach_velocity_scaling_dist \
 FollowPath.use_regulated_linear_velocity_scaling \
 FollowPath.regulated_linear_scaling_min_radius \
 FollowPath.regulated_linear_scaling_min_speed \
 FollowPath.use_cost_regulated_linear_velocity_scaling \
 FollowPath.use_rotate_to_heading \
 FollowPath.rotate_to_heading_min_angle \
 FollowPath.rotate_to_heading_angular_vel \
 FollowPath.max_angular_accel \
 FollowPath.use_collision_detection \
 FollowPath.max_allowed_time_to_collision_up_to_carrot \
 general_goal_checker.xy_goal_tolerance \
 general_goal_checker.yaw_goal_tolerance \
 progress_checker.required_movement_radius \
 progress_checker.required_movement_angle \
 progress_checker.movement_time_allowance
 do
  printf '%-62s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== SAFE TOUR ==="
for p in tour_auto_advance tour_speed_limit_mps speed_limit_topic; do
  printf '%-38s ' "$p"
  ros2 param get /semantic_nav_node "$p" 2>/dev/null || echo MISSING
done

echo "-- /speed_limit"
ros2 topic info /speed_limit --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true

echo
echo "=== RPP DEBUG TOPICS ==="
for t in /lookahead_point /lookahead_arc; do
  printf '%-24s ' "$t"
  ros2 topic info "$t" 2>/dev/null | tr '\n' ' ' || true
  echo
done

echo
echo "=== COMMAND OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "--- $t"
  ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
done

echo
echo "Expected core values:"
echo "  RPP plugin, cruise=0.45, regulated_min=0.30, approach_min=0.30"
echo "  lookahead=0.55, scaled lookahead ON, approach_dist=0.80"
echo "  tour_auto_advance=True, tour_speed_limit_mps=0.38"
echo "  motion arbiter should preserve RPP vx/wz shape (no actuator floor rewrite)"
