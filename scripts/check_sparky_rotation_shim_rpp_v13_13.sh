#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== PLUGINS ==="
ros2 pkg prefix nav2_rotation_shim_controller 2>/dev/null || echo MISSING_rotation_shim
ros2 pkg prefix nav2_regulated_pure_pursuit_controller 2>/dev/null || echo MISSING_rpp

echo
echo "=== CONTROLLER ==="
for p in \
 controller_frequency \
 min_x_velocity_threshold \
 min_theta_velocity_threshold \
 FollowPath.plugin \
 FollowPath.angular_dist_threshold \
 FollowPath.angular_disengage_threshold \
 FollowPath.forward_sampling_distance \
 FollowPath.rotate_to_heading_angular_vel \
 FollowPath.max_angular_accel \
 FollowPath.rotate_to_goal_heading \
 FollowPath.closed_loop \
 FollowPath.primary_controller.plugin \
 FollowPath.primary_controller.desired_linear_vel \
 FollowPath.primary_controller.lookahead_dist \
 FollowPath.primary_controller.min_lookahead_dist \
 FollowPath.primary_controller.max_lookahead_dist \
 FollowPath.primary_controller.lookahead_time \
 FollowPath.primary_controller.regulated_linear_scaling_min_speed \
 FollowPath.primary_controller.regulated_linear_scaling_min_radius \
 FollowPath.primary_controller.use_rotate_to_heading \
 FollowPath.primary_controller.rotate_to_heading_min_angle \
 FollowPath.primary_controller.rotate_to_heading_angular_vel \
 FollowPath.primary_controller.allow_reversing \
 general_goal_checker.xy_goal_tolerance \
 general_goal_checker.yaw_goal_tolerance \
 progress_checker.movement_time_allowance
 do
  printf '%-68s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
 done

echo
echo "=== TOUR ==="
for p in tour_auto_advance tour_speed_limit_mps speed_limit_topic; do
  printf '%-35s ' "$p"
  ros2 param get /semantic_nav_node "$p" 2>/dev/null || echo MISSING
 done

echo
echo "=== OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "-- $t"
  ros2 topic info "$t" 2>/dev/null | grep -E 'Publisher count|Subscription count' || true
 done

echo
echo "Expected core values:"
echo "  FollowPath = RotationShimController"
echo "  primary_controller = RegulatedPurePursuitController"
echo "  shim rotate = 0.75 rad/s, trigger 0.35 rad, disengage 0.12 rad"
echo "  RPP cruise = 0.48 m/s, lookahead 0.35 m, regulated minimum 0.36 m/s"
echo "  Tour cap = 0.36 m/s"
