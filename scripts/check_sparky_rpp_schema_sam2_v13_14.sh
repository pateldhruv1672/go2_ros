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
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done

echo
echo "=== ROTATION SHIM + RPP (CORRECT JAZZY NAMESPACE) ==="
for p in \
  controller_frequency \
  FollowPath.plugin \
  FollowPath.primary_controller \
  FollowPath.angular_dist_threshold \
  FollowPath.angular_disengage_threshold \
  FollowPath.rotate_to_heading_angular_vel \
  FollowPath.desired_linear_vel \
  FollowPath.lookahead_dist \
  FollowPath.min_lookahead_dist \
  FollowPath.max_lookahead_dist \
  FollowPath.lookahead_time \
  FollowPath.regulated_linear_scaling_min_speed \
  FollowPath.regulated_linear_scaling_min_radius \
  FollowPath.use_rotate_to_heading \
  FollowPath.rotate_to_heading_min_angle \
  FollowPath.allow_reversing
 do
  printf '%-55s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== NAVIGATE_TO_POSE ==="
ros2 action info /navigate_to_pose 2>/dev/null || true

echo
echo "=== CMD_VEL OWNERSHIP ==="
for t in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out; do
  echo "-- $t"
  ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
done

echo
echo "=== OBJECT PERCEPTION LOAD ==="
if ros2 node list 2>/dev/null | grep -qx /fast_sam2_tracker_overlay_node; then
  printf 'enable_sam2:             '; ros2 param get /fast_sam2_tracker_overlay_node enable_sam2 2>/dev/null || true
  printf 'publish_annotated_image: '; ros2 param get /fast_sam2_tracker_overlay_node publish_annotated_image 2>/dev/null || true
  printf 'inference_period_sec:    '; ros2 param get /fast_sam2_tracker_overlay_node inference_period_sec 2>/dev/null || true
else
  echo 'object perception node not running'
fi

echo
echo "Expected core values:"
echo "  controller_server/planner_server/bt_navigator/collision_monitor = active"
echo "  FollowPath.primary_controller = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
echo "  FollowPath.desired_linear_vel = 0.48"
echo "  /navigate_to_pose Action servers: 1 (/bt_navigator)"
echo "  enable_sam2 = False"
echo "  publish_annotated_image = False"
