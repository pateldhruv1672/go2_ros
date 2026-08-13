#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== NAV2 LIFECYCLE / ACTION ==="
for n in controller_server planner_server behavior_server bt_navigator collision_monitor; do
  printf '%-22s ' "$n"
  timeout 3 ros2 lifecycle get "/$n" 2>/dev/null || echo NOT_AVAILABLE
done
ros2 action info /navigate_to_pose 2>/dev/null || true

echo
echo "=== ROTATION SHIM + REACTIVE DWB ==="
for p in \
  controller_frequency \
  FollowPath.plugin \
  FollowPath.primary_controller \
  FollowPath.angular_dist_threshold \
  FollowPath.angular_disengage_threshold \
  FollowPath.rotate_to_heading_angular_vel \
  FollowPath.rotate_to_goal_heading \
  FollowPath.max_vel_x \
  FollowPath.min_speed_xy \
  FollowPath.max_vel_theta \
  FollowPath.acc_lim_x \
  FollowPath.acc_lim_theta \
  FollowPath.sim_time \
  FollowPath.vx_samples \
  FollowPath.vtheta_samples \
  general_goal_checker.xy_goal_tolerance \
  general_goal_checker.yaw_goal_tolerance
 do
  printf '%-52s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== PERCEPTION LATENCY PROFILE ==="
for p in enable_sam2 yolo_imgsz sam2_imgsz sam2_every_n inference_period_sec publish_annotated_image annotated_publish_period_sec; do
  printf '%-34s ' "$p"
  ros2 param get /fast_sam2_tracker_overlay_node "$p" 2>/dev/null || echo MISSING
done

echo "-- annotated topic ownership/QoS"
ros2 topic info /object_explorer/annotated_image --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:|Reliability:' || true

echo "-- annotated image rate (~4 sec sample)"
timeout 4 ros2 topic hz /object_explorer/annotated_image 2>/dev/null || true

echo
echo "Expected core values:"
echo "  controller_frequency=25"
echo "  FollowPath.plugin=RotationShimController"
echo "  FollowPath.primary_controller=dwb_core::DWBLocalPlanner"
echo "  shim rotate=0.80 rad/s; trigger=0.30; release=0.08"
echo "  DWB max_x=0.50; min_speed_xy=0.30; sim_time=0.80"
echo "  SAM2=true but sam2_every_n=4; YOLO=416; SAM2=384"
echo "  annotated image publish=true at up to ~20 Hz, queue depth 1 in RViz"
