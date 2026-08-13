#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== STRICT PATH ==="
for p in   controller_frequency   general_goal_checker.xy_goal_tolerance   general_goal_checker.yaw_goal_tolerance   FollowPath.plugin   FollowPath.angular_dist_threshold   FollowPath.angular_disengage_threshold   FollowPath.rotate_to_heading_angular_vel   FollowPath.primary_controller.plugin   FollowPath.primary_controller.max_vel_x   FollowPath.primary_controller.max_vel_theta   FollowPath.primary_controller.sim_time   FollowPath.primary_controller.PathAlign.scale   FollowPath.primary_controller.PathDist.scale   FollowPath.primary_controller.GoalAlign.scale   FollowPath.primary_controller.GoalDist.scale
do
  printf '%-64s ' "$p"
  ros2 param get /controller_server "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== FOLLOWPATH PARAM LIST ==="
ros2 param list /controller_server 2>/dev/null | grep FollowPath | sort || true

echo
echo "=== PERCEPTION PARAMS ==="
for p in enable_sam2 yolo_imgsz sam2_imgsz sam2_every_n inference_period_sec max_detections; do
  printf '%-28s ' "$p"
  ros2 param get /fast_sam2_tracker_overlay_node "$p" 2>/dev/null || echo MISSING
done

echo
echo "=== RAW CAMERA ==="
ros2 topic info /camera/image_raw --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:|Reliability:' || true

echo
echo "=== ANNOTATED CAMERA ==="
ros2 topic info /object_explorer/annotated_image --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:|Reliability:' || true

echo
echo "=== ANNOTATED RATE (5 sec) ==="
timeout 5 ros2 topic hz /object_explorer/annotated_image 2>/dev/null || true

echo
echo "Expected:"
echo "  max vx = 0.375 m/s"
echo "  PathAlign = 32, PathDist = 40"
echo "  SAM2=True, YOLO=416, SAM2=384, every_n=3"
echo "  annotated publisher Reliability=RELIABLE"
