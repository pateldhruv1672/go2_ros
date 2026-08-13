#!/usr/bin/env bash
set -eo pipefail

echo "=== V11.5 AMCL INPUT CONTRACT ==="
printf "%-28s " "AMCL scan_topic"
ros2 param get /amcl scan_topic 2>/dev/null || true
printf "%-28s " "AMCL base_frame_id"
ros2 param get /amcl base_frame_id 2>/dev/null || true
printf "%-28s " "AMCL odom_frame_id"
ros2 param get /amcl odom_frame_id 2>/dev/null || true
printf "%-28s " "AMCL global_frame_id"
ros2 param get /amcl global_frame_id 2>/dev/null || true

echo
echo "Expected: AMCL scan_topic = /scan"
echo "Safety/costmap scan remains /scan_nav."

echo
echo "=== TOPIC OWNERSHIP ==="
for t in /scan /scan_nav /amcl_pose /tf /tf_static; do
  echo "--- $t ---"
  ros2 topic info "$t" 2>/dev/null || true
done

echo
echo "=== RATES (short sample) ==="
for t in /scan /scan_nav /odom; do
  echo "--- $t ---"
  timeout 4 ros2 topic hz "$t" 2>/dev/null || true
done

echo
echo "=== AMCL POSE / COVARIANCE ==="
timeout 4 ros2 topic echo /amcl_pose --once 2>/dev/null || true

echo
echo "=== TF map -> base_link ==="
timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null || true

echo
echo "=== IMPORTANT ==="
echo "In RViz the red display is now 'AMCL Raw Scan' on /scan."
echo "After 2D Pose Estimate, red scan points should sit on the saved map walls."
echo "Do not send a Nav2 goal if the scan is visibly translated/rotated off the walls."
