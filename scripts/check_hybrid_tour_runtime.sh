#!/usr/bin/env bash
set -u
echo '=== RVIZ ==='
ros2 node list 2>/dev/null | grep -E 'sparky_tour_rviz|rviz2' || true
echo '=== HYBRID AGENT ==='
ros2 node list 2>/dev/null | grep -E 'agentic_voice_action|semantic_nav_node|speech_arbiter' || true
echo '=== OBJECT PIPELINE ==='
ros2 topic info /go2_vln/target_detections_3d 2>/dev/null || true
ros2 topic info /go2_vln/object_map 2>/dev/null || true
ros2 topic echo /go2_vln/projection_status --once 2>/dev/null || true
echo '=== NAV2 PLANNER ==='
ros2 action info /compute_path_to_pose 2>/dev/null || true
