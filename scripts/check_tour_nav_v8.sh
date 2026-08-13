#!/usr/bin/env bash
set -u

echo '=== RVIZ ==='
ros2 node list 2>/dev/null | grep -E '/sparky_tour_rviz$|rviz' || true

echo; echo '=== AGENT -> SEMANTIC NAV ==='
ros2 topic info /semantic_nav/command -v 2>/dev/null || true

echo; echo '=== NAV2 ==='
ros2 action info /navigate_to_pose 2>/dev/null || true
ros2 action info /compute_path_to_pose 2>/dev/null || true

echo; echo '=== OBJECT MEMORY / MAP ==='
ros2 topic info /go2_vln/object_map 2>/dev/null || true
ros2 topic info /go2_memory/object_inventory 2>/dev/null || true
ros2 topic echo /go2_vln/projection_status --once 2>/dev/null || true

echo; echo '=== MOTION ==='
ros2 topic info /motion_skills/command -v 2>/dev/null || true

echo; echo '=== INTERACTION ==='
ros2 topic info /go2_agent/interaction_state 2>/dev/null || true
