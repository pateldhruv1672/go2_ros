#!/usr/bin/env bash
set -eo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
source scripts/sparky_runtime_env.sh

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

nodes="$(ros2 node list 2>/dev/null || true)"
actions="$(ros2 action list 2>/dev/null || true)"
topics="$(ros2 topic list 2>/dev/null || true)"

node_check() { if grep -qx "$1" <<<"$nodes"; then echo "OK   node $1"; else echo "FAIL node $1"; fi; }
action_check() { if grep -qx "$1" <<<"$actions"; then echo "OK   action $1"; else echo "FAIL action $1"; fi; }
topic_check() { if grep -qx "$1" <<<"$topics"; then echo "OK   topic $1"; else echo "FAIL topic $1"; fi; }

echo "=== CONTROL PLANE ==="
node_check /semantic_nav_node
node_check /resume_map_server
node_check /amcl
node_check /planner_server
node_check /controller_server
node_check /bt_navigator
node_check /collision_monitor
action_check /navigate_to_pose
action_check /compute_path_to_pose

subs="$(ros2 topic info /semantic_nav/command 2>/dev/null | awk '/Subscription count:/ {print $3}' | tail -1)"
echo "semantic_nav command subscriptions=${subs:-0}"

echo
echo "=== MAP / TF ==="
if grep -qx /map <<<"$topics"; then
  ros2 topic echo /map --once --field info 2>/dev/null | head -18 || true
else
  echo "FAIL /map missing"
fi
ros2 param get /resume_map_server yaml_filename 2>/dev/null || true
timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | tail -15 || true

echo
echo "=== PERCEPTION ==="
node_check /go2_resume_yolo_sam2
node_check /go2_registered_cloud_object_projector
node_check /go2_resume_pose_aware_object_mapper
node_check /go2_resume_world_object_memory
topic_check /object_explorer/annotated_image
topic_check /object_explorer/sam2_detections
topic_check /go2_vln/target_detections_3d
topic_check /go2_vln/projection_status
topic_check /go2_vln/object_map
topic_check /go2_vln/object_markers
topic_check /go2_memory/object_inventory

echo
echo "=== VOICE / SPEECH ==="
node_check /go2_agentic_voice_action
node_check /go2_phone_web_gateway
node_check /go2_speech_arbiter
node_check /go2_tts_node
topic_check /go2_voice/transcript
topic_check /go2_tts/say

if grep -qx /go2_vln/projection_status <<<"$topics"; then
  echo "projection_status:"
  timeout 3 ros2 topic echo /go2_vln/projection_status --once 2>/dev/null || true
fi
if grep -qx /go2_memory/object_inventory <<<"$topics"; then
  echo "object_inventory:"
  timeout 3 ros2 topic echo /go2_memory/object_inventory --once 2>/dev/null || true
fi

echo
echo "=== ORCHESTRATION / LIVE VLM ==="
node_check /go2_resume_vlm_backup
topic_check /go2_vlm/query
topic_check /go2_vlm/query_result
topic_check /go2_agent/events
topic_check /go2_agent/interaction_state
topic_check /go2_speech/status
topic_check /motion_skills/status
topic_check /go2_tour/host_command
topic_check /go2_tour/host_status
if grep -qx /go2_vlm/query_result <<<"$topics"; then
  echo "latest live VLM result (if a query has been made):"
  timeout 2 ros2 topic echo /go2_vlm/query_result --once 2>/dev/null || true
fi

echo
echo "=== DASHBOARD / MAP UI ==="
node_check /go2_phone_web_gateway
if command -v curl >/dev/null 2>&1; then
  if curl -fsS --max-time 2 http://127.0.0.1:8765/api/health >/dev/null; then
    echo "OK   dashboard http://127.0.0.1:8765"
  else
    echo "FAIL dashboard HTTP health"
  fi
else
  echo "WARN curl missing; skipping dashboard HTTP health"
fi
topic_check /amcl_pose
topic_check /map

echo
echo "=== RVIZ ==="
node_check /sparky_tour_rviz
RVIZ="$(ros2 pkg prefix go2_semantic_nav_agent)/share/go2_semantic_nav_agent/config/semantic_nav.rviz"
echo "config=$RVIZ"
grep -nE '/map|/scan_nav|/object_explorer/annotated_image|/go2_vln/object_markers' "$RVIZ" 2>/dev/null || true
