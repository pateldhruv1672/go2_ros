#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
[[ -f scripts/sparky_runtime_env.sh ]] && source scripts/sparky_runtime_env.sh
set -u

ok_node() { ros2 node list 2>/dev/null | grep -Fxq "$1" && echo "OK   node $1" || echo "FAIL node $1"; }
ok_topic() { ros2 topic list 2>/dev/null | grep -Fxq "$1" && echo "OK   topic $1" || echo "FAIL topic $1"; }

echo "=== V11.4 TEACH CORE ==="
ok_node /semantic_nav_node
ok_topic /map
ok_topic /scan
ok_topic /odom
ok_topic /camera/image_raw
ok_topic /point_cloud2

echo
echo "=== REGISTERED 3D OBJECT PIPELINE ==="
ok_node /go2_teach_yolo_sam2
ok_node /go2_registered_lidar_object_projector
ok_node /go2_teach_pose_aware_object_mapper
ok_node /go2_teach_world_object_memory
ok_topic /object_explorer/sam2_detections
ok_topic /go2_vln/target_detections_3d
ok_topic /go2_vln/projection_status
ok_topic /go2_vln/object_map
ok_topic /go2_memory/object_inventory

echo
echo "=== MEMORY / VLM ==="
ok_node /go2_teach_background_pose_memory
ok_node /go2_teach_vlm_backup
ok_topic /go2_vlm_checkpoint/status
printf "semantic VLM provider: "; ros2 param get /semantic_nav_node vlm_provider 2>/dev/null || true
printf "semantic VLM model:    "; ros2 param get /semantic_nav_node vlm_model 2>/dev/null || true
printf "semantic auto-save:    "; ros2 param get /semantic_nav_node auto_save_places 2>/dev/null || true
printf "semantic VLM enabled:  "; ros2 param get /semantic_nav_node auto_save_use_vlm 2>/dev/null || true
printf "pose auto checkpoints: "; ros2 param get /go2_teach_background_pose_memory auto_write_checkpoints 2>/dev/null || true
printf "VLM auto checkpoints:  "; ros2 param get /go2_teach_vlm_backup auto_write_checkpoints 2>/dev/null || true

echo
echo "=== CURRENT TEACH SESSION ==="
ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"
SESSION="$(find "$ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"
if [[ -z "$SESSION" ]]; then
  echo "FAIL no session directory under $ROOT"
  exit 1
fi
echo "session=$SESSION"

count_lines() { [[ -f "$1" ]] && wc -l < "$1" | tr -d ' ' || echo 0; }
count_files() { [[ -d "$1" ]] && find "$1" -type f | wc -l | tr -d ' ' || echo 0; }

printf "places.yaml:             "; [[ -f "$SESSION/places.yaml" ]] && echo YES || echo NO
printf "route.yaml:              "; [[ -f "$SESSION/route.yaml" ]] && echo YES || echo NO
printf "session.yaml:            "; [[ -f "$SESSION/session.yaml" ]] && echo YES || echo NO
printf "map.yaml:                "; [[ -f "$SESSION/map.yaml" ]] && echo YES || echo "not yet (normal until map save/finalize)"
printf "memory spawn:            "; [[ -f "$SESSION/memory/spawn.json" ]] && echo YES || echo NO
printf "checkpoint records:      "; count_lines "$SESSION/memory/checkpoints.jsonl"
printf "place memory records:    "; count_lines "$SESSION/memory/places.jsonl"
printf "object records:          "; count_lines "$SESSION/memory/objects.jsonl"
printf "object observations:     "; count_lines "$SESSION/memory/object_observations.jsonl"
printf "saved image artifacts:   "; count_files "$SESSION/artifacts/images"
printf "semantic place images:   "; count_files "$SESSION/images"

echo
echo "=== LATEST STATUS SAMPLES ==="
echo "projection_status:"
timeout 2 ros2 topic echo /go2_vln/projection_status --once 2>/dev/null || echo "  no sample yet"
echo "object_inventory:"
timeout 2 ros2 topic echo /go2_memory/object_inventory --once 2>/dev/null || echo "  no sample yet"
echo "VLM checkpoint status:"
timeout 2 ros2 topic echo /go2_vlm_checkpoint/status --once 2>/dev/null || echo "  no sample yet"
