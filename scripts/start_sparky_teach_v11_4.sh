#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

# Source setup files with nounset disabled because ROS setup.bash is not nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
[[ -f scripts/sparky_runtime_env.sh ]] && source scripts/sparky_runtime_env.sh
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# One provider contract for both semantic-place labeling and VLM checkpoints.
# Switch per run without code changes:
#   GO2_TEACH_VLM_PROVIDER=openrouter
#   GO2_TEACH_VLM_PROVIDER=ollama
VLM_PROVIDER="${GO2_TEACH_VLM_PROVIDER:-${GO2_LIVE_VLM_PROVIDER:-openrouter}}"
VLM_PROVIDER="$(tr '[:upper:]' '[:lower:]' <<<"$VLM_PROVIDER")"
case "$VLM_PROVIDER" in
  local_ollama) VLM_PROVIDER="ollama" ;;
  openrouter|ollama|offline) ;;
  *)
    echo "ERROR: GO2_TEACH_VLM_PROVIDER must be openrouter, ollama, or offline; got '$VLM_PROVIDER'" >&2
    exit 3
    ;;
esac

case "$VLM_PROVIDER" in
  openrouter)
    VLM_MODEL="${GO2_TEACH_VLM_MODEL:-${GO2_LIVE_VLM_MODEL:-google/gemini-2.5-flash}}"
    VLM_BASE_URL="${GO2_TEACH_VLM_BASE_URL:-${OPENROUTER_BASE_URL:-}}"
    if [[ -z "${OPENROUTER_API_KEY:-}" ]]; then
      echo "ERROR: OPENROUTER_API_KEY is not set for GO2_TEACH_VLM_PROVIDER=openrouter." >&2
      exit 3
    fi
    ;;
  ollama)
    VLM_MODEL="${GO2_TEACH_VLM_MODEL:-${OLLAMA_VLM_MODEL:-gemma4:12b}}"
    VLM_BASE_URL="${GO2_TEACH_VLM_BASE_URL:-${OLLAMA_CHAT_URL:-http://127.0.0.1:11434/api/chat}}"
    ;;
  offline)
    VLM_MODEL="${GO2_TEACH_VLM_MODEL:-offline}"
    VLM_BASE_URL=""
    ;;
esac

# Keep the live-query path aligned with the selected provider by default.
export GO2_LIVE_VLM_PROVIDER="$VLM_PROVIDER"
export GO2_LIVE_VLM_MODEL="$VLM_MODEL"

MAP_LABEL="${MAP_LABEL:-digital_twin_lab_fresh}"
SESSION_ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"

# Require live mapping. semantic_nav_teach_world records semantics; it does not own SLAM.
MAP_INFO="$(ros2 topic info /map 2>/dev/null || true)"
if ! grep -Eq 'Publisher count: [1-9]' <<<"$MAP_INFO"; then
  cat >&2 <<MSG
ERROR: /map has no publisher.
Start the BASE stack in Teach/SLAM mode first, then run this script.
Example:
  BASE_MODE=teach bash scripts/run_robot_live.sh
MSG
  exit 4
fi

for topic in /camera/image_raw /camera/camera_info /point_cloud2 /scan /odom /tf; do
  if ! ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "ERROR: required Teach input missing: $topic" >&2
    exit 5
  fi
done

echo "============================================================"
echo " SPARKY V11.4 FULL SEMANTIC TEACH"
echo "============================================================"
echo "map_label=$MAP_LABEL"
echo "session_root=$SESSION_ROOT"
echo "VLM=$VLM_PROVIDER / $VLM_MODEL"
echo "VLM base=${VLM_BASE_URL:-provider-default}"
[[ "$VLM_PROVIDER" == "openrouter" ]] && echo "OpenRouter key=present (not printed)"
echo "records: map + TF pose + VLM places + VLM checkpoints + registered 3D objects"
echo "memory: places + checkpoints + objects + graph + vector + voxel + spawn"
echo "tour_mode=false during general mapping; curate tour stops separately"
echo "============================================================"

launch_args=(
  "map_label:=$MAP_LABEL"
  "session_root:=$SESSION_ROOT"
  "semantic_rviz:=true"
  "clear_places_on_start:=true"
  "auto_save_places:=true"
  "auto_save_interval_sec:=${TEACH_VLM_PLACE_PERIOD_SEC:-12.0}"
  "auto_save_use_vlm:=true"
  "auto_save_min_distance_m:=${TEACH_VLM_PLACE_MIN_DISTANCE_M:-1.25}"
  "enable_background_checkpoints:=true"
  "background_checkpoint_period_sec:=${TEACH_POSE_CHECKPOINT_PERIOD_SEC:-5.0}"
  "enable_vlm_backup:=true"
  "vlm_provider:=$VLM_PROVIDER"
  "vlm_model:=$VLM_MODEL"
  "vlm_timeout_sec:=${TEACH_VLM_TIMEOUT_SEC:-30.0}"
  "vlm_checkpoint_period_sec:=${TEACH_VLM_CHECKPOINT_PERIOD_SEC:-30.0}"
)

# ROS 2 CLI rejects an explicitly empty launch override such as `vlm_base_url:=`.
# Omit the override when empty so the launch/VLMClient provider default is used.
if [[ -n "${VLM_BASE_URL:-}" ]]; then
  launch_args+=("vlm_base_url:=$VLM_BASE_URL")
fi

exec ros2 launch go2_semantic_nav_agent semantic_nav_teach_world.launch.py "${launch_args[@]}"
