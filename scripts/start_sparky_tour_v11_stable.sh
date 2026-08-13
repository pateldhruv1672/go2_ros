#!/usr/bin/env bash
set -eo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

# ROS setup scripts are not nounset-safe. Enable -u only after sourcing.
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
source scripts/sparky_runtime_env.sh
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export SPARKY_WEB_TOKEN="${SPARKY_WEB_TOKEN:-0000}"
export SPARKY_ADMIN_PIN="${SPARKY_ADMIN_PIN:-0000}"
export SPARKY_PHONE_WEB_BIND_HOST="${SPARKY_PHONE_WEB_BIND_HOST:-0.0.0.0}"
export SPARKY_ENABLE_RVIZ="${SPARKY_ENABLE_RVIZ:-true}"
export SPARKY_ENABLE_DASHBOARD="${SPARKY_ENABLE_DASHBOARD:-true}"
export SPARKY_RESET_ROS_DAEMON_ON_START="${SPARKY_RESET_ROS_DAEMON_ON_START:-true}"
export GO2_AGENT_OLLAMA_MODEL="${GO2_AGENT_OLLAMA_MODEL:-llama3.2:3b}"
export GO2_LIVE_VLM_PROVIDER="${GO2_LIVE_VLM_PROVIDER:-openrouter}"
export GO2_LIVE_VLM_MODEL="${GO2_LIVE_VLM_MODEL:-google/gemini-2.5-flash}"
export GO2_SAFETY_SCAN_TOPIC="${GO2_SAFETY_SCAN_TOPIC:-/scan_nav}"
export GO2_COLLISION_SOURCE_TIMEOUT_SEC="${GO2_COLLISION_SOURCE_TIMEOUT_SEC:-1.6}"

if [[ -z "${SESSION:-}" ]]; then
  preferred="$HOME/.ros/go2_semantic_nav_sessions/digital_twin_lab_20260808_024634"
  if [[ -f "$preferred/map.yaml" ]]; then
    SESSION="$(basename "$preferred")"
  else
    latest="$(find "$HOME/.ros/go2_semantic_nav_sessions" -mindepth 2 -maxdepth 2 -name map.yaml -printf '%T@ %h\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
    if [[ -z "$latest" ]]; then
      echo "ERROR: no saved session containing map.yaml found." >&2
      exit 2
    fi
    SESSION="$(basename "$latest")"
  fi
fi
export SESSION
SESSION_DIR="$HOME/.ros/go2_semantic_nav_sessions/$SESSION"
if [[ "$SPARKY_RESET_ROS_DAEMON_ON_START" == "true" ]]; then
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
fi

if [[ ! -f "$SESSION_DIR/map.yaml" ]]; then
  echo "ERROR: selected session has no map.yaml: $SESSION_DIR" >&2
  exit 3
fi

echo "============================================================"
echo " SPARKY V11 STABLE TOUR"
echo "============================================================"
echo "session=$SESSION"
echo "map=$SESSION_DIR/map.yaml"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "safety_scan=$GO2_SAFETY_SCAN_TOPIC collision_timeout=$GO2_COLLISION_SOURCE_TIMEOUT_SEC"
echo "rviz=config/semantic_nav.rviz (owned by tour core)"
echo "dashboard=http://$(hostname -I 2>/dev/null | awk '{print $1}'):${SPARKY_PHONE_WEB_PORT:-8765}"
echo "rviz_enabled=$SPARKY_ENABLE_RVIZ dashboard_enabled=$SPARKY_ENABLE_DASHBOARD"
echo "============================================================"
cat "$SESSION_DIR/map.yaml"
echo
if [[ "$GO2_LIVE_VLM_PROVIDER" == "openrouter" && -z "${OPENROUTER_API_KEY:-}" ]]; then
  echo "WARNING: GO2_LIVE_VLM_PROVIDER=openrouter but OPENROUTER_API_KEY is not set." >&2
  echo "Live vision questions will return an explicit VLM error instead of a cached/deterministic answer." >&2
fi

# Base stack must already be running. Fail early instead of silently launching a dead Tour.
TOPICS="$(ros2 topic list 2>/dev/null || true)"
missing=0
for topic in /odom /scan /point_cloud2 /camera/image_raw /camera/camera_info; do
  if ! grep -qx "$topic" <<<"$TOPICS"; then
    echo "ERROR: base topic missing: $topic" >&2
    missing=1
  fi
done
if [[ "$missing" -ne 0 ]]; then
  echo "Start go2_robot_sdk robot.launch.py with slam:=false nav2:=false using the SAME DDS settings, then retry." >&2
  exit 4
fi

exec ros2 launch go2_semantic_nav_agent semantic_nav_tour_world_core.launch.py \
  session_name:="$SESSION" \
  rviz2:="$SPARKY_ENABLE_RVIZ" \
  restore_spawn_on_start:=true \
  enable_object_perception:=true \
  enable_sam2:=true \
  enable_vlm_backup:=true \
  vlm_provider:="$GO2_LIVE_VLM_PROVIDER" \
  vlm_model:="$GO2_LIVE_VLM_MODEL" \
  enable_agentic_voice:=true \
  ollama_model:="$GO2_AGENT_OLLAMA_MODEL" \
  require_motion_skill_confirmation:=false \
  motion_confirmation_policy:=risky_only \
  enable_phone_web:="$SPARKY_ENABLE_DASHBOARD" \
  phone_web_bind_host:="$SPARKY_PHONE_WEB_BIND_HOST" \
  phone_web_port:=8765 \
  phone_web_require_token:=true \
  phone_admin_pin:="${SPARKY_ADMIN_PIN}" \
  enable_motion_skills:=true \
  allow_motion_during_navigation:=false \
  allow_medium_risk_motion:=true \
  allow_high_risk_motion:="${SPARKY_ALLOW_HIGH_RISK_MOTION:-false}"
