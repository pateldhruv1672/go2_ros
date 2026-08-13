#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

if [ -f "$ROOT_DIR/.env.local" ]; then set -a; . "$ROOT_DIR/.env.local"; set +a; fi
source "$SCRIPT_DIR/sparky_ros_env.sh" 2>/dev/null || true
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$ROOT_DIR/src/.venv/bin/activate" ] && source "$ROOT_DIR/src/.venv/bin/activate"
source "$ROOT_DIR/install/setup.bash"
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export SPARKY_WEB_TOKEN="${SPARKY_WEB_TOKEN:-0000}"
export SPARKY_ADMIN_PIN="${SPARKY_ADMIN_PIN:-$SPARKY_WEB_TOKEN}"

# SPARKY_TRUE_AGENTIC_NAV_PROFILE_V12_8
export GO2_TOUR_AUTO_ADVANCE="${GO2_TOUR_AUTO_ADVANCE:-1}"
export GO2_TOUR_PAUSE_SEC="${GO2_TOUR_PAUSE_SEC:-2.0}"
export GO2_NAV_MIN_SPEED_XY="${GO2_NAV_MIN_SPEED_XY:-0.30}"
export GO2_NAV_MIN_SPEED_THETA="${GO2_NAV_MIN_SPEED_THETA:-0.22}"
export GO2_NAV_MAX_X="${GO2_NAV_MAX_X:-0.40}"
export GO2_NAV_MAX_THETA="${GO2_NAV_MAX_THETA:-0.50}"
export GO2_NAV_ACC_X="${GO2_NAV_ACC_X:-0.45}"
export GO2_NAV_ACC_THETA="${GO2_NAV_ACC_THETA:-0.80}"
export GO2_NAV_DECEL_X="${GO2_NAV_DECEL_X:-0.45}"
export GO2_NAV_DECEL_THETA="${GO2_NAV_DECEL_THETA:-0.90}"
export GO2_NAV_SIM_TIME="${GO2_NAV_SIM_TIME:-1.1}"
export GO2_NAV_GOAL_XY_TOLERANCE_M="${GO2_NAV_GOAL_XY_TOLERANCE_M:-0.18}"
export GO2_NAV_GOAL_YAW_TOLERANCE_RAD="${GO2_NAV_GOAL_YAW_TOLERANCE_RAD:-0.30}"
export GO2_NAV_PROGRESS_RADIUS_M="${GO2_NAV_PROGRESS_RADIUS_M:-0.05}"
export GO2_NAV_PROGRESS_ANGLE_RAD="${GO2_NAV_PROGRESS_ANGLE_RAD:-0.08}"
export GO2_NAV_PROGRESS_TIMEOUT_SEC="${GO2_NAV_PROGRESS_TIMEOUT_SEC:-15.0}"
export GO2_SENSOR_FRESH_FRAME_WAIT_SEC="${GO2_SENSOR_FRESH_FRAME_WAIT_SEC:-1.5}"
export SPARKY_REPAIR_OBJECT_JSONL_ON_START="${SPARKY_REPAIR_OBJECT_JSONL_ON_START:-1}"


SESSION_ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"
#REQUESTED_
# SPARKY_KILL_STALE_ARBITER_V13_4
# Do not allow a motion arbiter from a previous Resume invocation to remain
# subscribed to /cmd_vel_nav2 or publishing /cmd_vel_nav.
pkill -f 'go2_nav_tools/.*/motion_arbiter|/motion_arbiter([[:space:]]|$)' 2>/dev/null || true
sleep 0.5

SESSION_NAME="${SESSION_NAME:-}"
SESSION_NAME="${SESSION_NAME:-}"
if [ -z "$SESSION_NAME" ] || [ "$SESSION_NAME" = auto ] || [ "$SESSION_NAME" = latest ]; then
  SESSION_NAME="$(find "$SESSION_ROOT" -mindepth 1 -maxdepth 1 -type d -exec test -f '{}/map.yaml' ';' -printf '%T@ %f\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[ -n "$SESSION_NAME" ] || { echo "No resume-ready session with map.yaml found under $SESSION_ROOT" >&2; exit 2; }
[ -f "$SESSION_ROOT/$SESSION_NAME/map.yaml" ] || { echo "Session $SESSION_NAME has no map.yaml" >&2; exit 2; }
export SESSION_NAME

if [ "$SPARKY_REPAIR_OBJECT_JSONL_ON_START" = "1" ] && [ -x "$SCRIPT_DIR/repair_sparky_world_memory_jsonl.sh" ]; then
  "$SCRIPT_DIR/repair_sparky_world_memory_jsonl.sh" "$SESSION_NAME" >/tmp/sparky_object_jsonl_repair.log 2>&1 || {
    echo "[sparky] WARNING: object JSONL repair failed; see /tmp/sparky_object_jsonl_repair.log" >&2
  }
fi

VLM_PROVIDER="${SPARKY_VLM_PROVIDER:-${GO2_TEACH_VLM_PROVIDER:-openrouter}}"
VLM_MODEL="${SPARKY_VLM_MODEL:-${GO2_TEACH_VLM_MODEL:-google/gemini-2.5-flash}}"
if [ "$VLM_PROVIDER" = openrouter ] && [ -z "${OPENROUTER_API_KEY:-}" ] && [ -f "$ROOT_DIR/openrouter_api.key" ]; then
  export OPENROUTER_API_KEY="$(head -n 1 "$ROOT_DIR/openrouter_api.key" | tr -d '\r\n')"
fi
VLM_BASE_URL="${SPARKY_VLM_BASE_URL:-}"
if [ "$VLM_PROVIDER" = ollama ] && [ -z "$VLM_BASE_URL" ]; then
  VLM_BASE_URL="${OLLAMA_CHAT_URL:-http://127.0.0.1:11434/api/chat}"
fi

YOLO_MODEL="${SPARKY_YOLO_MODEL:-$ROOT_DIR/yolov8n.pt}"
SAM2_MODEL="${SPARKY_SAM2_MODEL:-$ROOT_DIR/sam2_t.pt}"
ENABLE_SAM2="${ENABLE_SAM2:-1}"
[ -f "$SAM2_MODEL" ] || ENABLE_SAM2=0
ENABLE_OBJECT_PERCEPTION="${ENABLE_OBJECT_PERCEPTION:-1}"
ENABLE_OBJECT_OVERLAY="${ENABLE_OBJECT_OVERLAY:-0}"
ENABLE_RICH_MEMORY="${ENABLE_RICH_MEMORY:-0}"
ENABLE_MOTION_SKILLS="${ENABLE_MOTION_SKILLS:-1}"
ENABLE_FRONT_FLIP="${ENABLE_FRONT_FLIP:-1}"
ENABLE_LLM_DEBATE="${ENABLE_LLM_DEBATE:-0}"
OBJECT_DEVICE="${OBJECT_DEVICE:-cuda:0}"
SPARKY_YOLO_IMGSZ="${SPARKY_YOLO_IMGSZ:-416}"
SPARKY_SAM2_IMGSZ="${SPARKY_SAM2_IMGSZ:-384}"
SPARKY_SAM2_EVERY_N="${SPARKY_SAM2_EVERY_N:-4}"
SPARKY_PERCEPTION_PERIOD_SEC="${SPARKY_PERCEPTION_PERIOD_SEC:-0.10}"
SPARKY_ANNOTATED_PERIOD_SEC="${SPARKY_ANNOTATED_PERIOD_SEC:-0.05}"
PHONE_PORT="${PHONE_PORT:-8765}"
LOCAL_SPEAKER_BACKEND="${LOCAL_SPEAKER_BACKEND:-auto}"

node_exists(){ ros2 node list 2>/dev/null | grep -qx "$1"; }
lifecycle_active(){ timeout 2 ros2 lifecycle get "$1" 2>/dev/null | grep -q 'active'; }
# SPARKY_ACTIVE_RESUME_READY_V13_16_5
node_active(){
  local node="$1"
  timeout 2 ros2 lifecycle get "$node" 2>/dev/null | grep -q '^active \[3\]$'
}
resume_ready(){
  node_exists /semantic_nav_node &&
  node_exists /amcl &&
  node_active /controller_server &&
  node_active /planner_server &&
  node_active /bt_navigator &&
  node_active /collision_monitor
}
param_string(){ ros2 param get "$1" "$2" 2>/dev/null | sed -n 's/^String value is: //p' | head -1; }

if node_exists /semantic_nav_node; then
  ACTIVE_MODE="$(param_string /semantic_nav_node mode || true)"
  if [ -n "$ACTIVE_MODE" ] && [ "$ACTIVE_MODE" != "resume" ]; then
    echo "[sparky] semantic_nav_node is running in mode=$ACTIVE_MODE. Refusing to kill/replace an active Teach stack." >&2
    exit 4
  fi
fi

# Remove only stale agentic overlay owners; do not kill Resume/Nav2/AMCL/base.
# SPARKY_PERCEPTION_CLEANUP_V13_16
pkill -f 'fast_sam2_tracker_overlay_node|go2_pose_aware_object_mapper' 2>/dev/null || true
# SPARKY_CLEAN_STALE_TTS_V13_3
# Repeated overlay launches could leave old TTS executables alive.
pkill -f 'go2_omi_voice_bridge/.*/tts_node|go2_tts_node' 2>/dev/null || true
pkill -f 'go2_omi_voice_bridge/.*/speech_arbiter_node|go2_speech_arbiter' 2>/dev/null || true
pkill -f "ros2 launch go2_omi_voice_bridge resume_agentic_tour_overlay.launch.py" 2>/dev/null || true
pkill -f "go2_unified_intent_gate|go2_phone_web_gateway|go2_speech_arbiter|go2_tour_host_script|go2_langgraph_main_supervisor|go2_world_object_memory|go2_vlm_checkpoint_node" 2>/dev/null || true
sleep 1

STARTED_RESUME=0
RESUME_PID=""
if ! resume_ready; then
  echo "[sparky] Resume/Nav2 not active -> starting existing semantic Resume stack for session: $SESSION_NAME"
  RESUME_LOG="/tmp/sparky_resume_agentic_tour_resume.log"
  SESSION_NAME="$SESSION_NAME" RVIZ2=false bash "$SCRIPT_DIR/run_semantic_nav_resume.sh" >"$RESUME_LOG" 2>&1 &
  RESUME_PID=$!
  STARTED_RESUME=1
  for _ in $(seq 1 90); do
    resume_ready && break
    kill -0 "$RESUME_PID" 2>/dev/null || { echo "Resume launcher exited; see $RESUME_LOG" >&2; tail -n 80 "$RESUME_LOG" >&2 || true; exit 3; }
    sleep 1
  done
  resume_ready || { echo "Resume/Nav2 did not become ready; see $RESUME_LOG" >&2; tail -n 80 "$RESUME_LOG" >&2 || true; exit 3; }
else
  echo "[sparky] Existing Resume/Nav2 detected -> attaching agentic Tour overlay without launching a second navigation stack"
  ACTIVE_SESSION="$(param_string /semantic_nav_node session_name || true)"
  if [ -n "$ACTIVE_SESSION" ] && [ "$ACTIVE_SESSION" != "$SESSION_NAME" ]; then
    if [ -n "$REQUESTED_SESSION_NAME" ] && [ "$REQUESTED_SESSION_NAME" != auto ] && [ "$REQUESTED_SESSION_NAME" != latest ]; then
      echo "[sparky] WARNING: requested session '$SESSION_NAME' differs from active Resume session '$ACTIVE_SESSION'; binding agent/VLM/dashboard to the active session to avoid split memory/navigation state."
    fi
    SESSION_NAME="$ACTIVE_SESSION"
    export SESSION_NAME
  fi
fi

RVIZ_PID=""
if ! node_exists /semantic_nav_rviz2; then
  if pgrep -x rviz2 >/dev/null 2>&1; then
    echo "[sparky] Another RViz is already running; not launching a duplicate. Close it and rerun if you want semantic_nav.rviz."
  else
    ros2 launch go2_semantic_nav_agent semantic_nav_rviz.launch.py > /tmp/sparky_semantic_rviz.log 2>&1 &
    RVIZ_PID=$!
  fi
fi

cleanup(){
  rc=$?
  [ -n "${RVIZ_PID:-}" ] && kill "$RVIZ_PID" 2>/dev/null || true
  if [ "${STARTED_RESUME:-0}" = 1 ] && [ -n "${RESUME_PID:-}" ]; then kill "$RESUME_PID" 2>/dev/null || true; fi
  exit $rc
}
trap cleanup INT TERM EXIT

LAN_IP="$(hostname -I 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i!~/^127\./){print $i; exit}}')"
LAN_IP="${LAN_IP:-127.0.0.1}"
if ! command -v espeak-ng >/dev/null 2>&1 && ! command -v espeak >/dev/null 2>&1 && ! command -v spd-say >/dev/null 2>&1 && ! command -v say >/dev/null 2>&1; then
  echo "[sparky] WARNING: no local TTS executable found; dashboard/agent text will work but computer audio needs espeak-ng (sudo apt install espeak-ng)." >&2
fi

echo
echo "Sparky Resume + Agentic Tour"
echo "  session:       $SESSION_NAME"
echo "  Resume/Nav2:   single existing authority"
echo "  semantic RViz: /semantic_nav_rviz2"
echo "  phone URL:     http://$LAN_IP:$PHONE_PORT/"
echo "  VLM:           $VLM_PROVIDER / $VLM_MODEL"
echo "  object memory: $ENABLE_OBJECT_PERCEPTION"
echo "  SAM2 masks:    $ENABLE_SAM2 (default OFF for navigation latency)"
echo "  object overlay:$ENABLE_OBJECT_OVERLAY (default OFF; YOLO detections still run)"
echo "  rich memory:   $ENABLE_RICH_MEMORY (off by default until navigation foundation is stable)"
echo "  motion skills: $ENABLE_MOTION_SKILLS (phone Sit/Stand/Wave/Dance are immediate)"
echo "  confirmations: OFF — one agent router executes user requests directly"
echo "  front flip:    $ENABLE_FRONT_FLIP (phone Front Flip is immediate; Nav interlock still applies)"
[ "$SPARKY_WEB_TOKEN" = 0000 ] && echo "  WARNING: using demo dashboard token 0000; change it off a trusted lab LAN."
echo

# Optional launch arguments must not be emitted as bare name:= values.
# ROS 2 rejects an empty CLI launch value; let the launch/VLM defaults handle it.
VLM_BASE_URL_ARGS=()
if [ -n "$VLM_BASE_URL" ]; then
  VLM_BASE_URL_ARGS+=("vlm_base_url:=$VLM_BASE_URL")
fi

ros2 launch go2_omi_voice_bridge resume_agentic_tour_overlay.launch.py \
  session_root:="$SESSION_ROOT" session_name:="$SESSION_NAME" \
  vlm_provider:="$VLM_PROVIDER" vlm_model:="$VLM_MODEL" \
  "${VLM_BASE_URL_ARGS[@]}" \
  enable_rich_memory:="$ENABLE_RICH_MEMORY" \
  enable_object_perception:="$ENABLE_OBJECT_PERCEPTION" enable_sam2:="$ENABLE_SAM2" \
  yolo_imgsz:="$SPARKY_YOLO_IMGSZ" sam2_imgsz:="$SPARKY_SAM2_IMGSZ" sam2_every_n:="$SPARKY_SAM2_EVERY_N" \
  perception_period_sec:="$SPARKY_PERCEPTION_PERIOD_SEC" annotated_publish_period_sec:="$SPARKY_ANNOTATED_PERIOD_SEC" \
  publish_object_overlay:="$ENABLE_OBJECT_OVERLAY" \
  yolo_model:="$YOLO_MODEL" sam2_model:="$SAM2_MODEL" object_device:="$OBJECT_DEVICE" \
  enable_motion_skills:="$ENABLE_MOTION_SKILLS" enable_front_flip:="$ENABLE_FRONT_FLIP" enable_llm_debate:="$ENABLE_LLM_DEBATE" \
  phone_port:="$PHONE_PORT" local_speaker_backend:="$LOCAL_SPEAKER_BACKEND"
