#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$ROOT_DIR/.env.local" ]; then
  set -a
  . "$ROOT_DIR/.env.local"
  set +a
fi

set +u
if [ -f "$ROOT_DIR/src/.venv/bin/activate" ]; then
  source "$ROOT_DIR/src/.venv/bin/activate"
elif [ -f "$ROOT_DIR/.venv/bin/activate" ]; then
  source "$ROOT_DIR/.venv/bin/activate"
fi
source "$SCRIPT_DIR/sparky_ros_env.sh"
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"

# Keep the base driver alive, but clear stale resume, Nav2, voice, and agent overlays.
pkill -f "semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|semantic_nav_rviz2|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking|go2_omi_bridge|go2_voice_stt_node|go2_voice_intent_gate|go2_tts_node|go2_tour_voice_command_router|go2_langgraph_main_supervisor|go2_vlm_checkpoint_node|go2_lidar_geometry_node|go2_pointcloud_analyzer_node|go2_traversability_node|go2_dynamic_obstacle_tracker|go2_open_vocab_detector" || true
sleep 2

ros2 daemon stop || true
ros2 daemon start || true

BASE_READY=0
if ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$"; then
  BASE_READY=1
fi

if [ "$BASE_READY" -eq 0 ]; then
  echo "[run_sparky_voice_resume] base bringup not detected; starting robot base in the background"
  BASE_LOG=/tmp/go2_base_bringup.log
  nohup ros2 launch go2_robot_sdk robot.launch.py foxglove:=false slam:=false nav2:=false rviz2:=false >"$BASE_LOG" 2>&1 </dev/null &
  for _ in $(seq 1 45); do
    if ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$" && \
       ros2 topic list 2>/dev/null | grep -q "^/odom$" && \
       ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
      echo "[run_sparky_voice_resume] base bringup is ready"
      break
    fi
    sleep 1
  done
  if ! ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$"; then
    echo "[run_sparky_voice_resume] base bringup did not become ready; see $BASE_LOG" >&2
    echo "[run_sparky_voice_resume] continuing anyway, but Nav2/voice context may fail until the base stack is up" >&2
  fi
fi

exec ros2 launch go2_omi_voice_bridge sparky_voice_resume.launch.py \
  session_root:="${SESSION_ROOT:-~/.ros/go2_semantic_nav_sessions}" \
  session_name:="${SESSION_NAME:-auto}" \
  rviz2:="${RVIZ2:-true}" \
  restore_spawn_on_start:="${RESTORE_SPAWN_ON_START:-true}" \
  adapter_mode:="${OMI_ADAPTER_MODE:-ble_audio}" \
  ble_device_address:="${OMI_BLE_DEVICE_ADDRESS:-EF:1C:34:C6:25:92}" \
  require_confirmation_for_motion:="${REQUIRE_CONFIRMATION_FOR_MOTION:-true}" \
  enable_llm_debate:="${ENABLE_LLM_DEBATE:-true}" \
  debate_llm_provider:="${DEBATE_LLM_PROVIDER:-ollama}" \
  debate_llm_model:="${DEBATE_LLM_MODEL:-gemma4:12b}" \
  debate_llm_timeout_sec:="${DEBATE_LLM_TIMEOUT_SEC:-30.0}" \
  enable_vlm_checkpointing:="${ENABLE_VLM_CHECKPOINTING:-true}" \
  vlm_provider:="${VLM_PROVIDER:-ollama}" \
  vlm_model:="${VLM_MODEL:-gemma4:12b}" \
  vlm_auto_write_checkpoints:="${VLM_AUTO_WRITE_CHECKPOINTS:-false}" \
  tts_enabled:="${TTS_ENABLED:-true}" \
  local_speaker_enabled:="${LOCAL_SPEAKER_ENABLED:-true}" \
  enable_perception_tools:="${ENABLE_PERCEPTION_TOOLS:-true}" \
  enable_dynamic_obstacle_tracking:="${ENABLE_DYNAMIC_OBSTACLE_TRACKING:-false}" \
  enable_open_vocab_detector:="${ENABLE_OPEN_VOCAB_DETECTOR:-false}"
