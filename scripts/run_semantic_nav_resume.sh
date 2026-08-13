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

source "$SCRIPT_DIR/sparky_ros_env.sh"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export ROBOT_IP="${ROBOT_IP:-192.168.12.1}"
export CONN_TYPE="${CONN_TYPE:-webrtc}"
export GO2_RESUME_INTERNAL_RVIZ="${GO2_RESUME_INTERNAL_RVIZ:-1}"

# Only stop the semantic resume overlay and resume-owned Nav2 nodes.
# Do not kill a healthy base robot bringup, driver, state publisher, or teleop stack here.

# SPARKY_KILL_STALE_ARBITER_V13_4
# Do not allow a motion arbiter from a previous Resume invocation to remain
# subscribed to /cmd_vel_nav2 or publishing /cmd_vel_nav.
pkill -f 'go2_nav_tools/.*/motion_arbiter|/motion_arbiter([[:space:]]|$)' 2>/dev/null || true
sleep 0.5
# Kill the parent launch first so old generated /tmp/launch_params_* files cannot
# keep stale Nav2 parameters alive after a source/config edit.
pkill -f "ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py" || true
# SPARKY_SINGLE_MOTION_ARBITER_V13_1
# A Resume launch owns exactly one go2_motion_arbiter. Kill stale copies before
# the new launch creates its command owner.
pkill -f "go2_motion_arbiter|go2_nav_tools/.*/motion_arbiter" || true
sleep 1
# V12_8_3_SINGLE_MOTION_ARBITER_OWNER
# semantic_nav_resume.launch.py owns exactly one go2_motion_arbiter. Kill stale
# arbiters before restarting Resume so independent 20 Hz publishers cannot race
# on /cmd_vel_nav after repeated launch/stop cycles.
pkill -f "go2_motion_arbiter|go2_nav_tools/.*/motion_arbiter" || true
sleep 1
if pgrep -af "go2_motion_arbiter|go2_nav_tools/.*/motion_arbiter" >/dev/null 2>&1; then
  echo "[run_semantic_nav_resume] stale go2_motion_arbiter survived cleanup; refusing duplicate command ownership" >&2
  pgrep -af "go2_motion_arbiter|go2_nav_tools/.*/motion_arbiter" >&2 || true
  exit 1
fi
pkill -f "semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|semantic_nav_rviz2|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking|go2_motion_arbiter" || true
sleep 2

ros2 daemon stop || true
ros2 daemon start || true

base_ready() {
  ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$" && \
  ros2 topic list 2>/dev/null | grep -q "^/odom$" && \
  ros2 topic list 2>/dev/null | grep -q "^/point_cloud2$" && \
  ros2 topic list 2>/dev/null | grep -q "^/scan$"
}

BASE_READY=0
if base_ready; then
  BASE_READY=1
fi

if [ "$BASE_READY" -eq 0 ]; then
  echo "[run_semantic_nav_resume] base bringup is missing driver/odom/point_cloud2/scan; clearing stale base launch"
  pkill -f "ros2 launch go2_robot_sdk robot.launch.py" || true
  pkill -f "go2_driver_node|lidar_to_pointcloud|pointcloud_aggregator|go2_pointcloud_to_laserscan|go2_robot_state_publisher|tts_node|joy_node|go2_teleop_node|twist_mux" || true
  pkill -f "speech_processor/lib/speech_processor/tts_node" || true
  pkill -f "/joy/joy_node" || true
  pkill -f "teleop_twist_joy/teleop_node" || true
  pkill -f "/twist_mux" || true
  sleep 2
  echo "[run_semantic_nav_resume] base bringup not detected; starting BASE_MODE=base in the background"
  BASE_LOG=/tmp/go2_base_bringup.log
  nohup ros2 launch go2_robot_sdk robot.launch.py foxglove:=false slam:=false nav2:=false rviz2:=false >"$BASE_LOG" 2>&1 </dev/null &
  for _ in $(seq 1 45); do
    if base_ready; then
      echo "[run_semantic_nav_resume] base bringup is ready"
      break
    fi
    sleep 1
  done
  if ! ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$"; then
    echo "[run_semantic_nav_resume] base bringup did not become ready; see $BASE_LOG" >&2
    echo "[run_semantic_nav_resume] continuing anyway, but Nav2 may fail until the base stack is up" >&2
  fi
fi

exec ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py \
  session_name:="${SESSION_NAME:-}" \
  rviz2:="${RVIZ2:-true}" \
  restore_spawn_on_start:="${RESTORE_SPAWN_ON_START:-true}" \
  nav2_start_delay_sec:="${NAV2_START_DELAY_SEC:-6.0}" \
  scan_input_topic:="${SCAN_INPUT_TOPIC:-/scan}" \
  scan_nav_topic:="${SCAN_NAV_TOPIC:-/scan_nav}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/point_cloud2}" \
  stvl_enabled:="${STVL_ENABLED:-auto}"
