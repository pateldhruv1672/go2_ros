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

# Only stop the semantic resume overlay and resume-owned Nav2 nodes.
# Do not kill the base robot bringup, driver, state publisher, or teleop stack here.
pkill -f "semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|semantic_nav_rviz2|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking" || true
sleep 2

ros2 daemon stop || true
ros2 daemon start || true

BASE_READY=0
if ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$"; then
  BASE_READY=1
fi

if [ "$BASE_READY" -eq 0 ]; then
  echo "[run_semantic_nav_resume] base bringup not detected; starting BASE_MODE=base in the background"
  BASE_LOG=/tmp/go2_base_bringup.log
  nohup ros2 launch go2_robot_sdk robot.launch.py foxglove:=false slam:=false nav2:=false rviz2:=false >"$BASE_LOG" 2>&1 </dev/null &
  for _ in $(seq 1 45); do
    if ros2 node list 2>/dev/null | grep -q "^/go2_driver_node$" && \
       ros2 topic list 2>/dev/null | grep -q "^/odom$" && \
       ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
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

exec ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py rviz2:=true restore_spawn_on_start:="${RESTORE_SPAWN_ON_START:-true}"
