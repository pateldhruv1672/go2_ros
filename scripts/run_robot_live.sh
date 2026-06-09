#!/usr/bin/env bash
set -euo pipefail

pkill -f "go2_driver_node|robot_state_publisher|slam_toolbox|amcl|map_server|rviz2|semantic_nav|bt_navigator|planner_server|controller_server|lifecycle_manager|foxglove_bridge|local_omi_stt_node|dialogue_orchestrator_node|camera_agent_node|speaker_tts_node|pointcloud_aggregator|pointcloud_to_laserscan_node|go2_pointcloud_to_laserscan|lidar_to_pointcloud|scan_retimestamp_node" || true
sleep 3

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/sparky_ros_env.sh"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

BASE_MODE="${BASE_MODE:-teach}"
case "$BASE_MODE" in
  teach)
    SLAM="${SLAM:-true}"
    NAV2="${NAV2:-false}"
    RVIZ2="${RVIZ2:-false}"
    ;;
  resume)
    SLAM="${SLAM:-false}"
    NAV2="${NAV2:-false}"
    RVIZ2="${RVIZ2:-false}"
    ;;
  *)
    echo "BASE_MODE must be 'teach' or 'resume' (got: $BASE_MODE)" >&2
    exit 2
    ;;
esac
FOXGLOVE="${FOXGLOVE:-false}"

ros2 daemon stop || true
ros2 daemon start || true

exec ros2 launch go2_robot_sdk robot.launch.py \
  foxglove:="$FOXGLOVE" \
  slam:="$SLAM" \
  nav2:="$NAV2" \
  rviz2:="$RVIZ2"
