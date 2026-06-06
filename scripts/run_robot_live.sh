#!/usr/bin/env bash
set -euo pipefail

pkill -f "go2_driver_node|robot_state_publisher|slam_toolbox|amcl|map_server|rviz2|semantic_nav|bt_navigator|planner_server|controller_server|lifecycle_manager|foxglove_bridge|local_omi_stt_node|dialogue_orchestrator_node|camera_agent_node|speaker_tts_node" || true
sleep 3

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

ros2 daemon stop || true
ros2 daemon start || true

exec ros2 launch go2_robot_sdk robot.launch.py
