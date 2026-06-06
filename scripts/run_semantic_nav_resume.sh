#!/usr/bin/env bash
set -euo pipefail

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

exec ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py rviz2:=true
