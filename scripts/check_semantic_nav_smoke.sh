#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

timeout 5s ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'status'}" >/tmp/semantic_nav_pub.log 2>&1 || true
timeout 5s ros2 topic echo --once /semantic_nav/status || true
echo "---"
timeout 5s ros2 topic echo --once /semantic_nav/event || true
