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

echo "NODES"
ros2 node list
echo "---"
echo "TOPICS"
ros2 topic list
echo "---"
echo "GOAL"
ros2 topic echo --once /goal_pose || true
