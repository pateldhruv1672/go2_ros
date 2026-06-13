#!/usr/bin/env bash
set -euo pipefail

cd ~/Dhruv/sparky/ros2_ws

set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u

export GO2_TARGET=isaac
export ROS_DOMAIN_ID=17
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch go2_isaac_bridge isaac_bridge.launch.py "$@"
