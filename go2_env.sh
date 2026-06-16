#!/usr/bin/env bash

conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/digital-twin-admin/Dhruv/sparky/ros2_ws/cyclonedds_go2.xml

export ROBOT_IP=192.168.12.1
export CONN_TYPE=webrtc

source /opt/ros/jazzy/setup.bash
source /home/digital-twin-admin/Dhruv/sparky/ros2_ws/src/.venv/bin/activate
source /home/digital-twin-admin/Dhruv/sparky/ros2_ws/install/setup.bash
