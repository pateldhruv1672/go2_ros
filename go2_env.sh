#!/usr/bin/env bash

# ROS setup files are not safe under set -u / nounset.
set +u

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

if [ -f /home/digital-twin-admin/Dhruv/sparky/ros2_ws/install/setup.bash ]; then
  source /home/digital-twin-admin/Dhruv/sparky/ros2_ws/install/setup.bash
fi

echo "Go2 env loaded"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "  ROBOT_IP=$ROBOT_IP"
echo "  CONN_TYPE=$CONN_TYPE"
echo "  PYTHON=$(which python)"
