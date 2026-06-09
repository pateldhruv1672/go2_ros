#!/usr/bin/env bash
set -euo pipefail

export ROS_HOME=/tmp/ros_home_sparky
export ROS_LOG_DIR=/tmp/ros_logs_sparky
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

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

# Only stop the semantic resume overlay and resume-owned Nav2 nodes.
# Do not kill the base robot bringup, driver, state publisher, or teleop stack here.
pkill -f "semantic_nav_node|scan_retimestamp_node|resume_map_server|resume_map_lifecycle_manager|semantic_nav_rviz2|controller_server|planner_server|bt_navigator|waypoint_follower|collision_monitor|lifecycle_manager_navigation|behavior_server|opennav_docking" || true
sleep 2

exec ros2 launch go2_semantic_nav_agent semantic_nav_resume.launch.py rviz2:=true
