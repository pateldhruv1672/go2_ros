#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-go2_isaac_base}"
WS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux missing. Install with:"
  echo "  sudo apt update && sudo apt install -y tmux"
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

ROS_SETUP='cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u
export ROS_DOMAIN_ID=17
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_TARGET=isaac'

tmux new-session -d -s "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:bridge" "scripts/run_isaac_bridge_ros.sh" C-m

tmux new-window -t "$SESSION" -n robot_model
tmux send-keys -t "$SESSION:robot_model" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:robot_model" "scripts/run_isaac_robot_description.sh" C-m

tmux new-window -t "$SESSION" -n sensor_tf
tmux send-keys -t "$SESSION:sensor_tf" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:sensor_tf" "scripts/run_isaac_sensor_static_tfs.sh" C-m

tmux new-window -t "$SESSION" -n cmd_relay
tmux send-keys -t "$SESSION:cmd_relay" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:cmd_relay" "python3 scripts/cmd_vel_relay.py" C-m

tmux new-window -t "$SESSION" -n slam
tmux send-keys -t "$SESSION:slam" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:slam" "sleep 4; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true" C-m

tmux new-window -t "$SESSION" -n nav2
tmux send-keys -t "$SESSION:nav2" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:nav2" 'sleep 8
NAV2_PARAMS=""
if [ -f "/home/digital-twin-admin/Dhruv/sparky/ros2_ws/config/isaac_nav2_params.yaml" ]; then
  NAV2_PARAMS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws/config/isaac_nav2_params.yaml"
elif [ -f "/home/digital-twin-admin/Dhruv/sparky/ros2_ws/install/go2_semantic_nav_agent/share/go2_semantic_nav_agent/config/nav2_params.yaml" ]; then
  NAV2_PARAMS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws/install/go2_semantic_nav_agent/share/go2_semantic_nav_agent/config/nav2_params.yaml"
elif [ -f "/home/digital-twin-admin/Dhruv/sparky/ros2_ws/src/go2_semantic_nav_agent/config/nav2_params.yaml" ]; then
  NAV2_PARAMS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws/src/go2_semantic_nav_agent/config/nav2_params.yaml"
else
  NAV2_PARAMS="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml"
fi
echo "[base_nav2] Using params: $NAV2_PARAMS"
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:="$NAV2_PARAMS"' C-m

tmux new-window -t "$SESSION" -n monitor
tmux send-keys -t "$SESSION:monitor" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:monitor" 'watch -n 1 "echo === TOPICS ===; ros2 topic list -t | grep -E \"clock|odom|tf|scan|map|plan|cmd_vel|cmd_vel_out|goal_pose\" || true; echo; echo === SCAN ===; ros2 topic info /scan 2>/dev/null || true; echo; echo === MAP ===; ros2 topic info /map 2>/dev/null || true; echo; echo === TF map odom ===; timeout 1 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -8 || true"' C-m

tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:rviz" "sleep 12; RVIZ_FIXED_FRAME=map scripts/run_isaac_full_rviz.sh" C-m

echo "Started base Isaac SLAM/Nav2 tmux session: $SESSION"
echo
echo "Attach:"
echo "  tmux attach -t $SESSION"
echo
echo "Detach without killing:"
echo "  Ctrl+B then D"
