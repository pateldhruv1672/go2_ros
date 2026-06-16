#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-go2_isaac_ros}"
WS="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is missing. Install with:"
  echo "  sudo apt update && sudo apt install -y tmux"
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

ROS_SETUP='
cd /home/digital-twin-admin/Dhruv/sparky/ros2_ws
set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
set -u
export ROS_DOMAIN_ID=17
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GO2_TARGET=isaac
'

tmux new-session -d -s "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" "$ROS_SETUP
scripts/run_isaac_bridge_ros.sh" C-m

tmux new-window -t "$SESSION" -n robot_model
tmux send-keys -t "$SESSION:robot_model" "$ROS_SETUP
scripts/run_isaac_robot_description.sh" C-m

tmux new-window -t "$SESSION" -n sensor_tf
tmux send-keys -t "$SESSION:sensor_tf" "$ROS_SETUP
scripts/run_isaac_sensor_static_tfs.sh" C-m

tmux new-window -t "$SESSION" -n monitor
tmux send-keys -t "$SESSION:monitor" "$ROS_SETUP
watch -n 1 '
echo === topics ===
ros2 topic list -t | grep -E \"clock|odom|joint|tf|scan|point|camera|map|plan\" || true
echo
echo === scan publisher ===
ros2 topic info /scan -v 2>/dev/null | grep -E \"Node name|Endpoint type|Reliability|Publisher count\" || true
echo
echo === pointcloud publisher ===
ros2 topic info /point_cloud2 -v 2>/dev/null | grep -E \"Node name|Endpoint type|Reliability|Publisher count\" || true
echo
echo === camera publisher ===
ros2 topic info /camera/image_raw -v 2>/dev/null | grep -E \"Node name|Endpoint type|Reliability|Publisher count\" || true
'" C-m

tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" "$ROS_SETUP
sleep 5
RVIZ_FIXED_FRAME=odom scripts/run_isaac_full_rviz.sh" C-m

echo "Started tmux session: $SESSION"
echo
echo "Attach with:"
echo "  tmux attach -t $SESSION"
echo
echo "Windows:"
echo "  bridge       - UDP bridge"
echo "  robot_model  - robot_state_publisher"
echo "  sensor_tf    - base_link -> base_scan TF"
echo "  monitor      - live topic contract"
echo "  rviz         - RViz"
