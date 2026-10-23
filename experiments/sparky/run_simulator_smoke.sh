#!/usr/bin/env bash
# This is only a real Gazebo/ROS readiness test, not the requested Nav2 benchmark.
set -eo pipefail
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim_workspace=${1:?Usage: run_simulator_smoke.sh /absolute/sim_workspace}
output_dir=${2:-"$task_dir/logs/simulator-smoke"}
mkdir -p "$output_dir"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$sim_workspace/install/setup.bash" ]]; then
  echo 'BLOCKED: ROS Jazzy or built simulator workspace missing.' | tee "$output_dir/blocker.log"
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$sim_workspace/install/setup.bash"
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=offscreen
export GZ_IP=127.0.0.1
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false gui:=false \
  disable_camera:=true disable_d455:=true disable_lidar_l1:=true disable_velodyne_lidar:=true \
  > "$output_dir/launch.log" 2>&1 &
launch_pid=$!
ros2 bag record -a -o "$output_dir/rosbag" > "$output_dir/rosbag.log" 2>&1 &
bag_pid=$!
cleanup() {
  kill -INT "$bag_pid" 2>/dev/null || true
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
/usr/bin/python3 "$task_dir/smoke_observer.py" "$output_dir"
