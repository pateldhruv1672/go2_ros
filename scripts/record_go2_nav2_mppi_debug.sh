#!/usr/bin/env bash
set -u
set -o pipefail

DURATION_SEC="${1:-30}"
if ! [[ "$DURATION_SEC" =~ ^[0-9]+$ ]]; then
  echo "Usage: $0 [duration_seconds]" >&2
  exit 2
fi

WORKSPACE="/home/digital-twin-admin/Dhruv/sparky/ros2_ws"
cd "$WORKSPACE" || exit 1

if [ -f "./go2_env.sh" ]; then
  # shellcheck disable=SC1091
  source ./go2_env.sh
fi

if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available. Source the ROS environment first." >&2
  exit 1
fi

if [ -s /tmp/go2_nav2_mppi_latest_log_dir.txt ]; then
  LOG_DIR="$(cat /tmp/go2_nav2_mppi_latest_log_dir.txt)"
else
  TS="$(date +%Y%m%d_%H%M%S)"
  LOG_DIR="$HOME/go2_nav2_mppi_fix_logs/$TS"
  mkdir -p "$LOG_DIR"
  printf '%s\n' "$LOG_DIR" > /tmp/go2_nav2_mppi_latest_log_dir.txt
fi

mkdir -p "$LOG_DIR/topics" "$LOG_DIR/bags"

echo "$LOG_DIR" | tee "$LOG_DIR/log_dir.txt"
date --iso-8601=seconds | tee "$LOG_DIR/record_started_at.txt"
ros2 topic list | sort | tee "$LOG_DIR/topics/topic_list.txt"
ros2 node list | sort > "$LOG_DIR/topics/node_list.txt" 2>&1 || true
ros2 node list | sort | uniq -c | sort -nr > "$LOG_DIR/topics/node_duplicate_summary.txt" 2>&1 || true
ros2 action list -t | sort > "$LOG_DIR/topics/action_list.txt" 2>&1 || true
ros2 action info /navigate_to_pose > "$LOG_DIR/topics/action_info_navigate_to_pose.txt" 2>&1 || true

for node in /controller_server /planner_server /bt_navigator /collision_monitor; do
  safe_name="${node#/}"
  ros2 lifecycle get "$node" > "$LOG_DIR/topics/lifecycle_${safe_name}.txt" 2>&1 || true
done

declare -a PARAM_CHECKS=(
  "/controller_server FollowPath.plugin param_followpath_plugin.txt"
  "/controller_server FollowPath.motion_model param_followpath_motion_model.txt"
  "/planner_server GridBased.plugin param_planner_plugin.txt"
  "/bt_navigator default_nav_to_pose_bt_xml param_bt_xml.txt"
)

for check in "${PARAM_CHECKS[@]}"; do
  set -- $check
  ros2 param get "$1" "$2" > "$LOG_DIR/topics/$3" 2>&1 || true
done

topic_exists() {
  grep -Fxq "$1" "$LOG_DIR/topics/topic_list.txt"
}

record_echo() {
  local topic="$1"
  local output="$2"
  if topic_exists "$topic"; then
    timeout "$DURATION_SEC" ros2 topic echo "$topic" > "$LOG_DIR/topics/$output" 2>&1 &
    PIDS+=("$!")
  else
    echo "Topic $topic not present" > "$LOG_DIR/topics/$output"
  fi
}

record_hz() {
  local topic="$1"
  local output="$2"
  if topic_exists "$topic"; then
    timeout "$DURATION_SEC" ros2 topic hz "$topic" > "$LOG_DIR/topics/$output" 2>&1 &
    PIDS+=("$!")
  else
    echo "Topic $topic not present" > "$LOG_DIR/topics/$output"
  fi
}

PIDS=()

record_echo /cmd_vel_nav cmd_vel_nav_stream.yaml
record_echo /cmd_vel_out cmd_vel_out_stream.yaml
record_echo /cmd_vel_sdk cmd_vel_sdk_stream.yaml
record_echo /odom odom_stream.yaml
record_echo /go2_states go2_states_stream.yaml
record_echo /collision_monitor_state collision_monitor_state_stream.yaml
record_echo /goal_pose goal_pose_stream.yaml
record_hz /cmd_vel_nav cmd_vel_nav_hz.txt
record_hz /cmd_vel_out cmd_vel_out_hz.txt
record_hz /cmd_vel_sdk cmd_vel_sdk_hz.txt
record_hz /odom odom_hz.txt
record_hz /go2_states go2_states_hz.txt
record_hz /point_cloud2 point_cloud2_hz.txt
record_hz /point_cloud2_nav point_cloud2_nav_hz.txt
record_hz /scan scan_hz.txt
record_hz /scan_nav scan_nav_hz.txt

declare -a BAG_TOPICS=()
for topic in /tf /tf_static /odom /point_cloud2 /point_cloud2_nav /scan /scan_nav /map /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk /collision_monitor_state /plan /local_plan /go2_states /goal_pose /behavior_tree_log /transformed_global_plan /trajectories /optimal_trajectory; do
  if topic_exists "$topic"; then
    BAG_TOPICS+=("$topic")
  fi
done

if [ "${#BAG_TOPICS[@]}" -gt 0 ]; then
  timeout "$DURATION_SEC" ros2 bag record \
    -o "$LOG_DIR/bags/nav2_mppi_goal_test" \
    "${BAG_TOPICS[@]}" > "$LOG_DIR/rosbag_record.log" 2>&1 &
  PIDS+=("$!")
else
  echo "No bag topics were present." > "$LOG_DIR/rosbag_record.log"
fi

for pid in "${PIDS[@]}"; do
  wait "$pid" || true
done

python - "$LOG_DIR/bags/nav2_mppi_goal_test/metadata.yaml" > "$LOG_DIR/topics/rosbag_message_counts.txt" 2>&1 <<'PY' || true
from pathlib import Path
import sys
import yaml

metadata_path = Path(sys.argv[1])
if not metadata_path.exists():
    print(f"Missing rosbag metadata: {metadata_path}")
    raise SystemExit(0)

data = yaml.safe_load(metadata_path.read_text())
info = data.get("rosbag2_bagfile_information", {})
for item in info.get("topics_with_message_count", []):
    topic = item.get("topic_metadata", {}).get("name", "")
    count = item.get("message_count", 0)
    if topic in {
        "/cmd_vel_nav",
        "/cmd_vel_out",
        "/cmd_vel_sdk",
        "/go2_states",
        "/odom",
        "/point_cloud2",
        "/point_cloud2_nav",
        "/scan_nav",
        "/plan",
        "/goal_pose",
        "/behavior_tree_log",
        "/collision_monitor_state",
    }:
        print(f"{topic}: {count}")
PY

cat > "$LOG_DIR/velocity_debug_README.txt" <<'EOF'
Primary evidence:
- cmd_vel_nav_stream.yaml: raw velocity commands produced by Nav2 controller before collision monitor.
- cmd_vel_out_stream.yaml: velocity commands after collision monitor, sent to the Go2 driver.
- cmd_vel_sdk_stream.yaml: rounded/scaled command values sent into the Go2 sport Move API.
- go2_states_stream.yaml: robot-reported sport mode state, including velocity.
- odom_stream.yaml: robot-reported odometry pose stream for measuring actual displacement.
- action_info_navigate_to_pose.txt: confirms whether the Nav2 action server and RViz clients are present.
- rosbag_message_counts.txt: quick check for whether a goal produced plan and velocity messages during this recording window.
- bags/nav2_mppi_goal_test: rosbag with available command, odometry, state, TF, scan, map, and plan topics.

Interpretation:
- Good /cmd_vel_nav but tiny /cmd_vel_out points at collision monitor or stale scan.
- Good /cmd_vel_out but poor /go2_states velocity or odom displacement points at driver, transport, gait, or robot mode.
- Good /cmd_vel_sdk but poor /go2_states velocity or odom displacement points below the ROS graph, at the robot command path or robot state.
- Angular-heavy /cmd_vel_nav with little linear.x points back at MPPI/planner tuning.
EOF

date --iso-8601=seconds | tee "$LOG_DIR/record_finished_at.txt"
echo "Saved Nav2 MPPI debug logs to: $LOG_DIR"
