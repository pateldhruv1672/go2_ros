#!/usr/bin/env bash
set -eo pipefail
DURATION="${1:-30}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="${SPARKY_DEBUG_DIR:-$HOME/sparky_nav_debug}"
OUT="$BASE/trajectory_$STAMP"
mkdir -p "$OUT"

echo "SPARKY V11.7 TRAJECTORY FLIGHT RECORDER"
echo "output=$OUT duration=${DURATION}s"
echo "READ ONLY: send ONE 0.5-1.0 m RViz goal during the capture."

ros2 node list > "$OUT/nodes.txt" 2>&1 || true
ros2 topic list -t > "$OUT/topics.txt" 2>&1 || true
for node in /controller_server /planner_server /local_costmap/local_costmap /global_costmap/global_costmap /amcl /go2_driver_node /go2_motion_arbiter /collision_monitor; do
  safe="$(echo "$node" | tr '/' '_' | sed 's/^_//')"
  ros2 param dump "$node" > "$OUT/params_${safe}.yaml" 2>&1 || true
done
for topic in /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk /evaluation /local_plan /transformed_global_plan; do
  safe="$(echo "$topic" | tr '/' '_' | sed 's/^_//')"
  ros2 topic info "$topic" --verbose > "$OUT/topic_${safe}.txt" 2>&1 || true
done

timeout 3 ros2 run tf2_ros tf2_echo map base_link > "$OUT/tf_map_base_start.txt" 2>&1 || true
timeout 3 ros2 run tf2_ros tf2_echo odom base_link > "$OUT/tf_odom_base_start.txt" 2>&1 || true

AVAILABLE="$(ros2 topic list 2>/dev/null || true)"
CANDIDATES=(/tf /tf_static /map /scan /scan_nav /odom /amcl_pose /plan /transformed_global_plan /local_plan /evaluation /marker /cost_cloud /local_costmap/costmap /local_costmap/published_footprint /global_costmap/costmap /global_costmap/published_footprint /cmd_vel_nav2 /cmd_vel_nav /cmd_vel_out /cmd_vel_sdk /navigate_to_pose/_action/status /follow_path/_action/status)
TOPICS=()
for t in "${CANDIDATES[@]}"; do grep -qxF "$t" <<<"$AVAILABLE" && TOPICS+=("$t"); done
printf '%s\n' "${TOPICS[@]}" > "$OUT/recorded_topics.txt"

python3 "$ROOT/scripts/sparky_trajectory_trace_v11_7.py" --outdir "$OUT" > "$OUT/trace_node.log" 2>&1 & TRACE_PID=$!
ros2 bag record -o "$OUT/bag" --topics "${TOPICS[@]}" > "$OUT/rosbag.log" 2>&1 & BAG_PID=$!

finish(){ kill -INT "$BAG_PID" 2>/dev/null || true; kill -INT "$TRACE_PID" 2>/dev/null || true; wait "$BAG_PID" 2>/dev/null || true; wait "$TRACE_PID" 2>/dev/null || true; }
trap finish INT TERM EXIT
sleep "$DURATION"
finish
trap - INT TERM EXIT

timeout 3 ros2 run tf2_ros tf2_echo map base_link > "$OUT/tf_map_base_end.txt" 2>&1 || true
timeout 3 ros2 run tf2_ros tf2_echo odom base_link > "$OUT/tf_odom_base_end.txt" 2>&1 || true
ros2 bag info "$OUT/bag" > "$OUT/bag_info.txt" 2>&1 || true
python3 "$ROOT/scripts/analyze_sparky_trajectory_v11_7.py" "$OUT" | tee "$OUT/SUMMARY.console.txt"
tar -C "$BASE" -czf "$OUT.tar.gz" "$(basename "$OUT")" --exclude='*/bag/*' 2>/dev/null || true

echo "Full bag: $OUT/bag"
echo "Compact bundle: $OUT.tar.gz"
echo "Summary: $OUT/SUMMARY.txt"
