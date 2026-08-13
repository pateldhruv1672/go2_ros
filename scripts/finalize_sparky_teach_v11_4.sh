#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
source src/.venv/bin/activate
source install/setup.bash
[[ -f scripts/sparky_runtime_env.sh ]] && source scripts/sparky_runtime_env.sh
set -u

echo "Writing final grounded pose checkpoint..."
ros2 topic pub --once /go2_memory/write_snapshot_now std_msgs/msg/String "{data: 'teach_finalize'}"

echo "Writing final OpenRouter VLM checkpoint..."
ros2 topic pub --once /go2_vlm_checkpoint/write_now std_msgs/msg/String "{data: 'teach_finalize'}"

echo "Saving map snapshot into the active semantic session..."
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'save_map'}"

sleep 3

echo
echo "Finalization requests sent. Verify saved_map and VLM checkpoint status before Ctrl+C."
echo "Run: bash scripts/check_sparky_teach_v11_4.sh"
