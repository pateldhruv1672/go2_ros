#!/usr/bin/env bash
set -euo pipefail
PAYLOAD='{"type":"reorder_tour_stops","order":["welcome_checkpoint","humanoid_checkpoint","roboarms_checkpoint"]}'
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: '$PAYLOAD'}"
echo "Tour order set: welcome -> humanoid -> roboarms"
