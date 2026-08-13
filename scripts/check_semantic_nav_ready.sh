#!/usr/bin/env bash
set -euo pipefail

if ! ros2 node list 2>/dev/null | grep -qx '/semantic_nav_node'; then
  echo 'ERROR: /semantic_nav_node is NOT running.' >&2
  echo 'Start semantic_nav_tour_world.launch.py first; checkpoint saving needs semantic-nav alive.' >&2
  exit 2
fi

INFO="$(ros2 topic info /semantic_nav/command 2>/dev/null || true)"
if [[ -z "$INFO" ]]; then
  echo 'ERROR: /semantic_nav/command does not exist.' >&2
  exit 3
fi

SUBS="$(printf '%s\n' "$INFO" | awk -F': ' '/Subscription count:/ {print $2}' | tail -1)"
SUBS="${SUBS:-0}"
if ! [[ "$SUBS" =~ ^[0-9]+$ ]] || (( SUBS < 1 )); then
  echo "ERROR: /semantic_nav/command has no subscriber (count=$SUBS)." >&2
  echo 'semantic_nav_node is not connected correctly.' >&2
  exit 4
fi

if ! ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then
  echo 'WARN: /navigate_to_pose is not currently available. Saving can work, but navigation cannot yet.' >&2
fi

echo 'semantic-nav ready'
echo "$INFO"
