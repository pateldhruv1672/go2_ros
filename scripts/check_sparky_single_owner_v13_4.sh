#!/usr/bin/env bash
set -euo pipefail
WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

echo "=== SINGLE SPEECH PATH ==="
ros2 topic info /go2_tts/say --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
echo
ros2 topic info /go2_speech/request --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true

echo
echo "=== SINGLE MOTION OWNER ==="
for t in /cmd_vel_nav2 /cmd_vel_escape /cmd_vel_nav; do
  echo "--- $t"
  ros2 topic info "$t" --verbose 2>/dev/null | grep -E 'Publisher count|Subscription count|Node name:' || true
done

echo
echo "=== PROCESSES ==="
echo "-- motion arbiter"
pgrep -af 'go2_nav_tools/.*/motion_arbiter|/motion_arbiter([[:space:]]|$)' || true
echo "-- TTS"
pgrep -af 'go2_omi_voice_bridge/.*/tts_node|go2_tts_node' || true

echo
echo "Expected after one clean restart:"
echo "  /go2_tts/say: exactly 1 publisher = go2_speech_arbiter"
echo "  /go2_speech/request: unified gate and other speech sources may publish; arbiter subscribes"
echo "  /cmd_vel_nav2: exactly 1 intended subscription = go2_motion_arbiter"
echo "  /cmd_vel_nav: exactly 1 publisher = go2_motion_arbiter"
