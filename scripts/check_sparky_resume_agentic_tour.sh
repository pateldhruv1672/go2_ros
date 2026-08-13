#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$ROOT_DIR/src/.venv/bin/activate" ] && source "$ROOT_DIR/src/.venv/bin/activate"
source "$ROOT_DIR/install/setup.bash"
set -u

MODE="${1:-}"
PASS=0; FAIL=0
check_node(){ if ros2 node list 2>/dev/null | grep -qx "$1"; then echo "PASS node $1"; PASS=$((PASS+1)); else echo "FAIL node $1"; FAIL=$((FAIL+1)); fi; }
check_topic_pub(){ local t="$1"; local n; n="$(ros2 topic info "$t" 2>/dev/null | awk '/Publisher count:/{print $3}')"; if [ "${n:-0}" -ge 1 ]; then echo "PASS publisher $t ($n)"; PASS=$((PASS+1)); else echo "FAIL no publisher $t"; FAIL=$((FAIL+1)); fi; }
check_topic_sub(){ local t="$1"; local n; n="$(ros2 topic info "$t" 2>/dev/null | awk '/Subscription count:/{print $3}')"; if [ "${n:-0}" -ge 1 ]; then echo "PASS subscriber $t ($n)"; PASS=$((PASS+1)); else echo "FAIL no subscriber $t"; FAIL=$((FAIL+1)); fi; }

for n in /go2_driver_node /semantic_nav_node /amcl /controller_server /planner_server /go2_motion_arbiter /go2_langgraph_main_supervisor /go2_unified_intent_gate /go2_vlm_checkpoint_node /go2_speech_arbiter /go2_tts_node /go2_phone_web_gateway /go2_tour_host_script; do check_node "$n"; done
check_topic_sub /go2_voice/transcript
check_topic_sub /go2_agent/query
check_topic_sub /go2_agent/user_command
check_topic_sub /go2_vlm/query
check_topic_pub /go2_vlm/query_result
check_topic_pub /go2_agent/speech
check_topic_pub /go2_tts/status

rviz_count="$(ros2 node list 2>/dev/null | grep -c 'rviz' || true)"
if [ "$rviz_count" -le 1 ]; then echo "PASS RViz node count=$rviz_count"; PASS=$((PASS+1)); else echo "FAIL duplicate RViz nodes=$rviz_count"; ros2 node list | grep rviz; FAIL=$((FAIL+1)); fi

if command -v espeak-ng >/dev/null 2>&1 || command -v espeak >/dev/null 2>&1 || command -v spd-say >/dev/null 2>&1; then
  echo "PASS local TTS executable found"
else
  echo "WARN no espeak-ng/espeak/spd-say executable; audio cannot play until one is installed"
fi

if [ "$MODE" = "--speak-test" ]; then
  echo "Sending SAFE no-motion speaker test..."
  ros2 topic pub --once /go2_speech/request std_msgs/msg/String "{data: '{\"text\":\"Sparky audio test successful.\",\"category\":\"diagnostic\"}'}"
  timeout 8 ros2 topic echo /go2_tts/status --once || true
fi
if [ "$MODE" = "--vlm-test" ]; then
  echo "Sending SAFE no-motion live-camera VLM test..."
  ros2 topic pub --once /go2_vlm/query std_msgs/msg/String "{data: '{\"request_id\":\"manual_vlm_test\",\"question\":\"What do you see in front of you?\",\"source\":\"runtime_check\"}'}"
  timeout 30 ros2 topic echo /go2_vlm/query_result --once || true
fi

echo "Summary: PASS=$PASS FAIL=$FAIL"
[ "$FAIL" -eq 0 ]
