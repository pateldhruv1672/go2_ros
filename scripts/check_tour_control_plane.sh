#!/usr/bin/env bash
set -u

ok=1
check_node() {
  local n="$1"
  if ros2 node list 2>/dev/null | grep -qx "$n"; then
    echo "OK   node $n"
  else
    echo "FAIL node $n"
    ok=0
  fi
}
check_topic_sub() {
  local t="$1" min="${2:-1}"
  local info sub
  info="$(ros2 topic info "$t" 2>/dev/null || true)"
  sub="$(printf '%s\n' "$info" | awk -F': ' '/Subscription count:/ {print $2}' | tail -1)"
  sub="${sub:-0}"
  if [[ "$sub" =~ ^[0-9]+$ ]] && (( sub >= min )); then
    echo "OK   topic $t subscriptions=$sub"
  else
    echo "FAIL topic $t subscriptions=$sub"
    ok=0
  fi
}
check_action() {
  local a="$1"
  if ros2 action list 2>/dev/null | grep -qx "$a"; then
    echo "OK   action $a"
  else
    echo "FAIL action $a"
    ok=0
  fi
}

echo '=== CORE CONTROL PLANE ==='
check_node /semantic_nav_node
check_topic_sub /semantic_nav/command 1
check_action /navigate_to_pose
check_action /compute_path_to_pose

echo
echo '=== TOUR / VOICE ==='
for n in /go2_agentic_voice_action /go2_phone_web_gateway /go2_tts_node; do
  if ros2 node list 2>/dev/null | grep -qx "$n"; then echo "OK   node $n"; else echo "WARN node $n missing"; fi
done
check_topic_sub /go2_tts/say 1

echo
echo '=== RVIZ ==='
if ros2 node list 2>/dev/null | grep -Eq '^/(sparky_tour_rviz|semantic_nav_rviz2|rviz2)$'; then
  ros2 node list 2>/dev/null | grep -E '^/(sparky_tour_rviz|semantic_nav_rviz2|rviz2)$' | sed 's/^/OK   node /'
else
  echo 'FAIL no RViz node'
  ok=0
fi

echo
echo '=== OBJECT MEMORY ==='
ros2 topic echo /go2_memory/object_inventory --once --full-length 2>/dev/null | head -80 || echo 'WARN object inventory unavailable'

echo
echo '=== COLLISION SOURCE ==='
ros2 param get /collision_monitor observation_sources 2>/dev/null || echo 'WARN collision_monitor unavailable'
ros2 param get /collision_monitor pointcloud.topic 2>/dev/null || true
ros2 param get /collision_monitor pointcloud.source_timeout 2>/dev/null || true

echo
if (( ok == 1 )); then
  echo 'CONTROL_PLANE_READY=1'
  exit 0
else
  echo 'CONTROL_PLANE_READY=0'
  exit 1
fi
