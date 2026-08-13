#!/usr/bin/env bash
set -u

echo '=== OLLAMA ==='
curl -fsS --max-time 3 http://127.0.0.1:11434/api/tags 2>/dev/null | head -c 800 || echo 'OLLAMA UNREACHABLE'
echo; echo

echo '=== AGENTIC NODES ==='
ros2 node list 2>/dev/null | grep -E 'agentic_voice_action|speech_arbiter|phone_web_gateway|tour_host|tts_node|semantic_nav_node|safe_webrtc' || true

echo; echo '=== TRANSCRIPT CONTRACT ==='
ros2 topic info /go2_voice/transcript 2>/dev/null || true

echo; echo '=== SPEECH CONTRACT ==='
ros2 topic info /go2_speech/request 2>/dev/null || true
ros2 topic info /go2_tts/say 2>/dev/null || true

echo; echo '=== AGENT STATE ==='
ros2 topic echo /go2_agent/interaction_state --once 2>/dev/null || true

echo; echo '=== OBJECT PIPELINE ==='
ros2 topic info /object_explorer/sam2_detections 2>/dev/null || true
ros2 topic info /go2_vln/target_detections_3d 2>/dev/null || true
ros2 topic info /go2_vln/object_map 2>/dev/null || true
