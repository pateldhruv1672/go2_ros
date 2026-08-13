#!/usr/bin/env bash
set -u
printf '\n=== NODES ===\n'
ros2 node list 2>/dev/null | grep -E 'phone_web_gateway|unified_intent_gate|semantic_tour_adapter|tour_host|tts_node|langgraph_main_supervisor' || true
printf '\n=== TRANSCRIPT ===\n'
ros2 topic info /go2_voice/transcript 2>/dev/null || true
printf '\n=== HOST COMMAND ===\n'
ros2 topic info /go2_tour/host_command 2>/dev/null || true
printf '\n=== TTS ===\n'
ros2 topic info /go2_tts/say 2>/dev/null || true
printf '\nHealthy Tour interaction requires transcript subscriptions >= 1 and /go2_tour/host_command + /go2_tts/say to exist.\n'
