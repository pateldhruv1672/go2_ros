#!/usr/bin/env bash
set -euo pipefail

WS="${SPARKY_WS:-$HOME/Dhruv/sparky/ros2_ws}"
cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f src/.venv/bin/activate ]] && source src/.venv/bin/activate
source install/setup.bash
set -u

MODE="${1:-status}"

echo '=== OpenRouter front-door ==='
ros2 param get /go2_unified_intent_gate frontdoor_enabled || true
ros2 param get /go2_unified_intent_gate frontdoor_model || true
ros2 param get /go2_unified_intent_gate frontdoor_timeout_sec || true
printf 'OPENROUTER_API_KEY: '
[[ -n "${OPENROUTER_API_KEY:-}" ]] && echo SET || echo MISSING
printf 'SPARKY_FRONTDOOR_MODEL: %s\n' "${SPARKY_FRONTDOOR_MODEL:-<default google/gemini-2.5-flash>}"

echo
echo '=== agent path ==='
for t in /go2_voice/transcript /go2_agent/user_command /go2_agent/query /go2_vlm/query /go2_agent/speech /go2_tts/say; do
  echo "--- $t"
  ros2 topic info "$t" 2>/dev/null | sed -n '1,4p' || true
done

if [[ "$MODE" == "--hi-test" ]]; then
  echo
echo '=== HI test: expecting a natural OpenRouter-generated greeting ==='
  python - <<'PY'
import json, time
import rclpy
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node('sparky_frontdoor_hi_test')
seen=[]
def cb(m):
    seen.append(m.data)
sub=n.create_subscription(String,'/go2_tts/say',cb,20)
pub=n.create_publisher(String,'/go2_voice/transcript',20)
for _ in range(15): rclpy.spin_once(n,timeout_sec=0.05)
now=time.time()
payload={'text':'Sparky, hi','confidence':1.0,'source':'phone_web','input_kind':'unified','request_id':f'hi_{time.time_ns()}','command_received_unix':now}
pub.publish(String(data=json.dumps(payload)))
deadline=time.time()+18
while time.time()<deadline and not seen:
    rclpy.spin_once(n,timeout_sec=0.1)
if seen:
    print('TTS:',seen[-1])
    bad='I will proceed safely.' in seen[-1]
    print('PASS' if not bad else 'FAIL', '- no heuristic proceed-safely boilerplate')
else:
    print('FAIL: no /go2_tts/say response within 18s')
n.destroy_node(); rclpy.shutdown()
PY
fi

if [[ "$MODE" == "--vision-route-test" ]]; then
  echo
echo '=== Vision route test: expecting OpenRouter -> /go2_vlm/query ==='
  python - <<'PY'
import json, time
import rclpy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('sparky_frontdoor_vision_route_test')
seen=[]
def cb(m): seen.append(m.data)
n.create_subscription(String,'/go2_vlm/query',cb,20)
p=n.create_publisher(String,'/go2_voice/transcript',20)
for _ in range(15): rclpy.spin_once(n,timeout_sec=0.05)
now=time.time(); p.publish(String(data=json.dumps({'text':'Sparky, what do you see in front of you?','confidence':1.0,'source':'phone_web','request_id':f'vision_{time.time_ns()}','command_received_unix':now})))
deadline=time.time()+18
while time.time()<deadline and not seen: rclpy.spin_once(n,timeout_sec=0.1)
if not seen:
    print('FAIL: no VLM handoff within 18s')
else:
    print('VLM HANDOFF:',seen[-1])
    try:
        d=json.loads(seen[-1]); print('PASS' if float(d.get('sensor_not_before_unix',0))>=now else 'FAIL','- sensor barrier is after user command')
    except Exception as e: print('FAIL: invalid VLM JSON',e)
n.destroy_node(); rclpy.shutdown()
PY
fi
