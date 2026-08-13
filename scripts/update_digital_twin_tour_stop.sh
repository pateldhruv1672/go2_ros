#!/usr/bin/env bash
set -euo pipefail
NAME="${1:-}"
SCRIPT="${2:-}"
FACT="${3:-}"
if [[ -z "$NAME" || -z "$SCRIPT" ]]; then
  echo "Usage: $0 <stop_name> '<new narration>' ['short fact']" >&2
  exit 2
fi
PAYLOAD="$(python3 - "$NAME" "$SCRIPT" "$FACT" <<'PY'
import json,sys
name,script,fact=sys.argv[1:]
p={"type":"update_tour_stop","name":name,"script":script}
if fact: p["fact"]=fact
print(json.dumps(p,separators=(",",":")))
PY
)"
ESCAPED="${PAYLOAD//\'/\'\'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: '$ESCAPED'}"
