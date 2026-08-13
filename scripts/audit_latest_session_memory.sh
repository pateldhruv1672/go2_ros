#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/.ros/go2_semantic_nav_sessions"
SESSION="${1:-}"
if [[ -z "$SESSION" ]]; then
  D="$(ls -td "$ROOT"/* 2>/dev/null | head -1 || true)"
  [[ -n "$D" ]] || { echo "No session found under $ROOT"; exit 1; }
else
  D="$ROOT/$SESSION"
fi
[[ -d "$D" ]] || { echo "Session directory not found: $D"; exit 2; }
echo "SESSION=$(basename "$D")"
echo "DIR=$D"
echo
for rel in \
  map.yaml places.yaml route.yaml tour_host.yaml \
  memory/objects.jsonl memory/object_observations.jsonl memory/checkpoints.jsonl \
  memory/vlm_checkpoints.jsonl memory/places.jsonl memory/tour_stops.jsonl; do
  f="$D/$rel"
  if [[ -f "$f" ]]; then
    if [[ "$f" == *.jsonl ]]; then printf '%-38s %8d rows\n' "$rel" "$(grep -cve '^\s*$' "$f" || true)"
    else printf '%-38s %8s\n' "$rel" "present"; fi
  else printf '%-38s %8s\n' "$rel" "MISSING"; fi
done
python3 - "$D" <<'PY2'
from pathlib import Path
from collections import Counter
import json, sys, yaml
D=Path(sys.argv[1])
rows=[]
p=D/'memory/objects.jsonl'
if p.is_file():
    for line in p.read_text(errors='replace').splitlines():
        try:
            r=json.loads(line); data=r.get('data') if isinstance(r.get('data'),dict) else r
            if bool(data.get('confirmed',True)) and bool(data.get('countable',True)):
                rows.append((str(r.get('id') or data.get('object_id') or data.get('id') or len(rows)),str(data.get('label') or 'object')))
        except Exception: pass
latest={oid:label for oid,label in rows}; counts=Counter(latest.values())
print('\nCONFIRMED OBJECT MEMORY:',len(latest))
print('OBJECT CLASSES:',dict(counts))
route={}
rp=D/'route.yaml'
if rp.is_file():
    route=yaml.safe_load(rp.read_text()) or {}; route=route.get('route',route) if isinstance(route,dict) else {}
print('TOUR STOPS:',len((route or {}).get('stops') or []))
pp=D/'places.yaml'; places={}
if pp.is_file(): places=yaml.safe_load(pp.read_text()) or {}
print('SEMANTIC PLACES:',len((places or {}).get('places') or []))
PY2
