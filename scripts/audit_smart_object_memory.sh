#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/.ros/go2_semantic_nav_sessions"
SESSION="${1:-$(basename "$(ls -td "$ROOT"/* | head -1)")}"
FILE="$ROOT/$SESSION/memory/objects.jsonl"
echo "SESSION=$SESSION"
echo "FILE=$FILE"
python - "$FILE" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); rows=[]
if p.is_file():
  for line in p.read_text().splitlines():
    try:r=json.loads(line)
    except:continue
    d=r.get('data') or {}
    if not d.get('confirmed'):continue
    rows.append((d.get('label'), d.get('mapper_confidence') or (d.get('confidence') or {}).get('perception_confidence'), d.get('object_pose') or d.get('map_pose'), d.get('navigation_approach_pose')))
print('confirmed_objects=',len(rows))
for label,conf,obj,nav in rows[:80]:
  print(f"{str(label):18s} conf={conf} object={obj} nav={nav}")
PY
