#!/usr/bin/env bash
set -euo pipefail
SESSION="${1:-$(basename "$(ls -td "$HOME/.ros/go2_semantic_nav_sessions"/* | head -1)")}"
echo "SESSION=$SESSION"
echo '=== LIVE PIPELINE ==='
ros2 topic info /object_explorer/sam2_detections 2>/dev/null || true
ros2 topic info /go2_vln/target_detections_3d 2>/dev/null || true
ros2 topic info /go2_vln/object_map 2>/dev/null || true
echo '=== PROJECTOR STATUS ==='
ros2 topic echo /go2_vln/projection_status --once 2>/dev/null || true
echo '=== UNIQUE MEMORY ==='
python - "$SESSION" <<'PY'
import json,sys
from pathlib import Path
root=Path.home()/'.ros/go2_semantic_nav_sessions'/sys.argv[1]/'memory'/'objects.jsonl'
rows=[]
if root.is_file():
  for line in root.read_text().splitlines():
    try:r=json.loads(line)
    except:continue
    d=r.get('data') if isinstance(r,dict) and isinstance(r.get('data'),dict) else r
    if isinstance(d,dict) and d.get('confirmed'): rows.append(r)
print('raw_confirmed_records=',len(rows))
try:
  from go2_langgraph_agent.tools.object_memory_dedup import deduplicate_records,record_data
  unique=deduplicate_records(rows)
  print('deduplicated_unique_objects=',len(unique))
  for r in unique[:80]:
    d=record_data(r); p=d.get('object_pose') or d.get('map_pose') or {}
    print(f"{d.get('label','object'):18s} id={d.get('object_id') or r.get('id')} x={p.get('x')} y={p.get('y')} members={d.get('dedup_size',1)} conf={d.get('mapper_confidence')}")
except Exception as e:
  print('dedup audit error:',e)
PY
