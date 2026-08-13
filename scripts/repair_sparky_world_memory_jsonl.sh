#!/usr/bin/env bash
set -euo pipefail
SESSION_NAME="${1:-${SESSION_NAME:-}}"
SESSION_ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"
[ -n "$SESSION_NAME" ] || { echo "usage: $0 <session_name>" >&2; exit 2; }
DIR="$SESSION_ROOT/$SESSION_NAME/memory"
[ -d "$DIR" ] || { echo "object memory directory not found: $DIR" >&2; exit 2; }
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$HOME/.ros/go2_object_memory_backups/jsonl_repair_${SESSION_NAME}_${STAMP}"
mkdir -p "$BACKUP"

python3 - "$DIR" "$BACKUP" <<'PY'
from pathlib import Path
import json, shutil, sys
mem=Path(sys.argv[1]); backup=Path(sys.argv[2])
files=['objects.jsonl','object_observations.jsonl']
for name in files:
    p=mem/name
    if not p.exists():
        print(f'{name}: absent')
        continue
    shutil.copy2(p, backup/name)
    valid=[]; invalid=[]
    for lineno,line in enumerate(p.read_text(encoding='utf-8',errors='replace').splitlines(),1):
        if not line.strip():
            continue
        try:
            value=json.loads(line)
            if not isinstance(value,dict):
                raise ValueError('JSONL row is not an object')
            valid.append(json.dumps(value,sort_keys=True,default=str))
        except Exception as exc:
            invalid.append({'line':lineno,'error':str(exc),'raw':line[:4000]})
    if invalid:
        p.write_text(('\n'.join(valid)+'\n') if valid else '',encoding='utf-8')
        (backup/(name+'.invalid.json')).write_text(json.dumps(invalid,indent=2)+'\n',encoding='utf-8')
    print(f'{name}: valid={len(valid)} invalid_removed={len(invalid)}')
print(f'backup={backup}')
print('VLM checkpoints/places/map/route were not touched.')
PY
