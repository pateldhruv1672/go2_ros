#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SESSION_ROOT="${SESSION_ROOT:-$HOME/.ros/go2_semantic_nav_sessions}"
SESSION="${1:-${SESSION_NAME:-}}"

if [ -z "$SESSION" ] || [ "$SESSION" = latest ] || [ "$SESSION" = auto ]; then
  SESSION="$(find "$SESSION_ROOT" -mindepth 1 -maxdepth 1 -type d -exec test -f '{}/map.yaml' ';' -printf '%T@ %f\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[ -n "$SESSION" ] || { echo "No resume-ready session found under $SESSION_ROOT" >&2; exit 2; }
SESSION_DIR="$SESSION_ROOT/$SESSION"
[ -d "$SESSION_DIR" ] || { echo "Session not found: $SESSION_DIR" >&2; exit 2; }

# Do not erase while mapper/memory writers are live: they would immediately repopulate the files.
if command -v ros2 >/dev/null 2>&1; then
  ACTIVE="$(ros2 node list 2>/dev/null | grep -E '^/(go2_pose_aware_object_mapper|go2_world_object_memory|fast_sam2_tracker_overlay_node)$' || true)"
  if [ -n "$ACTIVE" ]; then
    echo "Object-memory writers are still running:" >&2
    echo "$ACTIVE" >&2
    echo "Stop scripts/run_sparky_resume_agentic_tour.sh first, then rerun this reset." >&2
    exit 3
  fi
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$HOME/.ros/go2_object_memory_backups/${SESSION}_${STAMP}"
mkdir -p "$BACKUP/session" "$BACKUP/legacy_global_mapper"

echo "Resetting OBJECT memory only for session: $SESSION"
echo "Backup: $BACKUP"

# Fingerprint data that must never be touched by this operation.
PRESERVE=(
  "memory/checkpoints.jsonl"
  "memory/places.jsonl"
  "places.yaml"
  "route.yaml"
  "session.yaml"
  "map.yaml"
  "map.pgm"
)
: > "$BACKUP/preserved_before.sha256"
for rel in "${PRESERVE[@]}"; do
  [ -f "$SESSION_DIR/$rel" ] && sha256sum "$SESSION_DIR/$rel" >> "$BACKUP/preserved_before.sha256"
done

# Backup only object-related session records/artifacts/databases.
for rel in \
  memory/objects.jsonl \
  memory/object_observations.jsonl \
  graph_memory/nodes.jsonl \
  graph_memory/edges.jsonl \
  object_mapper.sqlite3 object_mapper.sqlite3-wal object_mapper.sqlite3-shm
do
  if [ -e "$SESSION_DIR/$rel" ]; then
    mkdir -p "$BACKUP/session/$(dirname "$rel")"
    cp -a "$SESSION_DIR/$rel" "$BACKUP/session/$rel"
  fi
done
for rel in artifacts/object_crops artifacts/object_masks; do
  if [ -e "$SESSION_DIR/$rel" ]; then
    mkdir -p "$BACKUP/session/$(dirname "$rel")"
    cp -a "$SESSION_DIR/$rel" "$BACKUP/session/$rel"
  fi
done

# The pre-v12.5 mapper used one global DB for every session. Archive and remove it
# so an older launcher cannot re-inject the stale cross-session object cloud.
for f in "$HOME/.ros/go2_sysnav_vln/object_map.sqlite3" "$HOME/.ros/go2_sysnav_vln/object_map.sqlite3-wal" "$HOME/.ros/go2_sysnav_vln/object_map.sqlite3-shm"; do
  if [ -e "$f" ]; then
    cp -a "$f" "$BACKUP/legacy_global_mapper/"
    rm -f "$f"
  fi
done

mkdir -p "$SESSION_DIR/memory"
: > "$SESSION_DIR/memory/objects.jsonl"
: > "$SESSION_DIR/memory/object_observations.jsonl"
rm -f "$SESSION_DIR/object_mapper.sqlite3" "$SESSION_DIR/object_mapper.sqlite3-wal" "$SESSION_DIR/object_mapper.sqlite3-shm"
rm -rf "$SESSION_DIR/artifacts/object_crops" "$SESSION_DIR/artifacts/object_masks"
mkdir -p "$SESSION_DIR/artifacts/object_crops" "$SESSION_DIR/artifacts/object_masks"

# Remove only ObjectInstance nodes and edges that point at them from the JSONL
# graph mirror. Checkpoint/Place/TourStop graph records remain byte-for-byte except
# that object-linked edges are removed.
python3 - "$SESSION_DIR" <<'PY'
from pathlib import Path
import json, sys
root=Path(sys.argv[1])
nodes_p=root/'graph_memory/nodes.jsonl'
edges_p=root/'graph_memory/edges.jsonl'
removed=set()
if nodes_p.exists():
    keep=[]
    for line in nodes_p.read_text().splitlines():
        if not line.strip():
            continue
        rec=json.loads(line)
        if str(rec.get('type') or rec.get('node_type') or '') == 'ObjectInstance':
            removed.add(str(rec.get('id') or ''))
        else:
            keep.append(rec)
    nodes_p.write_text(''.join(json.dumps(x,sort_keys=True,default=str)+'\n' for x in keep))
if edges_p.exists() and removed:
    keep=[]
    for line in edges_p.read_text().splitlines():
        if not line.strip():
            continue
        rec=json.loads(line)
        if str(rec.get('from_id') or '') in removed or str(rec.get('to_id') or '') in removed:
            continue
        keep.append(rec)
    edges_p.write_text(''.join(json.dumps(x,sort_keys=True,default=str)+'\n' for x in keep))
print(f'Graph mirror: removed {len(removed)} ObjectInstance node(s); preserved all non-object nodes')
PY

# Verify VLM/checkpoint/place/map material is unchanged.
if [ -s "$BACKUP/preserved_before.sha256" ]; then
  (cd / && sha256sum -c "$BACKUP/preserved_before.sha256") >/tmp/sparky_object_reset_preserve_check.txt
  cat /tmp/sparky_object_reset_preserve_check.txt
fi

CKPTS="$(wc -l < "$SESSION_DIR/memory/checkpoints.jsonl" 2>/dev/null || echo 0)"
PLACES="$(wc -l < "$SESSION_DIR/memory/places.jsonl" 2>/dev/null || echo 0)"
VLM_RAW="$(find "$SESSION_DIR/artifacts/vlm_raw" -type f 2>/dev/null | wc -l || true)"

echo
echo "OBJECT RESET COMPLETE"
echo "  objects.jsonl:             0 records"
echo "  object_observations.jsonl: 0 records"
echo "  mapper DB:                 reset; next launch creates session-local object_mapper.sqlite3"
echo "  VLM checkpoints preserved: $CKPTS"
echo "  semantic places preserved: $PLACES"
echo "  VLM raw artifacts present: $VLM_RAW"
echo "  backup:                    $BACKUP"
echo
echo "Restart with:"
echo "  scripts/run_sparky_resume_agentic_tour.sh"
