#!/usr/bin/env python3
from __future__ import annotations
import json
import os
import sqlite3
from pathlib import Path

root = Path(os.path.expanduser(os.environ.get("SPARKY_SESSION_ROOT", "~/.ros/go2_semantic_nav_sessions")))
session = os.environ.get("SESSION_NAME", "").strip()
if not session:
    dirs = sorted([p for p in root.glob("*") if p.is_dir()], key=lambda p: p.stat().st_mtime, reverse=True)
    if not dirs:
        raise SystemExit("No semantic sessions found")
    session_dir = dirs[0]
else:
    session_dir = root / session
print(f"session={session_dir.name}")

checkpoints = session_dir / "memory/checkpoints.jsonl"
vlm = []
if checkpoints.exists():
    for line in checkpoints.read_text(errors="replace").splitlines():
        try:
            r = json.loads(line)
        except Exception:
            continue
        d = r.get("data") or {}
        if d.get("vlm_summary"):
            vlm.append(r)
print(f"VLM checkpoints: {len(vlm)}")
for r in vlm[-3:]:
    d = r.get("data") or {}
    print("  -", r.get("id"), str(d.get("vlm_summary") or "")[:180])

candidates = [
    session_dir / "object_mapper.sqlite3",
    session_dir / "object_map.sqlite3",
    session_dir / "memory/object_mapper.sqlite3",
    Path(os.path.expanduser("~/.ros/go2_sysnav_vln/object_map.sqlite3")),
]
for dbp in candidates:
    if not dbp.exists():
        continue
    try:
        db = sqlite3.connect(f"file:{dbp}?mode=ro", uri=True)
        if not db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='objects'").fetchone():
            db.close(); continue
        count = db.execute("SELECT COUNT(*) FROM objects").fetchone()[0]
        confirmed = db.execute("SELECT COUNT(*) FROM objects WHERE confirmed=1").fetchone()[0]
        print(f"Mapper SQL: {dbp} objects={count} confirmed={confirmed}")
        for label, n in db.execute("SELECT label,COUNT(*) FROM objects WHERE confirmed=1 GROUP BY label ORDER BY COUNT(*) DESC,label LIMIT 12"):
            print(f"  {label}: {n}")
        db.close()
        break
    except Exception as exc:
        print(f"SQL warning {dbp}: {exc}")

from go2_langgraph_agent.tools.fused_memory_bridge import canonical_object_label
for text in ["Sparky find the nearerst chair", "find nearest chair", "where is the closest chair"]:
    print(f"parse {text!r} -> {canonical_object_label(text)}")
