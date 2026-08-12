from __future__ import annotations

import math
import os
import re
import sqlite3
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def _norm(text: str) -> str:
    return " ".join(re.sub(r"[^a-z0-9\s]", " ", (text or "").lower()).split())


def canonical_object_label(text: str) -> tuple[str, bool]:
    """Return a compact object label plus whether the user asked for the nearest one."""
    raw = _norm(text)
    nearest = bool(re.search(r"\b(nearest|nearerst|closest|closeest)\b", raw))
    raw = re.sub(r"^(hey\s+)?sparky\s+", "", raw)
    raw = re.sub(r"^(please\s+)?(can you\s+|could you\s+|would you\s+)?", "", raw)
    raw = re.sub(r"^(find|locate|search for|look for|go find|navigate to)\s+", "", raw)
    raw = re.sub(r"\b(nearest|nearerst|closest|closeest|nearby)\b", " ", raw)
    raw = re.sub(r"^(the|a|an|my|our)\s+", "", raw.strip())
    raw = re.sub(r"\b(to me|from me|around me|near me|nearby|please|now)\b", " ", raw)
    raw = " ".join(raw.split())
    # Keep the semantic noun phrase small.  For the current object classes, the
    # final noun is usually the most reliable class label.
    tokens = raw.split()
    if len(tokens) > 3:
        raw = " ".join(tokens[-3:])
    if raw.endswith("s") and len(raw) > 3:
        # WorldMemory already performs singular matching, but returning the
        # singular form makes SQL lookup deterministic too.
        raw = raw[:-1]
    return raw or "object", nearest


class FusedMemoryBridge:
    """Read-only bridge over durable semantic JSONL + mapper SQLite memory.

    JSONL remains the durable unified-memory source.  The mapper SQLite DB is
    queried as a read-only fallback/index so the agent can immediately use
    confirmed objects even when the JSONL bridge is stale or incomplete.
    """

    def __init__(self, api: Any, session_name: str):
        self.api = api
        self.session_name = session_name

    @property
    def session_dir(self) -> Path:
        return Path(self.api.store.session_dir(self.session_name))

    def mapper_db_candidates(self) -> List[Path]:
        candidates = [
            self.session_dir / "object_mapper.sqlite3",
            self.session_dir / "object_map.sqlite3",
            self.session_dir / "memory" / "object_mapper.sqlite3",
            Path(os.path.expanduser("~/.ros/go2_sysnav_vln/object_map.sqlite3")),
        ]
        out: List[Path] = []
        seen = set()
        for path in candidates:
            key = str(path)
            if key not in seen:
                out.append(path)
                seen.add(key)
        return out

    @staticmethod
    def _has_objects_table(path: Path) -> bool:
        if not path.is_file():
            return False
        try:
            uri = f"file:{path}?mode=ro"
            with sqlite3.connect(uri, uri=True, timeout=0.2) as db:
                row = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='objects'").fetchone()
                return bool(row)
        except Exception:
            return False

    def mapper_db_path(self) -> Optional[Path]:
        for path in self.mapper_db_candidates():
            if self._has_objects_table(path):
                return path
        return None

    def query_mapper_objects(
        self,
        label: str = "",
        confirmed_only: bool = True,
        limit: int = 100,
        robot_map_pose: Optional[Dict[str, Any]] = None,
        nearest: bool = False,
    ) -> Dict[str, Any]:
        path = self.mapper_db_path()
        canonical, inferred_nearest = canonical_object_label(label)
        nearest = bool(nearest or inferred_nearest)
        if path is None:
            return {"success": False, "objects": [], "count": 0, "db_path": "", "canonical_label": canonical}
        try:
            uri = f"file:{path}?mode=ro"
            db = sqlite3.connect(uri, uri=True, timeout=0.35)
            db.row_factory = sqlite3.Row
            where = []
            args: List[Any] = []
            if canonical and canonical != "object":
                where.append("LOWER(label) LIKE ?")
                args.append(f"%{canonical.lower()}%")
            if confirmed_only:
                where.append("confirmed = 1")
            sql = (
                "SELECT object_id,label,x,y,z,extent_x,extent_y,extent_z,confidence,"
                "confirmations,observations,variance_m2,first_seen,last_seen,confirmed,source FROM objects"
            )
            if where:
                sql += " WHERE " + " AND ".join(where)
            sql += " ORDER BY last_seen DESC LIMIT ?"
            args.append(max(1, int(limit) * 4))
            rows = db.execute(sql, args).fetchall()
            objects: List[Dict[str, Any]] = []
            rx = ry = None
            if isinstance(robot_map_pose, dict) and "x" in robot_map_pose and "y" in robot_map_pose:
                rx, ry = float(robot_map_pose["x"]), float(robot_map_pose["y"])
            for row in rows:
                oid = int(row["object_id"])
                obs = db.execute(
                    "SELECT robot_x,robot_y,robot_yaw,stamp FROM observations "
                    "WHERE object_id=? AND independent=1 ORDER BY observation_id DESC LIMIT 1",
                    (oid,),
                ).fetchone()
                approach = {}
                if obs is not None:
                    yaw = float(obs["robot_yaw"])
                    approach = {
                        "frame_id": "map",
                        "x": float(obs["robot_x"]),
                        "y": float(obs["robot_y"]),
                        "z": 0.0,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": math.sin(0.5 * yaw),
                        "qw": math.cos(0.5 * yaw),
                        "source": "mapper_sql_last_confirmed_viewpoint",
                    }
                distance = None
                if rx is not None and ry is not None:
                    distance = math.hypot(float(row["x"]) - rx, float(row["y"]) - ry)
                data = {
                    "label": str(row["label"]),
                    "map_pose": {"frame_id": "map", "x": float(row["x"]), "y": float(row["y"]), "z": float(row["z"]), "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                    "approach_pose": approach,
                    "confidence": float(row["confidence"]),
                    "confirmations": int(row["confirmations"]),
                    "observations": int(row["observations"]),
                    "confirmed": bool(row["confirmed"]),
                    "last_seen": float(row["last_seen"]),
                    "source": str(row["source"]),
                    "source_object_id": oid,
                    "memory_source": "mapper_sqlite",
                }
                if distance is not None:
                    data["distance_from_robot_m"] = float(distance)
                objects.append({"id": f"mapper_sql_{oid}", "type": "ObjectInstance", "session_name": self.session_name, "layer": "permanent" if bool(row["confirmed"]) else "temporary", "data": data})
            db.close()
            if nearest and rx is not None and ry is not None:
                objects.sort(key=lambda r: float((r.get("data") or {}).get("distance_from_robot_m", 1e12)))
            return {
                "success": True,
                "objects": objects[: max(1, int(limit))],
                "count": len(objects),
                "db_path": str(path),
                "canonical_label": canonical,
                "nearest": nearest,
            }
        except Exception as exc:
            return {"success": False, "objects": [], "count": 0, "db_path": str(path), "canonical_label": canonical, "error": str(exc)}

    @staticmethod
    def _dedup_objects(records: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for rec in records:
            data = rec.get("data") or {}
            label = _norm(str(data.get("label") or ""))
            pose = data.get("map_pose") or {}
            duplicate = False
            for old in out:
                od = old.get("data") or {}
                if _norm(str(od.get("label") or "")) != label:
                    continue
                op = od.get("map_pose") or {}
                if all(k in pose for k in ("x", "y")) and all(k in op for k in ("x", "y")):
                    if math.hypot(float(pose["x"]) - float(op["x"]), float(pose["y"]) - float(op["y"])) <= 0.35:
                        duplicate = True
                        # Prefer the record that contains a safe approach pose.
                        if not od.get("approach_pose") and data.get("approach_pose"):
                            old["data"] = dict(od, approach_pose=data.get("approach_pose"))
                        break
            if not duplicate:
                out.append(rec)
        return out

    def fuse_object_results(
        self,
        unified: Dict[str, Any],
        label: str,
        confirmed_only: bool,
        limit: int,
        robot_map_pose: Optional[Dict[str, Any]] = None,
        nearest: bool = False,
    ) -> Dict[str, Any]:
        canonical, inferred_nearest = canonical_object_label(label)
        nearest = bool(nearest or inferred_nearest)
        sql_result = self.query_mapper_objects(canonical, confirmed_only, limit, robot_map_pose, nearest)
        merged = self._dedup_objects([*(unified.get("objects") or []), *(sql_result.get("objects") or [])])
        rx = ry = None
        if isinstance(robot_map_pose, dict) and "x" in robot_map_pose and "y" in robot_map_pose:
            rx, ry = float(robot_map_pose["x"]), float(robot_map_pose["y"])
            for rec in merged:
                data = rec.get("data") or {}
                pose = data.get("map_pose") or {}
                if "x" in pose and "y" in pose:
                    data["distance_from_robot_m"] = math.hypot(float(pose["x"]) - rx, float(pose["y"]) - ry)
        if nearest and rx is not None and ry is not None:
            merged.sort(key=lambda r: float((r.get("data") or {}).get("distance_from_robot_m", 1e12)))
        else:
            merged.sort(key=lambda r: float((r.get("data") or {}).get("last_seen", 0.0) or 0.0), reverse=True)
        result = dict(unified)
        result.update({
            "success": True,
            "objects": merged[: max(1, int(limit))],
            "count": len(merged),
            "canonical_label": canonical,
            "nearest": nearest,
            "mapper_sql": {k: v for k, v in sql_result.items() if k != "objects"},
            "memory_sources": ["unified_jsonl", "mapper_sqlite"],
        })
        return result

    def vlm_checkpoints(self, text: str = "", limit: int = 30) -> List[Dict[str, Any]]:
        records = self.api.store.read_jsonl(self.session_name, "memory/checkpoints.jsonl")
        vlm = []
        target = _norm(text)
        for rec in records:
            data = rec.get("data") or {}
            if not (data.get("vlm_summary") or "vlm" in [str(x).lower() for x in (rec.get("source") or data.get("source") or [])]):
                continue
            if target:
                hay = _norm(" ".join([str(data.get("label") or ""), str(data.get("vlm_summary") or ""), str(data.get("object_inventory") or "")]))
                if target not in hay:
                    continue
            vlm.append(rec)
        return vlm[-max(1, int(limit)):]
