from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple
import json
import math
import os
import sqlite3
import time
import uuid


def voxel_key_for(x: float, y: float, z: float, size_m: float) -> Tuple[int, int, int]:
    return (math.floor(x / size_m), math.floor(y / size_m), math.floor(z / size_m))


def voxel_id_for(x: float, y: float, z: float, size_m: float) -> str:
    ix, iy, iz = voxel_key_for(x, y, z, size_m)
    return f"vox_{ix}_{iy}_{iz}"


def voxel_center_for(x: float, y: float, z: float, size_m: float) -> Dict[str, float]:
    ix, iy, iz = voxel_key_for(x, y, z, size_m)
    half = size_m / 2.0
    return {"x": ix * size_m + half, "y": iy * size_m + half, "z": iz * size_m + half}


class SemanticVoxelStore:
    """SQLite-backed persistent semantic voxel memory.

    This store aggregates repeated observations into one stable voxel row, keeps a
    raw observation log for auditability, and provides query/comparison helpers
    used by resume/tour/explore modes. It has no ROS dependency, so tests and bag
    processing can use it directly.
    """

    def __init__(self, session_root: str, session_name: str, voxel_size_m: float = 0.25):
        self.session_root = Path(os.path.expanduser(session_root)).resolve()
        self.session_name = session_name or "default"
        self.voxel_size_m = float(voxel_size_m)
        self.dir = self.session_root / self.session_name / "voxel_memory"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / "semantic_voxels.db"
        self.legacy_jsonl_path = self.dir / "semantic_voxels.jsonl"
        self.conn = sqlite3.connect(str(self.path))
        self.conn.row_factory = sqlite3.Row
        self._ensure_schema()
        self._migrate_legacy_jsonl_once()

    def close(self) -> None:
        self.conn.close()

    def write_observation(
        self,
        center_xyz: Dict[str, float],
        labels: List[str],
        properties: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        properties = properties or {}
        x = float(center_xyz.get("x", 0.0))
        y = float(center_xyz.get("y", 0.0))
        z = float(center_xyz.get("z", 0.0))
        center = voxel_center_for(x, y, z, self.voxel_size_m)
        voxel_id = voxel_id_for(x, y, z, self.voxel_size_m)
        now = self._now()
        labels = sorted({str(label).strip() for label in labels if str(label).strip()})
        object_ids = sorted({str(x) for x in properties.get("object_ids", []) if str(x)})
        occupancy = float(properties.get("occupancy_probability", properties.get("occupancy", 0.5)))
        traversability = float(properties.get("traversability_score", 1.0 - occupancy))
        confidence = properties.get("source_confidence", properties.get("confidence", {}))
        if not isinstance(confidence, dict):
            confidence = {"overall": float(confidence)}
        observation_id = "voxel_obs_" + uuid.uuid4().hex[:12]
        self.conn.execute(
            """
            INSERT INTO observations(observation_id, voxel_id, x, y, z, labels_json, properties_json, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (observation_id, voxel_id, x, y, z, json.dumps(labels, sort_keys=True), json.dumps(properties, sort_keys=True), now),
        )
        existing = self._get_voxel_row(voxel_id)
        if existing:
            seen_count = int(existing["seen_count"]) + 1
            old_labels = set(json.loads(existing["semantic_labels_json"] or "[]"))
            old_objects = set(json.loads(existing["object_ids_json"] or "[]"))
            merged_labels = sorted(old_labels.union(labels))
            merged_objects = sorted(old_objects.union(object_ids))
            new_occ = self._running_average(float(existing["occupancy_probability"]), occupancy, seen_count)
            new_trav = self._running_average(float(existing["traversability_score"]), traversability, seen_count)
            merged_conf = self._merge_confidence(json.loads(existing["modality_confidence_json"] or "{}"), confidence)
            merged_props = self._merge_properties(json.loads(existing["properties_json"] or "{}"), properties)
            self.conn.execute(
                """
                UPDATE voxels
                SET occupancy_probability=?, traversability_score=?, semantic_labels_json=?, object_ids_json=?,
                    seen_count=?, last_seen=?, modality_confidence_json=?, properties_json=?
                WHERE voxel_id=?
                """,
                (
                    new_occ,
                    new_trav,
                    json.dumps(merged_labels, sort_keys=True),
                    json.dumps(merged_objects, sort_keys=True),
                    seen_count,
                    now,
                    json.dumps(merged_conf, sort_keys=True),
                    json.dumps(merged_props, sort_keys=True),
                    voxel_id,
                ),
            )
        else:
            self.conn.execute(
                """
                INSERT INTO voxels(voxel_id, center_x, center_y, center_z, size_m, occupancy_probability,
                                   traversability_score, semantic_labels_json, object_ids_json, seen_count,
                                   first_seen, last_seen, modality_confidence_json, properties_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    voxel_id,
                    center["x"],
                    center["y"],
                    center["z"],
                    self.voxel_size_m,
                    occupancy,
                    traversability,
                    json.dumps(labels, sort_keys=True),
                    json.dumps(object_ids, sort_keys=True),
                    1,
                    now,
                    now,
                    json.dumps(confidence, sort_keys=True),
                    json.dumps(properties, sort_keys=True),
                ),
            )
        self.conn.commit()
        record = self.get_voxel(voxel_id)
        record["observation_id"] = observation_id
        return record

    def write_pointcloud_observation(
        self,
        points: Sequence[Sequence[float] | Dict[str, float]],
        labels: List[str] | None = None,
        properties: Dict[str, Any] | None = None,
        max_points: int = 2000,
    ) -> Dict[str, Any]:
        labels = labels or ["pointcloud_observed"]
        properties = properties or {"source": "pointcloud"}
        written: Dict[str, Dict[str, Any]] = {}
        for idx, point in enumerate(points):
            if idx >= max_points:
                break
            x, y, z = self._point_xyz(point)
            voxel_id = voxel_id_for(x, y, z, self.voxel_size_m)
            if voxel_id in written:
                continue
            written[voxel_id] = self.write_observation({"x": x, "y": y, "z": z}, labels, properties)
        return {"success": True, "unique_voxels_written": len(written), "voxel_ids": sorted(written)}

    def query_near(self, x: float, y: float, radius_m: float = 2.0, limit: int = 100) -> List[Dict[str, Any]]:
        rows = self.conn.execute(
            """
            SELECT * FROM voxels
            WHERE ((center_x - ?) * (center_x - ?) + (center_y - ?) * (center_y - ?)) <= (? * ?)
            ORDER BY seen_count DESC, last_seen DESC
            LIMIT ?
            """,
            (x, x, y, y, radius_m, radius_m, int(limit)),
        ).fetchall()
        return [self._row_to_record(row) for row in rows]

    def query_box(self, min_xyz: Dict[str, float], max_xyz: Dict[str, float], limit: int = 1000) -> List[Dict[str, Any]]:
        rows = self.conn.execute(
            """
            SELECT * FROM voxels
            WHERE center_x BETWEEN ? AND ? AND center_y BETWEEN ? AND ? AND center_z BETWEEN ? AND ?
            ORDER BY seen_count DESC
            LIMIT ?
            """,
            (
                float(min_xyz.get("x", -math.inf)),
                float(max_xyz.get("x", math.inf)),
                float(min_xyz.get("y", -math.inf)),
                float(max_xyz.get("y", math.inf)),
                float(min_xyz.get("z", -math.inf)),
                float(max_xyz.get("z", math.inf)),
                int(limit),
            ),
        ).fetchall()
        return [self._row_to_record(row) for row in rows]

    def query_landmarks(self, x: float, y: float, radius_m: float = 5.0, min_seen_count: int = 2, limit: int = 50) -> List[Dict[str, Any]]:
        candidates = self.query_near(x, y, radius_m=radius_m, limit=limit * 4)
        return [v for v in candidates if int(v.get("seen_count", 0)) >= min_seen_count and v.get("semantic_labels")][:limit]

    def get_traversability(self, x: float, y: float, radius_m: float = 0.75) -> Dict[str, Any]:
        voxels = self.query_near(x, y, radius_m=radius_m, limit=50)
        if not voxels:
            return {"known": False, "traversability_score": 0.5, "voxel_count": 0}
        score = sum(float(v.get("traversability_score", 0.5)) for v in voxels) / len(voxels)
        hazards = sorted({label for v in voxels for label in v.get("semantic_labels", []) if label in {"hazard", "blocked", "obstacle", "stairs", "person"}})
        return {"known": True, "traversability_score": score, "voxel_count": len(voxels), "hazard_labels": hazards}

    def compare_pointcloud(
        self,
        points: Sequence[Sequence[float] | Dict[str, float]],
        match_radius_voxels: int = 0,
        max_points: int = 5000,
    ) -> Dict[str, Any]:
        known_ids = {v["voxel_id"] for v in self.read_all(limit=1_000_000)}
        observed_ids: set[str] = set()
        for idx, point in enumerate(points):
            if idx >= max_points:
                break
            x, y, z = self._point_xyz(point)
            observed_ids.add(voxel_id_for(x, y, z, self.voxel_size_m))
        if match_radius_voxels > 0:
            matched = {vid for vid in observed_ids if self._has_neighbor_id(vid, known_ids, match_radius_voxels)}
        else:
            matched = observed_ids.intersection(known_ids)
        novel = observed_ids.difference(matched)
        missing = known_ids.difference(observed_ids)
        observed_count = len(observed_ids)
        return {
            "success": True,
            "observed_voxel_count": observed_count,
            "known_voxel_count": len(known_ids),
            "matched_voxel_count": len(matched),
            "novel_voxel_count": len(novel),
            "missing_known_voxel_count": len(missing),
            "match_ratio": (len(matched) / observed_count) if observed_count else 0.0,
            "novel_voxel_ids": sorted(novel)[:100],
        }

    def get_voxel(self, voxel_id: str) -> Dict[str, Any]:
        row = self._get_voxel_row(voxel_id)
        if row is None:
            return {}
        return self._row_to_record(row)

    def read_all(self, limit: int = 1000) -> List[Dict[str, Any]]:
        rows = self.conn.execute("SELECT * FROM voxels ORDER BY last_seen DESC LIMIT ?", (int(limit),)).fetchall()
        return [self._row_to_record(row) for row in rows]

    def export_summary(self) -> Dict[str, Any]:
        row = self.conn.execute(
            "SELECT COUNT(*) AS n, AVG(occupancy_probability) AS occ, AVG(traversability_score) AS trav, SUM(seen_count) AS observations FROM voxels"
        ).fetchone()
        labels: Dict[str, int] = {}
        for voxel in self.read_all(limit=1_000_000):
            for label in voxel.get("semantic_labels", []):
                labels[label] = labels.get(label, 0) + 1
        return {
            "session_name": self.session_name,
            "db_path": str(self.path),
            "voxel_count": int(row["n"] or 0),
            "total_seen_count": int(row["observations"] or 0),
            "average_occupancy_probability": float(row["occ"] or 0.0),
            "average_traversability_score": float(row["trav"] or 0.0),
            "top_labels": sorted(labels.items(), key=lambda kv: (-kv[1], kv[0]))[:20],
        }

    def _ensure_schema(self) -> None:
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS voxels(
                voxel_id TEXT PRIMARY KEY,
                center_x REAL NOT NULL,
                center_y REAL NOT NULL,
                center_z REAL NOT NULL,
                size_m REAL NOT NULL,
                occupancy_probability REAL NOT NULL,
                traversability_score REAL NOT NULL,
                semantic_labels_json TEXT NOT NULL,
                object_ids_json TEXT NOT NULL,
                seen_count INTEGER NOT NULL,
                first_seen TEXT NOT NULL,
                last_seen TEXT NOT NULL,
                modality_confidence_json TEXT NOT NULL,
                properties_json TEXT NOT NULL
            )
            """
        )
        self.conn.execute("CREATE INDEX IF NOT EXISTS idx_voxels_xy ON voxels(center_x, center_y)")
        self.conn.execute("CREATE INDEX IF NOT EXISTS idx_voxels_last_seen ON voxels(last_seen)")
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS observations(
                observation_id TEXT PRIMARY KEY,
                voxel_id TEXT NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                labels_json TEXT NOT NULL,
                properties_json TEXT NOT NULL,
                created_at TEXT NOT NULL
            )
            """
        )
        self.conn.commit()

    def _migrate_legacy_jsonl_once(self) -> None:
        marker = self.dir / ".jsonl_migrated"
        if marker.exists() or not self.legacy_jsonl_path.exists():
            return
        for line in self.legacy_jsonl_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                self.write_observation(record.get("center_xyz", {}), record.get("semantic_labels", []), record.get("properties", {}))
            except Exception:
                continue
        marker.write_text(self._now(), encoding="utf-8")

    def _get_voxel_row(self, voxel_id: str) -> sqlite3.Row | None:
        return self.conn.execute("SELECT * FROM voxels WHERE voxel_id=?", (voxel_id,)).fetchone()

    @staticmethod
    def _row_to_record(row: sqlite3.Row) -> Dict[str, Any]:
        return {
            "voxel_id": row["voxel_id"],
            "center_xyz": {"x": row["center_x"], "y": row["center_y"], "z": row["center_z"]},
            "size_m": row["size_m"],
            "occupancy_probability": row["occupancy_probability"],
            "traversability_score": row["traversability_score"],
            "semantic_labels": json.loads(row["semantic_labels_json"] or "[]"),
            "object_ids": json.loads(row["object_ids_json"] or "[]"),
            "seen_count": row["seen_count"],
            "first_seen": row["first_seen"],
            "last_seen": row["last_seen"],
            "source_confidence": json.loads(row["modality_confidence_json"] or "{}"),
            "properties": json.loads(row["properties_json"] or "{}"),
        }

    @staticmethod
    def _running_average(old: float, new: float, new_count: int) -> float:
        return ((old * (new_count - 1)) + new) / max(1, new_count)

    @staticmethod
    def _merge_confidence(old: Dict[str, Any], new: Dict[str, Any]) -> Dict[str, Any]:
        merged = dict(old)
        for key, value in new.items():
            try:
                merged[key] = max(float(merged.get(key, 0.0)), float(value))
            except Exception:
                merged[key] = value
        return merged

    @staticmethod
    def _merge_properties(old: Dict[str, Any], new: Dict[str, Any]) -> Dict[str, Any]:
        merged = dict(old)
        for key, value in new.items():
            if key not in merged or merged[key] in (None, "", [], {}):
                merged[key] = value
        return merged

    @staticmethod
    def _point_xyz(point: Sequence[float] | Dict[str, float]) -> Tuple[float, float, float]:
        if isinstance(point, dict):
            return float(point.get("x", 0.0)), float(point.get("y", 0.0)), float(point.get("z", 0.0))
        return float(point[0]), float(point[1]), float(point[2]) if len(point) > 2 else 0.0

    @staticmethod
    def _has_neighbor_id(voxel_id: str, known_ids: set[str], radius: int) -> bool:
        try:
            _, sx, sy, sz = voxel_id.split("_")
            ix, iy, iz = int(sx), int(sy), int(sz)
        except Exception:
            return voxel_id in known_ids
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    if f"vox_{ix + dx}_{iy + dy}_{iz + dz}" in known_ids:
                        return True
        return False

    @staticmethod
    def _now() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
