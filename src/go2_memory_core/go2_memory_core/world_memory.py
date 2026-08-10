from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from .backends.artifact_store_files import ArtifactStoreFiles
from .backends.graph_store_kuzu import GraphStoreKuzu
from .memory_schema import GraphEdge, GraphNode, MemoryRecord, new_id, normalize_json_payload, pose_from_dict, utc_now


def _slug(value: str) -> str:
    value = re.sub(r"[^a-z0-9]+", "_", (value or "").lower()).strip("_")
    return value or "item"


def _norm_label(value: str) -> str:
    value = re.sub(r"[^a-z0-9 ]+", " ", (value or "").lower())
    return " ".join(value.split())


def _point_in_polygon(x: float, y: float, polygon: Iterable[Any]) -> bool:
    points: List[tuple[float, float]] = []
    for p in polygon or []:
        if isinstance(p, dict):
            try:
                points.append((float(p["x"]), float(p["y"])))
            except Exception:
                continue
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            try:
                points.append((float(p[0]), float(p[1])))
            except Exception:
                continue
    if len(points) < 3:
        return False
    inside = False
    j = len(points) - 1
    for i, (xi, yi) in enumerate(points):
        xj, yj = points[j]
        crosses = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if crosses:
            inside = not inside
        j = i
    return inside


class WorldMemoryStore:
    """Persistent room/object/fact/tour memory layered on the existing session store.

    This deliberately keeps JSONL mirrors as the source of truth so the memory can
    be inspected and recovered even when Kuzu/vector dependencies are unavailable.
    Graph nodes are an index/relationship layer, not the sole durable copy.
    """

    ROOMS = "memory/rooms.jsonl"
    OBJECTS = "memory/objects.jsonl"
    OBSERVATIONS = "memory/object_observations.jsonl"
    FACTS = "memory/facts.jsonl"
    TOUR_STOPS = "memory/tour_stops.jsonl"
    WEB_CACHE = "memory/web_cache.jsonl"

    def __init__(self, store: ArtifactStoreFiles, graph_getter, enable_graph_memory: bool = True):
        self.store = store
        self.graph_getter = graph_getter
        self.enable_graph_memory = bool(enable_graph_memory)

    @staticmethod
    def _rewrite_jsonl(path: Path, records: List[Dict[str, Any]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            "".join(json.dumps(r, sort_keys=True, default=str) + "\n" for r in records),
            encoding="utf-8",
        )

    def _upsert(self, session_name: str, relative_path: str, record: Dict[str, Any]) -> Dict[str, Any]:
        records = self.store.read_jsonl(session_name, relative_path)
        found = False
        for idx, old in enumerate(records):
            if str(old.get("id")) == str(record.get("id")):
                records[idx] = record
                found = True
                break
        if not found:
            records.append(record)
        self._rewrite_jsonl(self.store.session_dir(session_name) / relative_path, records)
        return record

    def _graph_node(self, session_name: str, record: Dict[str, Any], label: str = "") -> None:
        if not self.enable_graph_memory:
            return
        data = record.get("data") or {}
        self.graph_getter(session_name).add_node(
            GraphNode(
                id=str(record.get("id")),
                type=str(record.get("type")),
                session_name=session_name,
                label=label or data.get("name") or data.get("label") or str(record.get("id")),
                layer=str(record.get("layer") or "permanent"),
                map_pose=data.get("map_pose"),
                properties=data,
            ).to_dict()
        )

    def _graph_edge(self, session_name: str, from_id: str, to_id: str, edge_type: str, props: Dict[str, Any] | None = None) -> None:
        if not self.enable_graph_memory or not from_id or not to_id:
            return
        edge_id = f"{_slug(edge_type)}_{_slug(from_id)}_{_slug(to_id)}"
        self.graph_getter(session_name).add_edge(
            GraphEdge(
                id=edge_id,
                type=edge_type,
                session_name=session_name,
                from_id=from_id,
                to_id=to_id,
                properties=props or {},
            ).to_dict()
        )

    def write_room(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        name = str(data.get("name") or data.get("label") or data.get("room_id") or "room").strip()
        room_id = str(data.get("room_id") or data.get("id") or f"room_{_slug(name)}")
        if data.get("map_pose") or data.get("pose"):
            data["map_pose"] = pose_from_dict(data.get("map_pose") or data.get("pose"), "map")
        data.setdefault("name", name)
        data.setdefault("aliases", [])
        data.setdefault("region", {})
        data.setdefault("radius_m", 5.0)
        data.setdefault("verified", False)
        data.setdefault("created_at", utc_now())
        record = MemoryRecord(
            id=room_id,
            type="Room",
            session_name=session_name,
            layer="permanent" if data.get("verified") else str(data.get("layer") or "temporary"),
            confidence=data.get("confidence", {}),
            source=data.get("source", ["teach"]),
            data=data,
        ).to_dict()
        self._upsert(session_name, self.ROOMS, record)
        self._graph_node(session_name, record, name)
        return {"success": True, "id": room_id, "record": record}

    def write_object_observation(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        obs_id = str(data.get("observation_id") or data.get("id") or new_id("objobs"))
        data.setdefault("timestamp", utc_now())
        record = MemoryRecord(
            id=obs_id,
            type="ObjectObservation",
            session_name=session_name,
            layer=str(data.get("layer") or "session"),
            confidence=data.get("confidence", {}),
            source=data.get("source", ["perception"]),
            data=data,
        ).to_dict()
        self.store.append_jsonl(session_name, self.OBSERVATIONS, record)
        return {"success": True, "id": obs_id, "record": record}

    def _assign_room(self, session_name: str, x: float, y: float) -> str:
        rooms = self.store.read_jsonl(session_name, self.ROOMS)
        best_id = ""
        best_dist = float("inf")
        for room in rooms:
            data = room.get("data") or {}
            region = data.get("region") or {}
            polygon = region.get("polygon") or region.get("points") or data.get("polygon") or []
            if polygon and _point_in_polygon(x, y, polygon):
                return str(room.get("id") or "")
            pose = data.get("map_pose") or {}
            if "x" not in pose or "y" not in pose:
                continue
            dist = math.hypot(float(pose.get("x", 0.0)) - x, float(pose.get("y", 0.0)) - y)
            radius = float(data.get("radius_m", 5.0) or 5.0)
            if dist <= radius and dist < best_dist:
                best_dist = dist
                best_id = str(room.get("id") or "")
        return best_id

    def upsert_object(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        label = _norm_label(str(data.get("label") or data.get("class_name") or data.get("class") or "object"))
        source_object_id = data.get("source_object_id") or data.get("track_id")
        requested_object_id = str(data.get("object_id") or data.get("id") or "").strip()
        proposed_object_id = requested_object_id or (
            f"object_{_slug(label)}_{source_object_id}" if source_object_id is not None else new_id("object")
        )
        map_pose = data.get("map_pose") or data.get("pose")
        if not map_pose and data.get("x") is not None and data.get("y") is not None:
            map_pose = {
                "frame_id": "map",
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "z": float(data.get("z", 0.0)),
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            }
        if map_pose:
            data["map_pose"] = pose_from_dict(map_pose, "map")

        # SPARKY_SPATIAL_OBJECT_ID_V1
        # Runtime mapper IDs are not durable across Teach/Resume processes. Reconcile
        # same-class observations by map position before accepting a mapper-derived ID.
        existing_objects = self.store.read_jsonl(session_name, self.OBJECTS)
        merge_radius_m = float(data.get("merge_radius_m", 0.45) or 0.45)
        matched_id = ""
        matched_old_data = {}
        best_dist = float("inf")
        pose_now = data.get("map_pose") or {}
        if "x" in pose_now and "y" in pose_now:
            x_now = float(pose_now.get("x", 0.0))
            y_now = float(pose_now.get("y", 0.0))
            for old in existing_objects:
                old_data = old.get("data") or {}
                if _norm_label(str(old_data.get("label") or "")) != label:
                    continue
                old_pose = old_data.get("map_pose") or {}
                if "x" not in old_pose or "y" not in old_pose:
                    continue
                dist = math.hypot(float(old_pose.get("x", 0.0)) - x_now, float(old_pose.get("y", 0.0)) - y_now)
                if dist <= merge_radius_m and dist < best_dist:
                    best_dist = dist
                    matched_id = str(old.get("id") or "")
                    matched_old_data = old_data
        if matched_id:
            object_id = matched_id
            if matched_old_data.get("first_seen"):
                data.setdefault("first_seen", matched_old_data.get("first_seen"))
            data["previous_source_object_id"] = matched_old_data.get("source_object_id")
        else:
            existing_ids = {str(old.get("id") or "") for old in existing_objects}
            # Never overwrite an existing durable object merely because a fresh runtime
            # mapper reused its source ID at a different map location.
            object_id = proposed_object_id if proposed_object_id not in existing_ids else new_id(f"object_{_slug(label)}")
        data["source_object_id"] = source_object_id
        data["label"] = label
        data.setdefault("aliases", [])
        data.setdefault("confirmed", False)
        data.setdefault("status", "present")
        data.setdefault("countable", True)
        data.setdefault("first_seen", utc_now())
        data.setdefault("last_seen", utc_now())
        if not data.get("room_id") and data.get("map_pose"):
            p = data["map_pose"]
            data["room_id"] = self._assign_room(session_name, float(p.get("x", 0.0)), float(p.get("y", 0.0)))
        layer = "permanent" if bool(data.get("confirmed")) else str(data.get("layer") or "temporary")
        record = MemoryRecord(
            id=object_id,
            type="ObjectInstance",
            session_name=session_name,
            layer=layer,
            confidence=data.get("confidence", {}),
            source=data.get("source", ["perception"]),
            data=data,
        ).to_dict()
        self._upsert(session_name, self.OBJECTS, record)
        self._graph_node(session_name, record, label)
        room_id = str(data.get("room_id") or "")
        if room_id:
            self._graph_edge(session_name, object_id, room_id, "LOCATED_IN", {"last_seen": data.get("last_seen")})
        return {"success": True, "id": object_id, "record": record}

    def write_fact(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        text = str(data.get("text") or data.get("fact") or "").strip()
        fact_id = str(data.get("fact_id") or data.get("id") or new_id("fact"))
        data.setdefault("text", text)
        data.setdefault("source_type", "user_verified" if data.get("verified") else "agent")
        data.setdefault("fetched_at", utc_now())
        record = MemoryRecord(
            id=fact_id,
            type="Fact",
            session_name=session_name,
            layer="permanent" if data.get("verified") else str(data.get("layer") or "temporary"),
            confidence=data.get("confidence", {}),
            source=data.get("source", [data.get("source_type", "agent")]),
            data=data,
        ).to_dict()
        self._upsert(session_name, self.FACTS, record)
        self._graph_node(session_name, record, text[:80])
        subject_id = str(data.get("subject_id") or "")
        if subject_id:
            self._graph_edge(session_name, subject_id, fact_id, "HAS_FACT", {"verified": bool(data.get("verified"))})
        return {"success": True, "id": fact_id, "record": record}

    def write_tour_stop(self, session_name: str, payload: str | Dict[str, Any]) -> Dict[str, Any]:
        data = normalize_json_payload(payload)
        name = str(data.get("name") or data.get("label") or "tour stop").strip()
        stop_id = str(data.get("tour_stop_id") or data.get("id") or f"tour_{_slug(name)}")
        if data.get("map_pose") or data.get("pose"):
            data["map_pose"] = pose_from_dict(data.get("map_pose") or data.get("pose"), "map")
        data.setdefault("name", name)
        record = MemoryRecord(
            id=stop_id,
            type="TourStop",
            session_name=session_name,
            layer="permanent",
            confidence=data.get("confidence", {}),
            source=data.get("source", ["tour"]),
            data=data,
        ).to_dict()
        self._upsert(session_name, self.TOUR_STOPS, record)
        self._graph_node(session_name, record, name)
        place_id = str(data.get("place_id") or data.get("room_id") or "")
        if place_id:
            self._graph_edge(session_name, place_id, stop_id, "HAS_TOUR_STOP", {})
        return {"success": True, "id": stop_id, "record": record}

    def query_objects(
        self,
        session_name: str,
        label: str = "",
        room: str = "",
        confirmed_only: bool = True,
        limit: int = 100,
    ) -> Dict[str, Any]:
        target = _norm_label(label)
        room_norm = _norm_label(room)
        rooms = self.store.read_jsonl(session_name, self.ROOMS)
        room_ids = set()
        if room_norm:
            for rec in rooms:
                data = rec.get("data") or {}
                hay = " ".join([str(data.get("name") or ""), *[str(x) for x in data.get("aliases", [])]])
                if room_norm in _norm_label(hay):
                    room_ids.add(str(rec.get("id")))
        out: List[Dict[str, Any]] = []
        for rec in self.store.read_jsonl(session_name, self.OBJECTS):
            data = rec.get("data") or {}
            if confirmed_only and not bool(data.get("confirmed")):
                continue
            if not bool(data.get("countable", True)):
                continue
            if str(data.get("status") or "present") not in {"present", "unknown", "seen"}:
                continue
            if target:
                hay = " ".join([str(data.get("label") or ""), *[str(x) for x in data.get("aliases", [])]])
                norm_hay = _norm_label(hay)
                target_singular = target[:-1] if target.endswith("s") else target
                if target not in norm_hay and target_singular not in norm_hay:
                    continue
            if room_norm:
                rec_room = str(data.get("room_id") or "")
                if room_ids and rec_room not in room_ids:
                    continue
                if not room_ids and room_norm not in _norm_label(rec_room):
                    continue
            out.append(rec)
        def _sort_stamp(record):
            value = (record.get("data") or {}).get("last_seen", 0.0)
            try:
                return float(value or 0.0)
            except Exception:
                return 0.0
        out.sort(key=_sort_stamp, reverse=True)
        return {"success": True, "objects": out[: max(1, int(limit))], "count": len(out)}

    def count_objects(self, session_name: str, label: str = "", room: str = "", confirmed_only: bool = True) -> Dict[str, Any]:
        result = self.query_objects(session_name, label=label, room=room, confirmed_only=confirmed_only, limit=10000)
        counts: Dict[str, int] = {}
        for rec in result.get("objects", []):
            obj_label = str((rec.get("data") or {}).get("label") or "object")
            counts[obj_label] = counts.get(obj_label, 0) + 1
        result["counts"] = counts
        return result

    def snapshot(self, session_name: str, limit: int = 100) -> Dict[str, Any]:
        return {
            "rooms": self.store.read_jsonl(session_name, self.ROOMS)[-limit:],
            "objects": self.store.read_jsonl(session_name, self.OBJECTS)[-limit:],
            "facts": self.store.read_jsonl(session_name, self.FACTS)[-limit:],
            "tour_stops": self.store.read_jsonl(session_name, self.TOUR_STOPS)[-limit:],
            "latest_object_observations": self.store.read_jsonl(session_name, self.OBSERVATIONS)[-min(limit, 30):],
        }

    def cache_web_result(self, session_name: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        data = dict(payload or {})
        data.setdefault("fetched_at", utc_now())
        data.setdefault("layer", "temporary")
        data.setdefault("promotion_policy", "ask_user")
        data.setdefault("verified", False)
        cache_id = str(data.get("id") or new_id("web"))
        record = {"id": cache_id, "type": "WebResearch", "session_name": session_name, "data": data}
        self.store.append_jsonl(session_name, self.WEB_CACHE, record)
        return {"success": True, "id": cache_id, "record": record}
