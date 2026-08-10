from __future__ import annotations

import math
import re
from difflib import SequenceMatcher
from typing import Any, Dict, Iterable, List, Optional, Tuple
from go2_langgraph_agent.tools.object_memory_dedup import deduplicate_records


def _norm(value: str) -> str:
    return " ".join(re.sub(r"[^a-z0-9\s]", " ", str(value or "").lower()).split())


def _singular(value: str) -> str:
    value = _norm(value)
    if value.endswith("ies") and len(value) > 4:
        return value[:-3] + "y"
    if value.endswith("es") and len(value) > 4:
        return value[:-2]
    if value.endswith("s") and len(value) > 3:
        return value[:-1]
    return value


# Small semantic alias layer for common COCO/lab vocabulary.  This is not intent
# routing; it only improves retrieval over already-grounded ObjectInstance records.
_ALIAS_GROUPS = (
    {"tv", "monitor", "screen", "display"},
    {"couch", "sofa"},
    {"cell phone", "phone", "smartphone"},
    {"dining table", "table", "desk"},
    {"robot arm", "robotic arm", "manipulator", "arm"},
    {"person", "human", "guest", "visitor"},
    {"bottle", "water bottle"},
)


def _aliases_for(value: str) -> set[str]:
    q = _norm(value)
    out = {q, _singular(q)}
    for group in _ALIAS_GROUPS:
        ng = {_norm(x) for x in group}
        if q in ng or _singular(q) in {_singular(x) for x in ng}:
            out.update(ng)
            out.update(_singular(x) for x in ng)
    return {x for x in out if x}


def _pose(data: Dict[str, Any]) -> Dict[str, Any]:
    for key in ("object_pose", "map_pose", "position_map", "pose"):
        value = data.get(key)
        if isinstance(value, dict) and value.get("x") is not None and value.get("y") is not None:
            return dict(value)
    return {}


def _confidence(data: Dict[str, Any]) -> float:
    vals: List[float] = []
    for key in ("mapper_confidence", "confidence_score"):
        try:
            vals.append(float(data.get(key)))
        except Exception:
            pass
    conf = data.get("confidence")
    if isinstance(conf, dict):
        for key in ("perception_confidence", "memory_confidence"):
            try:
                vals.append(float(conf.get(key)))
            except Exception:
                pass
    else:
        try:
            vals.append(float(conf))
        except Exception:
            pass
    return max(vals or [0.0])


def _record_data(record: Dict[str, Any]) -> Dict[str, Any]:
    data = record.get("data") if isinstance(record, dict) else None
    return data if isinstance(data, dict) else (record if isinstance(record, dict) else {})


class SmartObjectMemoryTools:
    """Grounded ObjectInstance retrieval and safe object-navigation goal creation.

    Object location and robot observation location are deliberately separate:
      object_pose/map_pose       = physical object centroid in map frame
      last_observer_pose         = robot pose from a confirming observation
      navigation_approach_pose   = robot-safe standoff goal derived from object pose
    """

    def __init__(self, memory_tools: Any, default_standoff_m: float = 0.85) -> None:
        self.memory = memory_tools
        self.default_standoff_m = float(default_standoff_m)

    def _all(self, room: str = "", limit: int = 300) -> List[Dict[str, Any]]:
        try:
            result = self.memory.query_objects(label="", room=room, confirmed_only=True, limit=limit)
        except Exception:
            result = {}
        rows = result.get("objects") if isinstance(result, dict) else []
        rows = [r for r in (rows or []) if isinstance(r, dict)]
        return deduplicate_records(rows)

    def _match_score(self, query: str, record: Dict[str, Any]) -> float:
        data = _record_data(record)
        q_aliases = _aliases_for(query)
        label = _norm(str(data.get("label") or data.get("class_name") or data.get("name") or ""))
        aliases = {_norm(str(x)) for x in (data.get("aliases") or [])}
        hay = {label, _singular(label), *aliases, *(_singular(x) for x in aliases)}
        hay = {x for x in hay if x}
        if not hay:
            return 0.0
        if q_aliases & hay:
            lexical = 1.0
        elif any(q in h or h in q for q in q_aliases for h in hay):
            lexical = 0.88
        else:
            lexical = max((SequenceMatcher(None, q, h).ratio() for q in q_aliases for h in hay), default=0.0)
        if lexical < 0.52:
            return 0.0
        conf = _confidence(data)
        confirmations = int(data.get("confirmations", 0) or 0)
        variance = float(data.get("variance_m2", 0.0) or 0.0)
        position_bonus = 0.12 if _pose(data) else -0.40
        return 4.0 * lexical + min(0.8, conf) + min(0.7, 0.12 * confirmations) + position_bonus - min(0.6, variance * 0.25)

    def search(self, label: str, room: str = "", limit: int = 8) -> Dict[str, Any]:
        ranked: List[Tuple[float, Dict[str, Any]]] = []
        for record in self._all(room=room):
            score = self._match_score(label, record)
            if score > 0.0:
                ranked.append((score, record))
        ranked.sort(key=lambda item: item[0], reverse=True)
        matches = []
        for score, record in ranked[: max(1, int(limit))]:
            data = _record_data(record)
            matches.append({
                "id": record.get("id") or data.get("object_id"),
                "label": data.get("label") or "object",
                "room_id": data.get("room_id") or "",
                "object_pose": _pose(data),
                "navigation_approach_pose": self.navigation_approach_pose(record),
                "confidence": _confidence(data),
                "confirmations": int(data.get("confirmations", 0) or 0),
                "last_seen": data.get("last_seen"),
                "extent_x": data.get("extent_x"),
                "extent_y": data.get("extent_y"),
                "extent_z": data.get("extent_z"),
                "score": round(float(score), 4),
            })
        return {"success": bool(matches), "query": label, "room": room, "matches": matches, "count": len(matches)}

    def navigation_approach_pose(self, record: Dict[str, Any]) -> Dict[str, Any]:
        data = _record_data(record)
        obj = _pose(data)
        if not obj:
            return {}
        # Prefer a previously derived candidate if present.
        prior = data.get("navigation_approach_pose")
        if isinstance(prior, dict) and prior.get("x") is not None and prior.get("y") is not None:
            return dict(prior)
        observer = data.get("last_observer_pose") or data.get("observation_viewpoint")
        if not isinstance(observer, dict) or observer.get("x") is None or observer.get("y") is None:
            # Backward compatibility: older memory stored the robot observation
            # viewpoint under approach_pose. Treat it as an observer, not object location.
            observer = data.get("approach_pose") if isinstance(data.get("approach_pose"), dict) else {}
        try:
            ox, oy = float(obj["x"]), float(obj["y"])
        except Exception:
            return {}
        try:
            vx, vy = float(observer.get("x")), float(observer.get("y"))
        except Exception:
            return {}
        dx, dy = vx - ox, vy - oy
        norm = math.hypot(dx, dy)
        if norm < 0.08:
            return {}
        extent = max(float(data.get("extent_x") or 0.3), float(data.get("extent_y") or 0.3))
        standoff = max(self.default_standoff_m, 0.5 * extent + 0.55)
        standoff = min(standoff, max(0.65, norm))
        gx, gy = ox + dx / norm * standoff, oy + dy / norm * standoff
        yaw = math.atan2(oy - gy, ox - gx)
        return {
            "frame_id": "map",
            "x": gx, "y": gy, "z": 0.0,
            "qx": 0.0, "qy": 0.0,
            "qz": math.sin(0.5 * yaw), "qw": math.cos(0.5 * yaw),
            "yaw": yaw,
            "source": "derived_from_object_pose_and_observer",
            "standoff_m": standoff,
            "object_x": ox, "object_y": oy,
        }

    def count_unique(self, label: str, room: str = "") -> Dict[str, Any]:
        result = self.search(label, room, limit=300)
        matches = result.get("matches") or []
        n = len(matches)
        noun = label if n == 1 else (label if str(label).endswith("s") else f"{label}s")
        return {
            "success": True, "query": label, "room": room, "count": n,
            "objects": matches,
            "speech": f"I remember {n} unique confirmed {noun}."
        }

    def best(self, label: str, room: str = "") -> Dict[str, Any]:
        result = self.search(label, room, limit=5)
        if not result.get("matches"):
            return {"success": False, "speech": f"I do not have a confirmed mapped location for {label}.", "query": label}
        best = result["matches"][0]
        pose = best.get("object_pose") or {}
        room_text = f" in {best.get('room_id')}" if best.get("room_id") else ""
        speech = f"I remember {best.get('label', label)}{room_text}"
        if pose:
            speech += f" at map position x {float(pose.get('x', 0.0)):.1f}, y {float(pose.get('y', 0.0)):.1f}."
        else:
            speech += "."
        return {"success": True, "speech": speech, "object": best, "alternatives": result["matches"][1:]}

    def find_navigation_target(self, label: str, room: str = "") -> Dict[str, Any]:
        located = self.best(label, room)
        if not located.get("success"):
            return located
        obj = located["object"]
        object_pose = obj.get("object_pose") or {}
        approach = obj.get("navigation_approach_pose") or {}
        if not object_pose:
            return {"success": False, "speech": f"I remember {label}, but its physical map position is missing."}
        if not approach:
            return {
                "success": False,
                "speech": f"I remember the physical location of {label}, but I do not yet have a safe standoff navigation pose for it.",
                "object": obj,
            }
        return {
            "success": True,
            "speech": f"I found a remembered {obj.get('label', label)}. I will navigate to a safe standoff near its stored object location.",
            "object": obj,
            "object_pose": object_pose,
            "navigation_pose": approach,
        }

    def hint_for_utterance(self, text: str, limit: int = 5) -> Dict[str, Any]:
        # Cheap lexical retrieval before the LLM call. This reduces prompt size and
        # gives the planner grounded candidates without dumping the full object DB.
        tokens = [t for t in _norm(text).split() if len(t) > 2]
        if not tokens:
            return {"matches": []}
        all_rows = self._all(limit=300)
        ranked: List[Tuple[float, Dict[str, Any]]] = []
        for record in all_rows:
            data = _record_data(record)
            label = str(data.get("label") or "")
            best = max((self._match_score(tok, record) for tok in tokens), default=0.0)
            if best > 0:
                ranked.append((best, record))
        ranked.sort(key=lambda x: x[0], reverse=True)
        matches = []
        seen = set()
        for _, rec in ranked:
            data = _record_data(rec)
            oid = rec.get("id") or data.get("object_id")
            if oid in seen:
                continue
            seen.add(oid)
            matches.append({
                "id": oid,
                "label": data.get("label"),
                "room_id": data.get("room_id"),
                "object_pose": _pose(data),
                "confidence": _confidence(data),
                "confirmations": int(data.get("confirmations", 0) or 0),
            })
            if len(matches) >= limit:
                break
        return {"matches": matches}
