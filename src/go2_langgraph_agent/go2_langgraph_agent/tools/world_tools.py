from __future__ import annotations

import re
import time
from typing import Any, Dict, Optional

from .gemini_web_search import GeminiGroundedSearch
from .ollama_reasoner import OllamaGroundedReasoner


def _norm(text: str) -> str:
    return " ".join(re.sub(r"[^a-z0-9\s]", " ", (text or "").lower()).split())


def _strip_room_suffix(value: str) -> str:
    value = re.sub(r"\b(in|inside|within) (this|the|current) (room|area|lab)\b.*$", "", value).strip()
    return value.strip(" .?!,:;")

# SPARKY_FUSED_OBJECT_QUERY_V13_2
def _clean_object_query(value: str) -> tuple[str, bool]:
    value = _norm(value)
    nearest = bool(re.search(r"\b(nearest|nearerst|closest|closeest)\b", value))
    value = re.sub(r"^(hey\s+)?sparky\s+", "", value)
    value = re.sub(r"^(please\s+)?(can you\s+|could you\s+|would you\s+)?", "", value)
    value = re.sub(r"^(find|locate|search for|look for|go find)\s+", "", value)
    value = re.sub(r"\b(nearest|nearerst|closest|closeest|nearby)\b", " ", value)
    value = re.sub(r"^(the|a|an|my|our)\s+", "", value.strip())
    value = re.sub(r"\b(to me|from me|around me|near me|please|now)\b", " ", value)
    value = " ".join(value.split())
    return value or "object", nearest


def classify_world_intent(text: str, hinted_intent: str = "") -> Dict[str, Any]:
    norm = _norm(text)
    norm = re.sub(r"^(hey\s+)?sparky\s+", "", norm)
    norm = re.sub(r"^(please\s+)?(can you\s+|could you\s+|would you\s+)?", "", norm)
    hint = (hinted_intent or "").strip().lower()
    # SPARKY_VISIBLE_MEMORY_QUERY_V2
    if hint in {"count_objects", "find_object", "where_object", "visible_objects", "memory_summary", "web_search"}:
        intent = hint
    elif any(p in norm for p in ("search the web", "search web", "look up online", "lookup online", "google this", "latest online", "current web")):
        intent = "web_search"
    elif any(p in norm for p in (
        "what do you see", "what can you see", "what are you seeing",
        "what is visible", "whats visible", "what objects are visible",
        "tell me what you see", "show me what you see",
    )):
        intent = "visible_objects"
    elif any(p in norm for p in (
        "what do you remember", "what did you remember", "what did you learn",
        "what objects do you remember", "who do you remember",
        "what is in your memory", "whats in your memory", "memory summary",
        "what did you save during teaching", "what did you save in teach",
    )):
        intent = "memory_summary"
    elif norm.startswith("how many ") or norm.startswith("count ") or " how many " in f" {norm} ":
        intent = "count_objects"
    elif any(norm.startswith(p) for p in ("find ", "locate ", "search for ")):
        intent = "find_object"
    elif any(norm.startswith(p) for p in ("where is the ", "where are the ", "where is my ", "where are my ")):
        intent = "where_object"
    else:
        return {"intent": "", "object_query": "", "room": "", "requires_motion": False}

    if intent in {"visible_objects", "memory_summary", "web_search"}:
        return {"intent": intent, "object_query": "", "room": "", "requires_motion": False}

    query = norm
    if intent == "count_objects":
        query = re.sub(r"^(how many|count)\s+", "", query)
        query = re.sub(r"\b(are|is|do we have|can you see|present|there)\b", " ", query)
    elif intent == "find_object":
        query = re.sub(r"^(find|locate|search for)\s+", "", query)
    elif intent == "where_object":
        query = re.sub(r"^where (is|are) (the|my)?\s*", "", query)
    room = ""
    room_match = re.search(r"\b(?:in|inside|within)\s+(?:the\s+)?(.+?)(?:\s+room)?$", query)
    if room_match:
        room_name = room_match.group(1).strip()
        if room_name not in {"this", "current", "here"}:
            room = room_name
        query = query[: room_match.start()].strip()
    query = _strip_room_suffix(query)
    query = re.sub(r"\b(this room|the room|current room|here)\b", "", query).strip()
    query, nearest = _clean_object_query(query)
    return {
        "intent": intent,
        "object_query": query or "object",
        "room": room,
        "nearest": nearest,
        "requires_motion": intent == "find_object",
    }


class WorldAgentTools:
    def __init__(
        self,
        memory_tools: Any,
        enable_web_search: bool = False,
        web_model: str = "gemini-2.5-flash",
        enable_ollama_reasoner: bool = True,
        ollama_model: str = "llama3.2:3b",
    ) -> None:
        self.memory = memory_tools
        self.enable_web_search = bool(enable_web_search)
        self.web = GeminiGroundedSearch(model=web_model)
        self.reasoner = OllamaGroundedReasoner(model=ollama_model) if enable_ollama_reasoner else None

    @staticmethod
    def _object_data(record: Dict[str, Any]) -> Dict[str, Any]:
        return (record or {}).get("data") or {}

    @staticmethod
    def _format_counts(counts: Dict[str, Any], limit: int = 8) -> str:
        rows = []
        for key, value in sorted((counts or {}).items(), key=lambda kv: (-int(kv[1] or 0), str(kv[0]))):
            n = int(value or 0)
            if n > 0:
                rows.append(f"{n} {key}")
        if not rows:
            return "none"
        suffix = "" if len(rows) <= limit else f", plus {len(rows) - limit} more classes"
        return ", ".join(rows[:limit]) + suffix

    def visible_objects(self, visible_inventory: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        inventory = visible_inventory or {}
        stamp = float(inventory.get("stamp_sec", 0.0) or 0.0)
        age = time.time() - stamp if stamp > 0 else 1e9
        counts = inventory.get("visible_counts") or {}
        total = int(inventory.get("visible_count_total", sum(int(v or 0) for v in counts.values())) or 0)
        if age > 4.0:
            return {"success": False, "speech": "I do not have a fresh live YOLO and SAM view right now. My stored object memory is still available.", "visible_count": 0, "visible_counts": {}, "age_sec": age}
        if total <= 0:
            return {"success": True, "speech": "My live YOLO and SAM view is active, but I do not currently have any tracked objects to report.", "visible_count": 0, "visible_counts": counts, "age_sec": age}
        return {"success": True, "speech": f"I can currently see {total} tracked objects: {self._format_counts(counts)}.", "visible_count": total, "visible_counts": counts, "age_sec": age}

    def memory_summary(self) -> Dict[str, Any]:
        counts_result = self.memory.count_objects(label="", room="", confirmed_only=True)
        object_counts = counts_result.get("counts") or {}
        object_total = int(counts_result.get("count", sum(int(v or 0) for v in object_counts.values())) or 0)
        result = self.memory.query("", limit=200)
        places = result.get("places") or []
        checkpoints = result.get("checkpoints") or []
        vlm = result.get("vlm_checkpoints") or []
        world = result.get("world_memory") or result
        tour_stops = world.get("tour_stops") or []
        place_count = len({str(r.get("id") or i) for i, r in enumerate(places)})
        checkpoint_count = len({str(r.get("id") or i) for i, r in enumerate(checkpoints)})
        vlm_count = len({str(r.get("id") or i) for i, r in enumerate(vlm)})
        tour_count = len({str(r.get("id") or i) for i, r in enumerate(tour_stops)})
        parts = [f"I remember {object_total} confirmed mapped objects"]
        if object_total:
            parts[-1] += f": {self._format_counts(object_counts)}"
        parts += [f"{place_count} saved semantic places", f"{checkpoint_count} background checkpoints", f"{vlm_count} VLM checkpoints", f"{tour_count} unified-memory tour stops"]
        return {"success": True, "speech": ". ".join(parts) + ".", "remembered_count": object_total, "remembered_counts": object_counts, "place_count": place_count, "checkpoint_count": checkpoint_count, "vlm_checkpoint_count": vlm_count, "tour_stop_count": tour_count}

    def count_objects(self, label: str, room: str = "", visible_inventory: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        result = self.memory.count_objects(label=label, room=room, confirmed_only=True)
        remembered = int(result.get("count", 0) or 0)
        visible_count = None
        visible_inventory = visible_inventory or {}
        visible_counts = visible_inventory.get("visible_counts") or {}
        target = _norm(label)
        if target:
            target_singular = target[:-1] if target.endswith("s") else target
            total = 0
            matched = False
            for key, value in visible_counts.items():
                key_norm = _norm(str(key))
                if target in key_norm or target_singular in key_norm or key_norm in {target, target_singular}:
                    total += int(value or 0)
                    matched = True
            if matched:
                visible_count = total
        where = f" in {room}" if room else ""
        if visible_count is None:
            speech = f"I remember {remembered} confirmed {label}{where}. I do not have a matching live visible count right now."
        else:
            speech = f"I remember {remembered} confirmed {label}{where}, and I can currently see {visible_count}."
        return {"success": True, "speech": speech, "remembered_count": remembered, "visible_count": visible_count, "result": result}

    def where_object(self, label: str, room: str = "", robot_map_pose=None, nearest: bool = False) -> Dict[str, Any]:
        label, inferred_nearest = _clean_object_query(label)
        nearest = bool(nearest or inferred_nearest)
        result = self.memory.query_objects(
            label=label, room=room, confirmed_only=True, limit=20,
            robot_map_pose=robot_map_pose, nearest=nearest,
        )
        objects = result.get("objects") or []
        if not objects:
            db = (result.get("mapper_sql") or {}).get("db_path") or "mapper/unified memory"
            return {"success": False, "speech": f"I do not have a confirmed remembered location for {label}. I checked {db}.", "objects": [], "memory_result": result}
        obj = objects[0]
        data = self._object_data(obj)
        pose = data.get("map_pose") or {}
        room_id = data.get("room_id") or "the mapped area"
        source = data.get("memory_source") or "unified_memory"
        distance = data.get("distance_from_robot_m")
        qualifier = "nearest remembered " if nearest else "remembered "
        speech = f"I found the {qualifier}{label} in {room_id}"
        if distance is not None:
            speech += f", about {float(distance):.1f} meters from my current map pose"
        if pose:
            speech += f", near map x {float(pose.get('x', 0.0)):.1f}, y {float(pose.get('y', 0.0)):.1f}"
        speech += f". Memory source: {source}."
        return {"success": True, "speech": speech, "object": obj, "objects": objects, "memory_result": result}

    def find_object(self, label: str, room: str = "", robot_map_pose=None, nearest: bool = False) -> Dict[str, Any]:
        label, inferred_nearest = _clean_object_query(label)
        nearest = bool(nearest or inferred_nearest)
        located = self.where_object(label, room, robot_map_pose=robot_map_pose, nearest=nearest)
        if not located.get("success"):
            return located
        obj = located.get("object") or {}
        data = self._object_data(obj)
        approach = data.get("approach_pose") or {}
        if not approach:
            return {
                "success": False,
                "speech": f"I found the remembered {label}, but it has no safe observation viewpoint in memory yet, so I cannot create a Nav2 goal for it.",
                "object": obj,
            }
        nav = {
            "action": "navigate_to_pose",
            "pose": approach,
            "reason": "nearest_memory_first_object_search" if nearest else "memory_first_object_search",
            "object_id": obj.get("id"),
            "object_label": data.get("label") or label,
            "verify_live_after_arrival": True,
        }
        distance = data.get("distance_from_robot_m")
        distance_text = f" about {float(distance):.1f} meters away" if distance is not None else ""
        return {
            "success": True,
            "speech": f"I found the {'nearest ' if nearest else ''}confirmed remembered {label}{distance_text}. I will navigate to its last safe observation viewpoint and verify it live.",
            "object": obj,
            "nav_command": nav,
        }

    def web_search(self, query: str) -> Dict[str, Any]:
        if not self.enable_web_search:
            return {"success": False, "speech": "Web search is disabled on this run.", "sources": []}
        result = self.web.search(query)
        if result.get("success"):
            self.memory.cache_web_result({
                "query": query,
                "answer": result.get("answer", ""),
                "sources": result.get("sources", []),
                "model": result.get("model"),
                "grounded": result.get("grounded", False),
                "source_type": "gemini_google_search",
            })
            return {
                "success": True,
                "speech": result.get("answer", ""),
                "sources": result.get("sources", []),
                "web_result": result,
            }
        return {"success": False, "speech": f"I could not complete the web search: {result.get('error', 'unknown error')}", "sources": []}
