from __future__ import annotations

from collections import Counter
from typing import Any, Dict, Iterable, List

from .ollama_reasoner import OllamaGroundedReasoner


def _data(record: Dict[str, Any]) -> Dict[str, Any]:
    return (record or {}).get("data") or {}


def _match_name(record: Dict[str, Any], stop_name: str) -> bool:
    if not stop_name:
        return False
    d = _data(record)
    names = [str(d.get("name") or d.get("label") or record.get("id") or "")]
    names.extend(str(x) for x in d.get("aliases", []) or [])
    target = stop_name.lower().strip()
    return any(target == n.lower().strip() or target in n.lower() for n in names if n)


def _verified_facts(records: Iterable[Dict[str, Any]], subject_ids: set[str]) -> List[str]:
    facts = []
    for record in records or []:
        d = _data(record)
        if subject_ids and str(d.get("subject_id") or "") not in subject_ids:
            continue
        if not bool(d.get("verified")) and str(d.get("source_type") or "") not in {"user_verified", "stored_department"}:
            continue
        text = str(d.get("text") or "").strip()
        if text:
            facts.append(text)
    return facts[:5]


def grounded_tour_response(context: Dict[str, Any], question: str = "", use_ollama: bool = True) -> str:
    memory = context.get("tour_memory") or context.get("world_memory") or {}
    event = context.get("semantic_nav_event") or {}
    stop_name = str(event.get("stop_name") or event.get("place") or "").strip()
    current_stop = context.get("current_tour_stop") or {}
    if not stop_name:
        stop_name = str(_data(current_stop).get("name") or _data(current_stop).get("label") or current_stop.get("id") or "this area")

    rooms = memory.get("rooms") or []
    places = memory.get("places") or []
    tour_stops = memory.get("tour_stops") or []
    objects = memory.get("objects") or []
    facts = memory.get("facts") or []

    candidates = [r for r in rooms + places + tour_stops if _match_name(r, stop_name)]
    selected = candidates[0] if candidates else (current_stop or {})
    selected_data = _data(selected)
    subject_ids = {str(selected.get("id") or ""), str(selected_data.get("room_id") or ""), str(selected_data.get("place_id") or "")}
    subject_ids.discard("")

    descriptions = []
    for key in ("short_description", "description", "spoken_script", "narration"):
        text = str(selected_data.get(key) or "").strip()
        if text and text not in descriptions:
            descriptions.append(text)

    stored_facts = _verified_facts(facts, subject_ids)
    object_counts = Counter()
    for record in objects:
        d = _data(record)
        if not bool(d.get("confirmed")):
            continue
        if subject_ids and str(d.get("room_id") or "") not in subject_ids:
            continue
        object_counts[str(d.get("label") or "object")] += 1
    object_summary = ", ".join(f"{count} {label}" for label, count in object_counts.most_common(6))

    inventory = context.get("object_inventory") or {}
    visible_counts = inventory.get("visible_counts") or {}
    visible_summary = ", ".join(f"{count} {label}" for label, count in list(visible_counts.items())[:6])
    live = str(context.get("live_observation") or "").strip()

    pieces = [f"We are at {stop_name}."]
    if descriptions:
        pieces.append(descriptions[0])
    if stored_facts:
        pieces.append("Verified stored information: " + " ".join(stored_facts[:3]))
    if object_summary:
        pieces.append("From teach memory, I have confirmed: " + object_summary + ".")
    if visible_summary:
        pieces.append("Right now I can see: " + visible_summary + ".")
    elif live:
        pieces.append("My current camera summary is: " + live)
    if question:
        pieces.append("The guest asked: " + question)
    fallback = " ".join(pieces)

    if not use_ollama:
        return fallback
    evidence = {
        "stop": stop_name,
        "stored_description": descriptions[:2],
        "verified_facts": stored_facts,
        "remembered_object_counts": dict(object_counts),
        "visible_object_counts": visible_counts,
        "live_vlm_observation": live,
        "question": question,
    }
    return OllamaGroundedReasoner().compose(question or "Give a short grounded tour explanation.", evidence, fallback=fallback)
