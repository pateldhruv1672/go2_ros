from __future__ import annotations

import math
import os
import re
from typing import Any, Dict, Iterable, List, Tuple


def _norm(value: Any) -> str:
    return " ".join(re.sub(r"[^a-z0-9 ]+", " ", str(value or "").lower()).split())


_ALIAS_GROUPS = (
    {"tv", "monitor", "screen", "display"},
    {"couch", "sofa"},
    {"cell phone", "phone", "smartphone"},
    {"dining table", "table", "desk"},
    {"robot arm", "robotic arm", "manipulator"},
)


def canonical_label(value: Any) -> str:
    value = _norm(value)
    for group in _ALIAS_GROUPS:
        ng = {_norm(x) for x in group}
        if value in ng:
            return sorted(ng)[0]
    if value.endswith("ies") and len(value) > 4:
        return value[:-3] + "y"
    if value.endswith("s") and len(value) > 3:
        return value[:-1]
    return value or "object"


def record_data(record: Dict[str, Any]) -> Dict[str, Any]:
    data = record.get("data") if isinstance(record, dict) else None
    return data if isinstance(data, dict) else (record if isinstance(record, dict) else {})


def pose_of(record: Dict[str, Any]) -> Dict[str, Any]:
    data = record_data(record)
    for key in ("object_pose", "map_pose", "position_map", "pose"):
        p = data.get(key)
        if isinstance(p, dict) and p.get("x") is not None and p.get("y") is not None:
            return p
    return {}


def confidence_of(record: Dict[str, Any]) -> float:
    d = record_data(record)
    vals: List[float] = []
    for key in ("mapper_confidence", "confidence_score"):
        try: vals.append(float(d.get(key)))
        except Exception: pass
    c = d.get("confidence")
    if isinstance(c, dict):
        for key in ("perception_confidence", "memory_confidence"):
            try: vals.append(float(c.get(key)))
            except Exception: pass
    else:
        try: vals.append(float(c))
        except Exception: pass
    return max(vals or [0.0])


def quality(record: Dict[str, Any]) -> float:
    d = record_data(record)
    conf = confidence_of(record)
    confirmations = int(d.get("confirmations", 0) or 0)
    observations = int(d.get("observations", 0) or 0)
    variance = float(d.get("variance_m2", 0.0) or 0.0)
    return 2.0 * conf + min(1.5, confirmations * 0.15) + min(0.8, observations * 0.03) - min(1.0, variance * 0.35)


def _extent_radius(record: Dict[str, Any]) -> float:
    d = record_data(record)
    vals = []
    for key in ("extent_x", "extent_y"):
        try:
            v = float(d.get(key) or 0.0)
            if v > 0: vals.append(v)
        except Exception: pass
    return max(vals or [0.30])


def merge_radius(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    # Conservative on purpose: two adjacent chairs should remain two chairs.
    base = float(os.getenv("GO2_OBJECT_DEDUP_BASE_M", "0.24"))
    max_r = float(os.getenv("GO2_OBJECT_DEDUP_MAX_M", "0.60"))
    extent = max(_extent_radius(a), _extent_radius(b))
    return min(max_r, max(base, base + 0.18 * min(extent, 1.8)))


def _distance(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    pa, pb = pose_of(a), pose_of(b)
    if not pa or not pb:
        return float("inf")
    try:
        return math.hypot(float(pa["x"]) - float(pb["x"]), float(pa["y"]) - float(pb["y"]))
    except Exception:
        return float("inf")


def _same_room(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
    da, db = record_data(a), record_data(b)
    ra = _norm(da.get("room_id") or da.get("room") or "")
    rb = _norm(db.get("room_id") or db.get("room") or "")
    return not ra or not rb or ra == rb


def _merge_cluster(members: List[Dict[str, Any]]) -> Dict[str, Any]:
    best = max(members, key=quality)
    out = dict(best)
    bd = dict(record_data(best))
    if out.get("data") is not None:
        out["data"] = bd
    else:
        out = bd
        bd = out

    weighted: List[Tuple[float, Dict[str, Any]]] = []
    for rec in members:
        p = pose_of(rec)
        if not p: continue
        d = record_data(rec)
        w = max(0.25, confidence_of(rec)) * max(1.0, float(d.get("confirmations", 0) or 0))
        weighted.append((w, p))
    if weighted:
        sw = sum(w for w, _ in weighted)
        x = sum(w * float(p["x"]) for w, p in weighted) / sw
        y = sum(w * float(p["y"]) for w, p in weighted) / sw
        zs = [(w, float(p.get("z", 0.0) or 0.0)) for w, p in weighted]
        z = sum(w * zv for w, zv in zs) / sw
        fused = {"frame_id": "map", "x": x, "y": y, "z": z, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        bd["object_pose"] = fused
        bd["map_pose"] = fused

    member_ids, source_ids = [], []
    for rec in members:
        d = record_data(rec)
        oid = rec.get("id") if isinstance(rec, dict) else None
        oid = oid or d.get("object_id")
        if oid is not None: member_ids.append(str(oid))
        sid = d.get("source_object_id")
        if sid is not None: source_ids.append(str(sid))
        for sid in d.get("source_object_ids") or []:
            source_ids.append(str(sid))
    bd["dedup_member_ids"] = sorted(set(member_ids))
    bd["source_object_ids"] = sorted(set(source_ids))
    bd["dedup_size"] = len(members)
    bd["deduplication"] = "same canonical class + conservative map-space clustering"
    bd["mapper_confidence"] = max(confidence_of(r) for r in members)
    bd["confirmations"] = max(int(record_data(r).get("confirmations", 0) or 0) for r in members)
    bd["observations"] = max(int(record_data(r).get("observations", 0) or 0) for r in members)
    for k in ("extent_x", "extent_y", "extent_z"):
        vals=[]
        for r in members:
            try:
                v=float(record_data(r).get(k) or 0.0)
                if v > 0: vals.append(v)
            except Exception: pass
        if vals: bd[k]=max(vals)
    return out


def deduplicate_records(records: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows = [r for r in records if isinstance(r, dict)]
    rows.sort(key=quality, reverse=True)
    clusters: List[List[Dict[str, Any]]] = []
    ambiguity = float(os.getenv("GO2_OBJECT_DEDUP_AMBIGUITY_M", "0.10"))

    for rec in rows:
        d = record_data(rec)
        label = canonical_label(d.get("label") or d.get("class_name") or d.get("name"))
        p = pose_of(rec)
        if not p:
            clusters.append([rec])
            continue
        candidates: List[Tuple[float, int]] = []
        for i, cluster in enumerate(clusters):
            rep = cluster[0]
            rd = record_data(rep)
            if canonical_label(rd.get("label") or rd.get("class_name") or rd.get("name")) != label:
                continue
            if not _same_room(rec, rep):
                continue
            dist = _distance(rec, rep)
            if dist <= merge_radius(rec, rep):
                candidates.append((dist, i))
        candidates.sort()
        # If two nearby canonical objects are almost equally plausible, do NOT merge.
        # This is important for rows of chairs / monitors.
        if not candidates or (len(candidates) > 1 and candidates[1][0] - candidates[0][0] < ambiguity):
            clusters.append([rec])
        else:
            clusters[candidates[0][1]].append(rec)

    merged = [_merge_cluster(c) for c in clusters]
    merged.sort(key=quality, reverse=True)
    return merged
