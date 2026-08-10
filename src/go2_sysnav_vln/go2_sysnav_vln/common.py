from __future__ import annotations

import hashlib
import json
import math
import re
from collections import deque
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def clean_label(value: str) -> str:
    text = (value or "").lower().strip()
    text = re.sub(r"[^a-z0-9\s_-]", " ", text)
    text = re.sub(r"\s+", " ", text).strip().replace(" ", "_")
    aliases = {
        "tv_monitor": "tv",
        "television": "tv",
        "trashcan": "trash_can",
        "garbage_can": "trash_can",
        "waste_bin": "trash_can",
        "microwave_oven": "microwave",
        "refrigerator": "fridge",
        "couch": "sofa",
        "potted_plant": "plant",
    }
    return aliases.get(text, text)


def target_from_text(value: str) -> str:
    raw = (value or "").strip()
    if raw.startswith("{"):
        try:
            payload = json.loads(raw)
            for key in ("target", "target_object", "object", "label", "query", "text", "instruction"):
                if isinstance(payload.get(key), str) and payload[key].strip():
                    raw = payload[key]
                    break
        except Exception:
            pass
    text = raw.lower().strip()
    for prefix in (
        "explore and find ", "find the ", "find a ", "find an ", "find ",
        "locate the ", "locate a ", "locate ", "search for the ", "search for ",
        "where is the ", "where is ",
    ):
        if text.startswith(prefix):
            text = text[len(prefix):]
            break
    # Strip common location/spatial tails only for heuristic fallback.
    text = re.split(r"\b(?:in|inside|near|next to|beside|on|under|behind|by)\b", text, maxsplit=1)[0]
    return clean_label(text)


def labels_match(a: str, b: str) -> bool:
    aa, bb = clean_label(a), clean_label(b)
    if not aa or not bb:
        return False
    if aa == bb:
        return True
    singular_a = aa[:-1] if aa.endswith("s") else aa
    singular_b = bb[:-1] if bb.endswith("s") else bb
    return singular_a == singular_b or singular_a in singular_b or singular_b in singular_a


def angle_wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quat_to_yaw(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def finite_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except Exception:
        return default


def median(values: Iterable[float]) -> Optional[float]:
    good = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not good:
        return None
    mid = len(good) // 2
    return good[mid] if len(good) % 2 else 0.5 * (good[mid - 1] + good[mid])


def stable_id(*parts: Any) -> str:
    raw = "|".join(str(p) for p in parts).encode("utf-8")
    return hashlib.sha1(raw).hexdigest()[:12]


def extract_detections(raw: str) -> List[Dict[str, Any]]:
    try:
        payload = json.loads(raw)
    except Exception:
        return []
    if isinstance(payload, list):
        return [x for x in payload if isinstance(x, dict)]
    if not isinstance(payload, dict):
        return []
    for key in ("detections", "objects", "items", "results"):
        if isinstance(payload.get(key), list):
            return [x for x in payload[key] if isinstance(x, dict)]
    return [payload]


def bbox_iou_2d(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    if len(a) < 4 or len(b) < 4:
        return 0.0, 0.0, 0.0
    ax0, ay0, ax1, ay1 = map(float, a[:4])
    bx0, by0, bx1, by1 = map(float, b[:4])
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return (
        inter / union if union > 1e-9 else 0.0,
        inter / area_a if area_a > 1e-9 else 0.0,
        inter / area_b if area_b > 1e-9 else 0.0,
    )


def object_relative_view(
    object_xy: Tuple[float, float], robot_pose: Tuple[float, float, float]
) -> Tuple[float, float]:
    dx = float(object_xy[0]) - float(robot_pose[0])
    dy = float(object_xy[1]) - float(robot_pose[1])
    return math.atan2(dy, dx), math.hypot(dx, dy)


def viewpoint_is_novel(
    object_xy: Tuple[float, float],
    robot_pose: Tuple[float, float, float],
    prior_robot_poses: Sequence[Tuple[float, float, float]],
    angle_threshold_rad: float,
    range_threshold_m: float,
    translation_threshold_m: float = 0.0,
    yaw_threshold_rad: float = 0.0,
) -> bool:
    """SysNav-style object-relative novelty plus a physical displacement gate.

    A view is redundant only when it is close to an existing observation in
    object-relative bearing and range. The additional global gate prevents two
    detector backends at one robot pose from creating independent evidence.
    """
    if not prior_robot_poses:
        return True
    new_angle, new_range = object_relative_view(object_xy, robot_pose)
    for old in prior_robot_poses:
        old_angle, old_range = object_relative_view(object_xy, old)
        relative_similar = (
            abs(angle_wrap(new_angle - old_angle)) < angle_threshold_rad
            and abs(new_range - old_range) < range_threshold_m
        )
        physical_similar = (
            math.hypot(robot_pose[0] - old[0], robot_pose[1] - old[1]) < translation_threshold_m
            and abs(angle_wrap(robot_pose[2] - old[2])) < yaw_threshold_rad
        )
        if relative_similar or physical_similar:
            return False
    return True


def nearest_true(mask: Any, start: Tuple[int, int], max_radius: int = 40) -> Optional[Tuple[int, int]]:
    h, w = mask.shape[:2]
    sx, sy = start
    if 0 <= sx < w and 0 <= sy < h and bool(mask[sy, sx]):
        return sx, sy
    q = deque([(sx, sy, 0)])
    seen = {(sx, sy)}
    while q:
        x, y, d = q.popleft()
        if d >= max_radius:
            continue
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if (nx, ny) in seen or not (0 <= nx < w and 0 <= ny < h):
                continue
            if bool(mask[ny, nx]):
                return nx, ny
            seen.add((nx, ny))
            q.append((nx, ny, d + 1))
    return None
