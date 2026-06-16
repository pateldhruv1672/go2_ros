from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple


@dataclass
class GoalCandidate:
    goal_id: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    source: str = "unknown"
    information_gain: float = 0.0
    distance_m: float = 0.0
    traversability: float = 0.5
    semantic_score: float = 0.0
    obstacle_penalty: float = 0.0
    return_path_score: float = 0.5
    score: float = 0.0
    reasons: List[str] | None = None

    def as_pose(self) -> Dict[str, float | str]:
        return {"frame_id": self.frame_id, "x": self.x, "y": self.y, "yaw": self.yaw, "qz": math.sin(self.yaw / 2.0), "qw": math.cos(self.yaw / 2.0)}

    def to_dict(self) -> Dict[str, Any]:
        return {
            "goal_id": self.goal_id,
            "pose": self.as_pose(),
            "source": self.source,
            "information_gain": self.information_gain,
            "distance_m": self.distance_m,
            "traversability": self.traversability,
            "semantic_score": self.semantic_score,
            "obstacle_penalty": self.obstacle_penalty,
            "return_path_score": self.return_path_score,
            "score": self.score,
            "reasons": self.reasons or [],
        }


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _robot_xy(context: Dict[str, Any]) -> Tuple[float, float]:
    odom = context.get("odom") or {}
    pose = (odom.get("pose") or odom.get("map_pose") or odom.get("position") or {})
    if isinstance(pose, dict):
        return _as_float(pose.get("x"), 0.0), _as_float(pose.get("y"), 0.0)
    return 0.0, 0.0


def _context_traversability(context: Dict[str, Any]) -> float:
    trav = context.get("traversability") or context.get("traversability_summary") or {}
    if isinstance(trav, dict):
        return max(0.0, min(1.0, _as_float(trav.get("traversability_score"), 0.5)))
    return 0.5


def _front_clearance(context: Dict[str, Any]) -> float:
    scan = context.get("scan_summary") or {}
    clearances = scan.get("sector_clearance_m") or {}
    return _as_float(clearances.get("front"), 10.0)


def _dynamic_penalty(context: Dict[str, Any], x: float, y: float) -> Tuple[float, List[str]]:
    reasons: List[str] = []
    payload = context.get("dynamic_obstacles") or {}
    tracks = payload.get("tracks") if isinstance(payload, dict) else []
    penalty = 0.0
    for track in tracks or []:
        if not isinstance(track, dict):
            continue
        tx = _as_float(track.get("x"), 999.0)
        ty = _as_float(track.get("y"), 999.0)
        speed = _as_float(track.get("speed_mps"), 0.0)
        dist = math.hypot(x - tx, y - ty)
        if dist < 1.25:
            penalty = max(penalty, min(0.8, 0.55 + speed * 0.4))
            reasons.append(f"dynamic_obstacle_near_{dist:.2f}m")
    return penalty, reasons


def _semantic_score(context: Dict[str, Any], goal_text: str = "") -> Tuple[float, List[str]]:
    detections = context.get("open_vocab_detections") or {}
    labels = []
    if isinstance(detections, dict):
        for det in detections.get("detections", []) or []:
            if isinstance(det, dict):
                labels.append(str(det.get("label", "")).lower())
    vlm = str(context.get("live_observation") or context.get("vlm_summary") or "").lower()
    text = goal_text.lower()
    score = 0.0
    reasons: List[str] = []
    useful_terms = ["door", "hallway", "opening", "corridor", "room", "lab", "exit", "sign", "poster"]
    for term in useful_terms:
        if term in vlm or term in labels or term in text:
            score += 0.04
    if "door" in labels or "opening" in labels or "opening" in vlm:
        score += 0.15
        reasons.append("semantic_opening_or_door")
    if "person" in labels:
        score -= 0.15
        reasons.append("person_detected_reduce_aggression")
    return max(-0.2, min(0.35, score)), reasons


def normalize_frontier_candidates(payload: Any, context: Dict[str, Any]) -> List[GoalCandidate]:
    if isinstance(payload, dict):
        raw_candidates = payload.get("candidates") or []
    elif isinstance(payload, list):
        raw_candidates = payload
    else:
        raw_candidates = []
    rx, ry = _robot_xy(context)
    out: List[GoalCandidate] = []
    for idx, cand in enumerate(raw_candidates):
        if not isinstance(cand, dict):
            continue
        pose = cand.get("pose") or cand
        x = _as_float(pose.get("x"), None)  # type: ignore[arg-type]
        y = _as_float(pose.get("y"), None)  # type: ignore[arg-type]
        if x is None or y is None:
            continue
        dist = _as_float(cand.get("distance_m"), math.hypot(x - rx, y - ry))
        out.append(GoalCandidate(
            goal_id=str(cand.get("id") or cand.get("goal_id") or f"frontier_{idx:03d}"),
            x=x,
            y=y,
            yaw=_as_float(pose.get("yaw"), 0.0),
            frame_id=str(pose.get("frame_id") or cand.get("frame_id") or "map"),
            source=str(cand.get("source") or "frontier"),
            information_gain=_as_float(cand.get("information_gain"), _as_float(cand.get("unknown_cells"), 1.0) / 40.0),
            distance_m=dist,
            traversability=_context_traversability(context),
            return_path_score=max(0.0, min(1.0, 1.0 - dist / 18.0)),
        ))
    return out


def normalize_coverage_candidates(payload: Any, context: Dict[str, Any]) -> List[GoalCandidate]:
    waypoints = payload.get("waypoints", payload) if isinstance(payload, dict) else payload
    if not isinstance(waypoints, list):
        return []
    rx, ry = _robot_xy(context)
    out: List[GoalCandidate] = []
    for idx, pose in enumerate(waypoints):
        if not isinstance(pose, dict):
            continue
        x = _as_float(pose.get("x"), None)  # type: ignore[arg-type]
        y = _as_float(pose.get("y"), None)  # type: ignore[arg-type]
        if x is None or y is None:
            continue
        out.append(GoalCandidate(
            goal_id=str(pose.get("goal_id") or f"coverage_{idx:03d}"),
            x=x,
            y=y,
            yaw=_as_float(pose.get("yaw"), 0.0),
            frame_id=str(pose.get("frame_id") or "map"),
            source="coverage",
            information_gain=0.35,
            distance_m=_as_float(pose.get("distance_m"), math.hypot(x - rx, y - ry)),
            traversability=_context_traversability(context),
            return_path_score=0.8,
        ))
    return out


def score_candidate(candidate: GoalCandidate, context: Dict[str, Any], goal_text: str = "") -> GoalCandidate:
    reasons: List[str] = []
    clearance = _front_clearance(context)
    trav = candidate.traversability
    if clearance < 0.45:
        trav = min(trav, 0.2)
        reasons.append(f"front_clearance_low_{clearance:.2f}m")
    elif clearance > 1.0:
        reasons.append(f"front_clearance_ok_{clearance:.2f}m")
    semantic, semantic_reasons = _semantic_score(context, goal_text)
    obstacle_penalty, obstacle_reasons = _dynamic_penalty(context, candidate.x, candidate.y)
    reasons.extend(semantic_reasons)
    reasons.extend(obstacle_reasons)
    info = max(0.0, min(1.0, candidate.information_gain))
    dist_pref = max(0.0, min(1.0, 1.0 - abs(candidate.distance_m - 2.5) / 8.0))
    score = 0.35 * info + 0.25 * trav + 0.15 * dist_pref + 0.15 * candidate.return_path_score + semantic - obstacle_penalty
    candidate.traversability = trav
    candidate.semantic_score = semantic
    candidate.obstacle_penalty = obstacle_penalty
    candidate.score = round(max(-1.0, min(1.0, score)), 4)
    candidate.reasons = reasons
    return candidate


def choose_best_goal(candidates: Iterable[GoalCandidate], context: Dict[str, Any], goal_text: str = "") -> Optional[GoalCandidate]:
    scored = [score_candidate(c, context, goal_text) for c in candidates]
    scored = [c for c in scored if c.traversability > 0.15 and c.obstacle_penalty < 0.75]
    if not scored:
        return None
    return sorted(scored, key=lambda c: (c.score, -c.distance_m), reverse=True)[0]


def select_exploration_goal(context: Dict[str, Any], strategy: str = "frontier", goal_text: str = "") -> Dict[str, Any]:
    if strategy == "coverage":
        candidates = normalize_coverage_candidates(context.get("coverage_waypoints") or context.get("coverage_plan") or [], context)
    else:
        candidates = normalize_frontier_candidates(context.get("frontier_candidates") or {}, context)
    best = choose_best_goal(candidates, context, goal_text)
    return {
        "strategy": strategy,
        "candidate_count": len(list(candidates)),
        "best_goal": best.to_dict() if best else None,
        "scored_candidates": [score_candidate(c, context, goal_text).to_dict() for c in candidates[:12]],
    }
