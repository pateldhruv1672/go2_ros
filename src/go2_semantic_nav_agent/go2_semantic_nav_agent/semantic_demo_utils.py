from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml
from geometry_msgs.msg import PoseStamped


def normalize_name(text: str) -> str:
    text = (text or "").strip().lower()
    text = re.sub(r"[^a-z0-9]+", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class SemanticPose:
    name: str
    aliases: List[str]
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    description: str = ""
    source: str = "unknown"

    @property
    def canonical(self) -> str:
        return normalize_name(self.name)

    def to_pose_stamped(self, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id or "map"
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(float(self.yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg


def read_yaml(path: str) -> Any:
    if not path or not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def latest_session_name(session_root: str) -> Optional[str]:
    root = os.path.expanduser(session_root)
    if not os.path.isdir(root):
        return None
    candidates = []
    for name in os.listdir(root):
        path = os.path.join(root, name)
        if not os.path.isdir(path):
            continue
        if os.path.isfile(os.path.join(path, "session.yaml")):
            candidates.append((os.path.getmtime(path), name))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][1]


def resolve_session_dir(session_root: str, session_name: str) -> str:
    root = os.path.expanduser(session_root)
    name = (session_name or "").strip()
    if not name or name.lower() in ("latest", "auto", "latest_usable"):
        latest = latest_session_name(root)
        if latest:
            name = latest
    if not name:
        name = "default"
    return os.path.join(root, name)


def _extract_num(obj: Dict[str, Any], keys: Iterable[str]) -> Optional[float]:
    for key in keys:
        if key in obj and obj[key] is not None:
            try:
                return float(obj[key])
            except (TypeError, ValueError):
                pass
    return None


def _pose_from_mapping(name: str, obj: Dict[str, Any], source: str) -> Optional[SemanticPose]:
    if not isinstance(obj, dict):
        return None

    # Common flat forms: {x, y, yaw}, {pose_x, pose_y}, etc.
    x = _extract_num(obj, ("x", "pose_x", "map_x"))
    y = _extract_num(obj, ("y", "pose_y", "map_y"))
    yaw = _extract_num(obj, ("yaw", "theta", "map_yaw"))

    # Nested pose forms.
    pose_obj = obj.get("pose") or obj.get("map_pose") or obj.get("position")
    if isinstance(pose_obj, dict):
        x = x if x is not None else _extract_num(pose_obj, ("x", "pose_x", "map_x"))
        y = y if y is not None else _extract_num(pose_obj, ("y", "pose_y", "map_y"))
        yaw = yaw if yaw is not None else _extract_num(pose_obj, ("yaw", "theta", "map_yaw"))
        orientation = pose_obj.get("orientation")
        if yaw is None and isinstance(orientation, dict):
            qx = _extract_num(orientation, ("x", "qx")) or 0.0
            qy = _extract_num(orientation, ("y", "qy")) or 0.0
            qz = _extract_num(orientation, ("z", "qz")) or 0.0
            qw = _extract_num(orientation, ("w", "qw")) or 1.0
            yaw = quat_to_yaw(qx, qy, qz, qw)

    # ROS-like form: pose: {position: {x,y}, orientation: {z,w}}
    if isinstance(pose_obj, dict) and isinstance(pose_obj.get("position"), dict):
        pos = pose_obj["position"]
        x = x if x is not None else _extract_num(pos, ("x",))
        y = y if y is not None else _extract_num(pos, ("y",))
        orientation = pose_obj.get("orientation")
        if yaw is None and isinstance(orientation, dict):
            qx = _extract_num(orientation, ("x", "qx")) or 0.0
            qy = _extract_num(orientation, ("y", "qy")) or 0.0
            qz = _extract_num(orientation, ("z", "qz")) or 0.0
            qw = _extract_num(orientation, ("w", "qw")) or 1.0
            yaw = quat_to_yaw(qx, qy, qz, qw)

    if x is None or y is None:
        return None

    resolved_name = str(obj.get("name") or obj.get("label") or obj.get("place_name") or name or "place")
    aliases_raw = obj.get("aliases", [])
    if isinstance(aliases_raw, str):
        aliases = [aliases_raw]
    elif isinstance(aliases_raw, list):
        aliases = [str(a) for a in aliases_raw]
    else:
        aliases = []
    if resolved_name not in aliases:
        aliases.insert(0, resolved_name)

    frame_id = str(obj.get("frame_id") or obj.get("frame") or "map")
    description = str(obj.get("description") or obj.get("summary") or obj.get("vlm_summary") or "")
    return SemanticPose(
        name=resolved_name,
        aliases=aliases,
        x=float(x),
        y=float(y),
        yaw=float(yaw or 0.0),
        frame_id=frame_id,
        description=description,
        source=source,
    )


def _iter_place_objects(data: Any) -> Iterable[Tuple[str, Dict[str, Any]]]:
    if data is None:
        return []
    if isinstance(data, list):
        return [(str(i), item) for i, item in enumerate(data) if isinstance(item, dict)]
    if isinstance(data, dict):
        for key in ("places", "locations", "checkpoints", "saved_places"):
            value = data.get(key)
            if isinstance(value, list):
                return [(str(i), item) for i, item in enumerate(value) if isinstance(item, dict)]
            if isinstance(value, dict):
                return [(str(k), v) for k, v in value.items() if isinstance(v, dict)]
        # Mapping of name -> record.
        if all(isinstance(v, dict) for v in data.values()):
            return [(str(k), v) for k, v in data.items()]
        # Single record.
        return [(str(data.get("name", "place")), data)]
    return []


def load_semantic_poses(session_root: str, session_name: str) -> Tuple[str, List[SemanticPose]]:
    session_dir = resolve_session_dir(session_root, session_name)
    poses: List[SemanticPose] = []

    spawn_path = os.path.join(session_dir, "spawn.yaml")
    spawn = _pose_from_mapping("spawn", read_yaml(spawn_path) or {}, "spawn.yaml")
    if spawn is not None:
        spawn.name = "spawn"
        if "spawn" not in spawn.aliases:
            spawn.aliases.insert(0, "spawn")
        poses.append(spawn)

    places_path = os.path.join(session_dir, "places.yaml")
    for key, obj in _iter_place_objects(read_yaml(places_path)):
        pose = _pose_from_mapping(key, obj, "places.yaml")
        if pose is not None:
            poses.append(pose)

    # Some older sessions store useful positions in session.yaml.
    session_path = os.path.join(session_dir, "session.yaml")
    for key, obj in _iter_place_objects(read_yaml(session_path)):
        pose = _pose_from_mapping(key, obj, "session.yaml")
        if pose is not None and pose.canonical not in {p.canonical for p in poses}:
            poses.append(pose)

    return session_dir, poses


def best_pose_match(command: str, poses: List[SemanticPose]) -> Optional[SemanticPose]:
    query = normalize_name(command)
    if not query:
        return None

    # Strip command words but keep destination words.
    removable = [
        "sparky", "please", "go to", "navigate to", "take me to", "return to",
        "move to", "drive to", "the", "a", "an", "location", "place",
    ]
    stripped = f" {query} "
    for word in removable:
        stripped = stripped.replace(f" {word} ", " ")
    query2 = normalize_name(stripped)

    candidates = []
    for pose in poses:
        names = [pose.name] + list(pose.aliases)
        score = 0
        for name in names:
            n = normalize_name(name)
            if not n:
                continue
            if query == n or query2 == n:
                score = max(score, 1000)
            if n in query or n in query2:
                score = max(score, 500 + len(n))
            n_tokens = set(n.split())
            q_tokens = set(query2.split() or query.split())
            if n_tokens and q_tokens:
                overlap = len(n_tokens & q_tokens)
                score = max(score, overlap * 10 + len(n_tokens & q_tokens))
        if score > 0:
            candidates.append((score, pose))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]
