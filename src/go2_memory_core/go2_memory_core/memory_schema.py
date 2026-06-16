from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional
import json
import math
import uuid


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_json_payload(payload: str | Dict[str, Any] | None) -> Dict[str, Any]:
    if payload is None or payload == "":
        return {}
    if isinstance(payload, dict):
        return dict(payload)
    try:
        value = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON payload: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("Expected JSON object payload")
    return value


@dataclass
class PoseRecord:
    frame_id: str = "map"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @property
    def yaw(self) -> float:
        return yaw_from_quaternion(self.qx, self.qy, self.qz, self.qw)


@dataclass
class VelocityRecord:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


@dataclass
class ConfidenceRecord:
    map_pose_confidence: float = 0.0
    odom_confidence: float = 0.0
    localization_confidence: float = 0.0
    perception_confidence: float = 0.0
    memory_confidence: float = 0.0


@dataclass
class MemoryRecord:
    id: str
    type: str
    session_name: str
    layer: str = "temporary"
    timestamp: str = field(default_factory=utc_now)
    confidence: Dict[str, Any] = field(default_factory=dict)
    source: List[str] = field(default_factory=list)
    data: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GraphNode:
    id: str
    type: str
    session_name: str
    label: str = ""
    layer: str = "temporary"
    map_pose: Optional[Dict[str, Any]] = None
    properties: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=utc_now)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GraphEdge:
    id: str
    type: str
    session_name: str
    from_id: str
    to_id: str
    properties: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=utc_now)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def pose_from_dict(data: Dict[str, Any], default_frame: str = "map") -> Dict[str, Any]:
    if "pose" in data and isinstance(data["pose"], dict):
        data = data["pose"]
    return {
        "frame_id": data.get("frame_id", default_frame),
        "x": float(data.get("x", 0.0)),
        "y": float(data.get("y", 0.0)),
        "z": float(data.get("z", 0.0)),
        "qx": float(data.get("qx", 0.0)),
        "qy": float(data.get("qy", 0.0)),
        "qz": float(data.get("qz", 0.0)),
        "qw": float(data.get("qw", 1.0)),
    }
