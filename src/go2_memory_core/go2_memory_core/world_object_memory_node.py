from __future__ import annotations

import json
import math
import re
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backends.artifact_store_files import ArtifactStoreFiles
from .memory_api import UnifiedMemoryAPI
from .session_resolution import resolve_semantic_session_name

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None
    np = None
    CvBridge = None


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _clean_label(value: str) -> str:
    value = re.sub(r"[^a-z0-9 ]+", " ", str(value or "").lower())
    return " ".join(value.split()) or "object"


def _slug(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value or "").lower()).strip("_") or "object"


class WorldObjectMemoryNode(Node):
    """Bridges live YOLO/SAM observations + fused VLN object map into session memory.

    Raw detector tracks answer *visible now*. Persistent counts only use confirmed
    records from the pose-aware object mapper, which fuses repeated views in map frame.
    """

    def __init__(self) -> None:
        super().__init__("go2_world_object_memory")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "latest")
        self.declare_parameter("detector_topic", "/object_explorer/sam2_detections")
        self.declare_parameter("object_map_topic", "/go2_vln/object_map")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("inventory_topic", "/go2_memory/object_inventory")
        self.declare_parameter("store_raw_observations", True)
        self.declare_parameter("observation_min_period_sec", 2.0)
        self.declare_parameter("min_raw_confidence", 0.25)
        self.declare_parameter("min_persistent_object_confidence", 0.60)
        self.declare_parameter("object_dedup_base_radius_m", 0.24)
        self.declare_parameter("object_dedup_max_radius_m", 0.60)
        self.declare_parameter("object_dedup_ambiguity_margin_m", 0.10)
        self.declare_parameter("ingest_historical_mapper_objects", False)
        self.declare_parameter("historical_allowance_sec", 3.0)
        self.declare_parameter("save_object_crops", True)
        self.declare_parameter("save_mask_polygons", True)
        self.declare_parameter("enable_graph_memory", True)
        self.declare_parameter("enable_vector_memory", True)
        self.declare_parameter("enable_voxel_memory", True)

        self.session_root = str(self.get_parameter("session_root").value)
        self.requested_session = str(self.get_parameter("session_name").value)
        self.session_name = ""
        self.api: Optional[UnifiedMemoryAPI] = None
        self.artifacts = ArtifactStoreFiles(self.session_root)
        self.started_at = time.time()
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_visible: Dict[str, Any] = {"detections": [], "stamp_sec": 0.0}
        self.last_obs_write: Dict[str, float] = {}
        self.source_to_canonical: Dict[str, str] = {}
        self.bridge = CvBridge() if CvBridge is not None else None

        self.inventory_pub = self.create_publisher(String, str(self.get_parameter("inventory_topic").value), 10)
        self.status_pub = self.create_publisher(String, "/go2_memory/world_status", 10)
        self.create_subscription(String, str(self.get_parameter("detector_topic").value), self._on_detections, 10)
        self.create_subscription(String, str(self.get_parameter("object_map_topic").value), self._on_object_map, 10)
        self.create_subscription(Image, str(self.get_parameter("camera_topic").value), self._on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)
        self.create_timer(2.0, self._publish_inventory)
        self.get_logger().info(
            "World object memory bridge ready: raw detections -> observations; confirmed fused map -> ObjectInstance memory"
        )

    def _ensure_api(self) -> Optional[UnifiedMemoryAPI]:
        if self.api is not None:
            return self.api
        try:
            if self.requested_session == "__latest_created__":
                # Teach creates a new semantic session before map.yaml exists. The normal
                # latest resolver prefers resume-ready sessions, which can select an older
                # map. Bind Teach object memory to the newest directory created by
                # semantic_nav_node instead.
                root = Path(self.session_root).expanduser()
                candidates = [p for p in root.iterdir() if p.is_dir()] if root.exists() else []
                if not candidates:
                    raise RuntimeError(f"No semantic Teach session directories exist under {root}")
                resolved = max(candidates, key=lambda p: p.stat().st_mtime_ns).name
            else:
                resolved = resolve_semantic_session_name(self.session_root, self.requested_session)
            self.session_name = str(resolved)
            self.api = UnifiedMemoryAPI(
                session_root=self.session_root,
                enable_graph_memory=_as_bool(self.get_parameter("enable_graph_memory").value),
                enable_vector_memory=_as_bool(self.get_parameter("enable_vector_memory").value),
                enable_voxel_memory=_as_bool(self.get_parameter("enable_voxel_memory").value),
            )
            self.get_logger().info(f"World memory resolved session '{self.requested_session}' -> '{self.session_name}'")
            return self.api
        except Exception as exc:
            self.get_logger().warn(f"World memory session is not ready yet: {exc}")
            return None

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _odom_pose(self) -> Dict[str, Any]:
        if self.latest_odom is None:
            return {}
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        return {
            "frame_id": self.latest_odom.header.frame_id or "odom",
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        }

    @staticmethod
    def _memory_record_data(record: Dict[str, Any]) -> Dict[str, Any]:
        data = record.get("data") if isinstance(record, dict) else None
        return data if isinstance(data, dict) else (record if isinstance(record, dict) else {})

    @staticmethod
    def _memory_pose(record: Dict[str, Any]) -> Dict[str, Any]:
        data = WorldObjectMemoryNode._memory_record_data(record)
        for key in ("object_pose", "map_pose", "position_map", "pose"):
            value = data.get(key)
            if isinstance(value, dict) and value.get("x") is not None and value.get("y") is not None:
                return value
        return {}

    def _canonical_object_identity(self, api: UnifiedMemoryAPI, label: str, source_id: Any, map_pose: Dict[str, Any], extent_x: Any, extent_y: Any) -> Tuple[str, Dict[str, Any]]:
        source_key = f"{_slug(label)}:{source_id}"
        remembered = self.source_to_canonical.get(source_key)
        try:
            query = api.query_objects(self.session_name, label=label, room="", confirmed_only=True, limit=300)
            rows = query.get("objects", []) if isinstance(query, dict) else []
        except Exception:
            rows = []
        rows = [r for r in rows if isinstance(r, dict)]
        if remembered:
            for rec in rows:
                data = self._memory_record_data(rec)
                oid = str(rec.get("id") or data.get("object_id") or "")
                if oid == remembered:
                    return remembered, data
        # Exact source-id continuity wins even across mapper updates.
        for rec in rows:
            data = self._memory_record_data(rec)
            sids = {str(data.get("source_object_id"))} | {str(x) for x in (data.get("source_object_ids") or [])}
            if str(source_id) in sids:
                oid = str(rec.get("id") or data.get("object_id") or "")
                if oid:
                    self.source_to_canonical[source_key] = oid
                    return oid, data
        try:
            nx, ny = float(map_pose["x"]), float(map_pose["y"])
        except Exception:
            nx = ny = 0.0
        base = float(self.get_parameter("object_dedup_base_radius_m").value)
        max_r = float(self.get_parameter("object_dedup_max_radius_m").value)
        ambiguity = float(self.get_parameter("object_dedup_ambiguity_margin_m").value)
        try:
            new_extent = max(float(extent_x or 0.30), float(extent_y or 0.30))
        except Exception:
            new_extent = 0.30
        candidates: List[Tuple[float, str, Dict[str, Any]]] = []
        for rec in rows:
            data = self._memory_record_data(rec)
            if _clean_label(data.get("label", "object")) != _clean_label(label):
                continue
            p = self._memory_pose(rec)
            if not p:
                continue
            try:
                d = math.hypot(nx - float(p["x"]), ny - float(p["y"]))
                old_extent = max(float(data.get("extent_x") or 0.30), float(data.get("extent_y") or 0.30))
            except Exception:
                continue
            radius = min(max_r, max(base, base + 0.18 * min(max(new_extent, old_extent), 1.8)))
            if d <= radius:
                oid = str(rec.get("id") or data.get("object_id") or "")
                if oid:
                    candidates.append((d, oid, data))
        candidates.sort(key=lambda x: x[0])
        # Ambiguous between two close same-class objects: create a new identity rather than collapsing them.
        if candidates and not (len(candidates) > 1 and candidates[1][0] - candidates[0][0] < ambiguity):
            _, oid, data = candidates[0]
            self.source_to_canonical[source_key] = oid
            return oid, data
        oid = f"object_{_slug(label)}_{int(time.time() * 1000)}_{source_id}"
        self.source_to_canonical[source_key] = oid
        return oid, {}

    def _artifact_refs(self, det: Dict[str, Any], stamp: float) -> Dict[str, str]:
        if not self.session_name:
            return {}
        refs: Dict[str, str] = {}
        track = str(det.get("track_id", "na"))
        label = _slug(det.get("label", "object"))
        stem = f"{int(stamp * 1000)}_{label}_track_{track}"
        session_dir = self.artifacts.session_dir(self.session_name)

        polygon = det.get("mask_polygon")
        if _as_bool(self.get_parameter("save_mask_polygons").value) and isinstance(polygon, list) and polygon:
            rel = Path("artifacts/object_masks") / f"{stem}.json"
            path = session_dir / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({"mask_polygon": polygon, "bbox": det.get("bbox")}, sort_keys=True), encoding="utf-8")
            refs["mask_ref"] = str(path)

        if not _as_bool(self.get_parameter("save_object_crops").value) or self.latest_image is None or self.bridge is None or cv2 is None:
            return refs
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            box = det.get("bbox") or det.get("box_xyxy") or []
            if not isinstance(box, (list, tuple)) or len(box) < 4:
                return refs
            h, w = frame.shape[:2]
            x1, y1, x2, y2 = [int(round(float(v))) for v in box[:4]]
            x1, y1 = max(0, min(w - 1, x1)), max(0, min(h - 1, y1))
            x2, y2 = max(x1 + 1, min(w, x2)), max(y1 + 1, min(h, y2))
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                return refs
            rel = Path("artifacts/object_crops") / f"{stem}.jpg"
            path = session_dir / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            if cv2.imwrite(str(path), crop):
                refs["crop_ref"] = str(path)
        except Exception as exc:
            self.get_logger().debug(f"Object crop write skipped: {exc}")
        return refs

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        detections = payload.get("detections", []) if isinstance(payload, dict) else []
        if not isinstance(detections, list):
            detections = []
        stamp = float(payload.get("stamp_sec", time.time()) or time.time()) if isinstance(payload, dict) else time.time()
        cleaned = []
        for det in detections:
            if not isinstance(det, dict):
                continue
            score = float(det.get("confidence", 0.0) or 0.0)
            if score < float(self.get_parameter("min_raw_confidence").value):
                continue
            item = dict(det)
            item["label"] = _clean_label(item.get("label", "object"))
            cleaned.append(item)
        self.latest_visible = {
            "stamp_sec": stamp,
            "sam2_mode": payload.get("sam2_mode", "") if isinstance(payload, dict) else "",
            "detections": cleaned,
        }
        api = self._ensure_api()
        if api is None or not _as_bool(self.get_parameter("store_raw_observations").value):
            self._publish_inventory()
            return
        min_period = float(self.get_parameter("observation_min_period_sec").value)
        now = time.time()
        for det in cleaned:
            key = f"{det.get('label')}:{det.get('track_id', 'na')}"
            if now - self.last_obs_write.get(key, -1e9) < min_period:
                continue
            self.last_obs_write[key] = now
            refs = self._artifact_refs(det, stamp)
            api.write_object_observation(self.session_name, {
                "timestamp_sec": stamp,
                "label": det.get("label"),
                "track_id": det.get("track_id"),
                "bbox": det.get("bbox"),
                "mask_polygon": det.get("mask_polygon"),
                "detector_confidence": float(det.get("confidence", 0.0) or 0.0),
                "sam2_mode": self.latest_visible.get("sam2_mode"),
                "seen_from_odom": self._odom_pose(),
                **refs,
                "confidence": {"perception_confidence": float(det.get("confidence", 0.0) or 0.0)},
                "source": ["yolo", "sam2" if det.get("mask_polygon") else "yolo_only"],
            })
        self._publish_inventory()

    def _on_object_map(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        objects = payload.get("objects", []) if isinstance(payload, dict) else []
        if not isinstance(objects, list):
            return
        api = self._ensure_api()
        if api is None:
            return
        allowance = float(self.get_parameter("historical_allowance_sec").value)
        allow_historical = _as_bool(self.get_parameter("ingest_historical_mapper_objects").value)
        ingested = 0
        for obj in objects:
            if not isinstance(obj, dict) or not bool(obj.get("confirmed")):
                continue
            persistent_min_conf = float(self.get_parameter("min_persistent_object_confidence").value)
            mapper_conf = float(obj.get("confidence", 0.0) or 0.0)
            if mapper_conf <= persistent_min_conf:
                continue
            last_seen = float(obj.get("last_seen", 0.0) or 0.0)
            if not allow_historical and last_seen < self.started_at - allowance:
                continue
            source_id = obj.get("object_id")
            label = _clean_label(obj.get("label", "object"))
            map_pose = {
                "frame_id": "map",
                "x": float(obj.get("x", 0.0)),
                "y": float(obj.get("y", 0.0)),
                "z": float(obj.get("z", 0.0)),
                "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
            }
            canonical_id, prior_data = self._canonical_object_identity(
                api, label, source_id, map_pose, obj.get("extent_x"), obj.get("extent_y")
            )
            prior_source_ids = []
            if isinstance(prior_data, dict):
                prior_source_ids = [str(x) for x in (prior_data.get("source_object_ids") or [])]
                if prior_data.get("source_object_id") is not None:
                    prior_source_ids.append(str(prior_data.get("source_object_id")))
            merged_source_ids = sorted(set(prior_source_ids + [str(source_id)]))

            # map_pose/object_pose are the PHYSICAL OBJECT centroid in map frame.
            # The mapper's legacy approach_pose was the robot observation viewpoint;
            # preserve that separately and derive a standoff Nav2 goal from the object.
            observer_pose = obj.get("last_observer_pose") or obj.get("observation_viewpoint") or obj.get("approach_pose") or {}
            nav_approach = {}
            if isinstance(observer_pose, dict) and observer_pose.get("x") is not None and observer_pose.get("y") is not None:
                try:
                    ox, oy = float(map_pose["x"]), float(map_pose["y"])
                    vx, vy = float(observer_pose["x"]), float(observer_pose["y"])
                    dx, dy = vx - ox, vy - oy
                    norm = math.hypot(dx, dy)
                    if norm > 0.08:
                        extent = max(float(obj.get("extent_x") or 0.30), float(obj.get("extent_y") or 0.30))
                        standoff = min(max(0.85, 0.5 * extent + 0.55), max(0.65, norm))
                        gx, gy = ox + dx / norm * standoff, oy + dy / norm * standoff
                        gyaw = math.atan2(oy - gy, ox - gx)
                        nav_approach = {
                            "frame_id": "map", "x": gx, "y": gy, "z": 0.0,
                            "qx": 0.0, "qy": 0.0,
                            "qz": math.sin(0.5 * gyaw), "qw": math.cos(0.5 * gyaw),
                            "yaw": gyaw, "standoff_m": standoff,
                            "source": "derived_from_object_pose_and_observer",
                        }
                except Exception:
                    nav_approach = {}
            api.upsert_object_instance(self.session_name, {
                "object_id": canonical_id,
                "source_object_id": source_id,
                "source_object_ids": merged_source_ids,
                "identity_policy": "source continuity, then conservative same-class map-space reconciliation",
                "label": label,
                "map_pose": map_pose,
                "object_pose": map_pose,
                "last_observer_pose": observer_pose if isinstance(observer_pose, dict) else {},
                "navigation_approach_pose": nav_approach,
                "approach_pose": nav_approach,
                "location_semantics": "object_pose/map_pose=physical object centroid; last_observer_pose=robot observation pose; navigation_approach_pose=derived Nav2 standoff",
                "extent_x": obj.get("extent_x"),
                "extent_y": obj.get("extent_y"),
                "extent_z": obj.get("extent_z"),
                "mapper_confidence": float(obj.get("confidence", 0.0) or 0.0),
                "confirmations": int(obj.get("confirmations", 0) or 0),
                "observations": int(obj.get("observations", 0) or 0),
                "variance_m2": float(obj.get("variance_m2", 0.0) or 0.0),
                "first_seen": obj.get("first_seen"),
                "last_seen": obj.get("last_seen"),
                "confirmed": True,
                "countable": True,
                "status": "present",
                "geometry_source": obj.get("source"),
                "confidence": {
                    "perception_confidence": float(obj.get("confidence", 0.0) or 0.0),
                    "memory_confidence": min(1.0, 0.4 + 0.15 * int(obj.get("confirmations", 0) or 0)),
                },
                "source": ["pose_aware_object_mapper", str(obj.get("source") or "vision")],
            })
            ingested += 1
        if ingested:
            self.status_pub.publish(String(data=json.dumps({
                "event": "object_map_ingested", "session": self.session_name, "count": ingested,
            }, sort_keys=True)))
        self._publish_inventory()

    def _publish_inventory(self) -> None:
        visible = self.latest_visible.get("detections", []) or []
        visible_counts = dict(Counter(str(d.get("label") or "object") for d in visible if isinstance(d, dict)))
        remembered_counts: Dict[str, int] = {}
        remembered_total = 0
        api = self._ensure_api()
        if api is not None:
            try:
                result = api.count_objects(self.session_name, label="", room="", confirmed_only=True)
                remembered_counts = result.get("counts", {}) or {}
                remembered_total = int(result.get("count", sum(remembered_counts.values())) or 0)
            except Exception as exc:
                self.get_logger().debug(f"Inventory memory query skipped: {exc}")
        payload = {
            "session_name": self.session_name,
            "stamp_sec": time.time(),
            "visible_counts": visible_counts,
            "visible_count_total": sum(visible_counts.values()),
            "visible_objects": visible[:50],
            "remembered_counts": remembered_counts,
            "remembered_count_total": remembered_total,
            "counting_policy": "visible=live YOLO/SAM tracks; remembered=confirmed pose-aware fused ObjectInstances",
        }
        self.inventory_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldObjectMemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
