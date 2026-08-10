from __future__ import annotations

import json
import math
import os
import sqlite3
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .common import (
    angle_wrap,
    bbox_iou_2d,
    clean_label,
    extract_detections,
    finite_float,
    median,
    quat_to_yaw,
    viewpoint_is_novel,
)


@dataclass
class ObjectRecord:
    object_id: int
    label: str
    x: float
    y: float
    z: float
    extent_x: float
    extent_y: float
    extent_z: float
    confidence: float
    confirmations: int
    observations: int
    variance_m2: float
    first_seen: float
    last_seen: float
    confirmed: bool
    source: str

    @property
    def bbox2d(self) -> Tuple[float, float, float, float]:
        hx = max(0.08, 0.5 * self.extent_x)
        hy = max(0.08, 0.5 * self.extent_y)
        return self.x - hx, self.y - hy, self.x + hx, self.y + hy


class PoseAwareObjectMapper(Node):
    """Persistent SysNav-style object fusion adapted to a Nav2 Go2 stack.

    Preferred input is a detector payload containing ``points_map`` or a map-frame
    ``centroid_map``/``bbox3d``. A LaserScan ray association fallback is retained
    for the current Go2 camera stack. Independent evidence follows upstream SysNav:
    object-relative view angle and range must be novel. A global displacement gate
    also prevents multiple detector backends at one pose from inflating confidence.
    """

    def __init__(self) -> None:
        super().__init__("go2_pose_aware_object_mapper")
        self.declare_parameter("detection_topics", ["/go2_vln/target_detections_3d"])
        self.declare_parameter("scan_topic", "/scan_nav")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fallback_hfov_deg", 70.0)
        self.declare_parameter("camera_to_base_yaw_rad", 0.0)
        self.declare_parameter("scan_bearing_offset_rad", 0.0)
        self.declare_parameter("scan_window_deg", 4.0)
        self.declare_parameter("max_scan_age_sec", 0.50)
        self.declare_parameter("min_range_m", 0.25)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("min_detection_confidence", 0.60)
        # Upstream SysNav uses 5 degrees and 0.3 m object-relative novelty.
        self.declare_parameter("view_angle_threshold_deg", 5.0)
        self.declare_parameter("view_range_threshold_m", 0.30)
        self.declare_parameter("min_robot_translation_m", 0.20)
        self.declare_parameter("min_robot_rotation_rad", 0.15)
        self.declare_parameter("merge_radius_m", 0.55)
        self.declare_parameter("minimum_direct_merge_m", 0.25)
        self.declare_parameter("merge_iou_threshold", 0.20)
        self.declare_parameter("merge_overlap_threshold", 0.40)
        self.declare_parameter("same_frame_dedup_radius_m", 0.22)
        self.declare_parameter("same_frame_bbox_iou", 0.45)
        self.declare_parameter("drop_2d_when_recent_3d_sec", 0.60)
        self.declare_parameter("min_confirmations", 2)
        self.declare_parameter("database_path", "~/.ros/go2_sysnav_vln/object_map.sqlite3")
        self.declare_parameter("reset_database_on_start", False)
        self.declare_parameter("publish_topic", "/go2_vln/object_map")
        self.declare_parameter("marker_topic", "/go2_vln/object_markers")
        self.declare_parameter("publish_tentative", False)
        self.declare_parameter("require_registered_3d", True)
        self.declare_parameter("allow_scan_ray_fallback", False)
        self.declare_parameter("publish_extent_markers", False)
        self.declare_parameter("publish_label_markers", True)
        self.declare_parameter("label_min_separation_m", 0.38)

        self.scan: Optional[LaserScan] = None
        self.scan_received_monotonic = 0.0
        self.camera_info: Optional[CameraInfo] = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.object_viewpoints: Dict[int, List[Tuple[float, float, float]]] = {}
        self.last_3d_label_stamp: Dict[str, float] = {}
        self.objects: Dict[int, ObjectRecord] = {}
        if str(self.get_parameter("reset_database_on_start").value).strip().lower() in {"1", "true", "yes", "on"}:
            _db_path = os.path.expanduser(str(self.get_parameter("database_path").value))
            for _suffix in ("", "-wal", "-shm"):
                try:
                    os.remove(_db_path + _suffix)
                except FileNotFoundError:
                    pass
        self.db = self.open_database()
        self.load_objects()
        self.load_object_viewpoints()

        self.pub = self.create_publisher(String, str(self.get_parameter("publish_topic").value), 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), marker_qos
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info, qos_profile_sensor_data,
        )
        for topic in list(self.get_parameter("detection_topics").value):
            self.create_subscription(String, str(topic), self.on_detections, 20)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            f"SysNav-style object mapper ready with {len(self.objects)} persisted objects"
        )

    def open_database(self) -> sqlite3.Connection:
        path = os.path.expanduser(str(self.get_parameter("database_path").value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        db = sqlite3.connect(path, check_same_thread=False)
        db.execute(
            """
            CREATE TABLE IF NOT EXISTS objects (
              object_id INTEGER PRIMARY KEY AUTOINCREMENT,
              label TEXT NOT NULL,
              x REAL NOT NULL,
              y REAL NOT NULL,
              z REAL NOT NULL DEFAULT 0.0,
              extent_x REAL NOT NULL DEFAULT 0.30,
              extent_y REAL NOT NULL DEFAULT 0.30,
              extent_z REAL NOT NULL DEFAULT 0.50,
              confidence REAL NOT NULL,
              confirmations INTEGER NOT NULL,
              observations INTEGER NOT NULL,
              variance_m2 REAL NOT NULL,
              first_seen REAL NOT NULL,
              last_seen REAL NOT NULL,
              confirmed INTEGER NOT NULL,
              source TEXT NOT NULL
            )
            """
        )
        object_columns = {str(row[1]) for row in db.execute("PRAGMA table_info(objects)")}
        for name, ddl in (
            ("z", "REAL NOT NULL DEFAULT 0.0"),
            ("extent_x", "REAL NOT NULL DEFAULT 0.30"),
            ("extent_y", "REAL NOT NULL DEFAULT 0.30"),
            ("extent_z", "REAL NOT NULL DEFAULT 0.50"),
        ):
            if name not in object_columns:
                db.execute(f"ALTER TABLE objects ADD COLUMN {name} {ddl}")
        db.execute(
            """
            CREATE TABLE IF NOT EXISTS observations (
              observation_id INTEGER PRIMARY KEY AUTOINCREMENT,
              object_id INTEGER NOT NULL,
              stamp REAL NOT NULL,
              robot_x REAL NOT NULL,
              robot_y REAL NOT NULL,
              robot_yaw REAL NOT NULL,
              object_x REAL NOT NULL,
              object_y REAL NOT NULL,
              object_z REAL NOT NULL DEFAULT 0.0,
              range_m REAL NOT NULL,
              bearing_rad REAL NOT NULL,
              confidence REAL NOT NULL,
              source TEXT NOT NULL,
              track_key TEXT NOT NULL,
              independent INTEGER NOT NULL DEFAULT 0,
              geometry_source TEXT NOT NULL DEFAULT 'scan'
            )
            """
        )
        obs_columns = {str(row[1]) for row in db.execute("PRAGMA table_info(observations)")}
        for name, ddl in (
            ("object_z", "REAL NOT NULL DEFAULT 0.0"),
            ("independent", "INTEGER NOT NULL DEFAULT 0"),
            ("geometry_source", "TEXT NOT NULL DEFAULT 'scan'"),
        ):
            if name not in obs_columns:
                db.execute(f"ALTER TABLE observations ADD COLUMN {name} {ddl}")
        db.commit()
        return db

    def load_objects(self) -> None:
        rows = self.db.execute(
            "SELECT object_id,label,x,y,z,extent_x,extent_y,extent_z,confidence,"
            "confirmations,observations,variance_m2,first_seen,last_seen,confirmed,source "
            "FROM objects"
        ).fetchall()
        for row in rows:
            rec = ObjectRecord(
                object_id=int(row[0]), label=str(row[1]), x=float(row[2]), y=float(row[3]),
                z=float(row[4]), extent_x=float(row[5]), extent_y=float(row[6]),
                extent_z=float(row[7]), confidence=float(row[8]), confirmations=int(row[9]),
                observations=int(row[10]), variance_m2=float(row[11]), first_seen=float(row[12]),
                last_seen=float(row[13]), confirmed=bool(row[14]), source=str(row[15]),
            )
            self.objects[rec.object_id] = rec

    def load_object_viewpoints(self) -> None:
        rows = self.db.execute(
            "SELECT object_id,robot_x,robot_y,robot_yaw FROM observations "
            "WHERE independent=1 ORDER BY observation_id"
        ).fetchall()
        for object_id, x, y, yaw in rows:
            self.object_viewpoints.setdefault(int(object_id), []).append(
                (float(x), float(y), float(yaw))
            )

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_received_monotonic = time.monotonic()

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(), timeout=Duration(seconds=0.15),
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                quat_to_yaw(tf.transform.rotation),
            )
        except Exception:
            return None

    def detection_bearing(self, det: Dict[str, Any], image_width: float) -> Optional[float]:
        explicit = finite_float(det.get("bearing_rad"))
        if explicit is not None:
            return angle_wrap(explicit)
        box = det.get("bbox", det.get("box_xyxy"))
        if not isinstance(box, (list, tuple)) or len(box) < 4:
            return None
        u = 0.5 * (float(box[0]) + float(box[2]))
        if self.camera_info is not None and len(self.camera_info.k) >= 3 and self.camera_info.k[0] > 1.0:
            bearing = math.atan2(u - float(self.camera_info.k[2]), float(self.camera_info.k[0]))
        else:
            width = image_width if image_width > 1.0 else 640.0
            hfov = math.radians(float(self.get_parameter("fallback_hfov_deg").value))
            bearing = ((u / width) - 0.5) * hfov
        return angle_wrap(bearing + float(self.get_parameter("camera_to_base_yaw_rad").value))

    def range_for_bearing(self, bearing: float) -> Optional[float]:
        scan = self.scan
        if scan is None or not scan.ranges or abs(scan.angle_increment) < 1e-9:
            return None
        if time.monotonic() - self.scan_received_monotonic > float(
            self.get_parameter("max_scan_age_sec").value
        ):
            return None
        scan_bearing = angle_wrap(
            bearing + float(self.get_parameter("scan_bearing_offset_rad").value)
        )
        center = int(round((scan_bearing - float(scan.angle_min)) / float(scan.angle_increment)))
        half = max(1, int(round(
            math.radians(float(self.get_parameter("scan_window_deg").value))
            / max(abs(float(scan.angle_increment)), 1e-6)
        )))
        min_r = max(float(scan.range_min), float(self.get_parameter("min_range_m").value))
        max_r = min(float(scan.range_max), float(self.get_parameter("max_range_m").value))
        values: List[float] = []
        for idx in range(center - half, center + half + 1):
            if 0 <= idx < len(scan.ranges):
                value = float(scan.ranges[idx])
                if math.isfinite(value) and min_r <= value <= max_r:
                    values.append(value)
        return median(values)

    @staticmethod
    def _xyz(value: Any) -> Optional[Tuple[float, float, float]]:
        if isinstance(value, dict):
            vals = (value.get("x"), value.get("y"), value.get("z", 0.0))
        elif isinstance(value, (list, tuple)) and len(value) >= 2:
            vals = (value[0], value[1], value[2] if len(value) > 2 else 0.0)
        else:
            return None
        xyz = tuple(finite_float(v) for v in vals)
        if any(v is None for v in xyz):
            return None
        return float(xyz[0]), float(xyz[1]), float(xyz[2])

    def localize_detection(
        self, det: Dict[str, Any], pose: Tuple[float, float, float], image_width: float
    ) -> Optional[Dict[str, Any]]:
        # 1) Preferred registered 3D mask cloud in map frame.
        points = det.get("points_map")
        if isinstance(points, list) and points:
            arr = np.asarray(points, dtype=np.float64)
            if arr.ndim == 2 and arr.shape[1] >= 2:
                arr = arr[:, :3] if arr.shape[1] >= 3 else np.c_[arr[:, :2], np.zeros(len(arr))]
                arr = arr[np.isfinite(arr).all(axis=1)]
                if len(arr) >= 2:
                    low = np.percentile(arr, 5, axis=0)
                    high = np.percentile(arr, 95, axis=0)
                    center = np.median(arr, axis=0)
                    return {
                        "x": float(center[0]), "y": float(center[1]), "z": float(center[2]),
                        "extent_x": max(0.08, float(high[0] - low[0])),
                        "extent_y": max(0.08, float(high[1] - low[1])),
                        "extent_z": max(0.08, float(high[2] - low[2])),
                        "range_m": math.hypot(float(center[0]) - pose[0], float(center[1]) - pose[1]),
                        "bearing_rad": angle_wrap(math.atan2(float(center[1]) - pose[1], float(center[0]) - pose[0]) - pose[2]),
                        "geometry_source": "registered_points",
                    }
        # 2) Explicit map-frame centroid and optional 3D box.
        center = self._xyz(det.get("centroid_map", det.get("position_map")))
        bbox3d = det.get("bbox3d")
        if center is not None:
            ext = self._xyz(det.get("extent", det.get("extent_xyz")))
            if ext is None and isinstance(bbox3d, dict):
                low_box = self._xyz(bbox3d.get("min"))
                high_box = self._xyz(bbox3d.get("max"))
                if low_box is not None and high_box is not None:
                    ext = tuple(max(0.08, b - a) for a, b in zip(low_box, high_box))
            ext = ext or (0.30, 0.30, 0.50)
            return {
                "x": center[0], "y": center[1], "z": center[2],
                "extent_x": max(0.08, abs(ext[0])), "extent_y": max(0.08, abs(ext[1])),
                "extent_z": max(0.08, abs(ext[2])),
                "range_m": math.hypot(center[0] - pose[0], center[1] - pose[1]),
                "bearing_rad": angle_wrap(math.atan2(center[1] - pose[1], center[0] - pose[0]) - pose[2]),
                "geometry_source": "explicit_map",
            }
        if isinstance(bbox3d, dict):
            low = self._xyz(bbox3d.get("min"))
            high = self._xyz(bbox3d.get("max"))
            if low is not None and high is not None:
                center = tuple(0.5 * (a + b) for a, b in zip(low, high))
                return {
                    "x": center[0], "y": center[1], "z": center[2],
                    "extent_x": max(0.08, high[0] - low[0]),
                    "extent_y": max(0.08, high[1] - low[1]),
                    "extent_z": max(0.08, high[2] - low[2]),
                    "range_m": math.hypot(center[0] - pose[0], center[1] - pose[1]),
                    "bearing_rad": angle_wrap(math.atan2(center[1] - pose[1], center[0] - pose[0]) - pose[2]),
                    "geometry_source": "bbox3d",
                }
        # 3) Explicit range/bearing, else planar scan association.
        bearing = self.detection_bearing(det, image_width)
        if bearing is None:
            return None
        rng = finite_float(det.get("range_m"))
        geometry_source = "explicit_range"
        if rng is None:
            if not bool(self.get_parameter("allow_scan_ray_fallback").value):
                return None
            rng = self.range_for_bearing(bearing)
            geometry_source = "scan_ray"
        if rng is None:
            return None
        if not (float(self.get_parameter("min_range_m").value) <= rng <= float(self.get_parameter("max_range_m").value)):
            return None
        global_bearing = pose[2] + bearing
        return {
            "x": pose[0] + rng * math.cos(global_bearing),
            "y": pose[1] + rng * math.sin(global_bearing),
            "z": finite_float(det.get("z_m"), 0.35) or 0.35,
            "extent_x": max(0.12, finite_float(det.get("extent_x"), 0.30) or 0.30),
            "extent_y": max(0.12, finite_float(det.get("extent_y"), 0.30) or 0.30),
            "extent_z": max(0.12, finite_float(det.get("extent_z"), 0.50) or 0.50),
            "range_m": rng, "bearing_rad": bearing, "geometry_source": geometry_source,
        }

    def independent_view(
        self, rec: ObjectRecord, pose: Tuple[float, float, float]
    ) -> bool:
        prior = self.object_viewpoints.get(rec.object_id, [])
        novel = viewpoint_is_novel(
            (rec.x, rec.y), pose, prior,
            math.radians(float(self.get_parameter("view_angle_threshold_deg").value)),
            float(self.get_parameter("view_range_threshold_m").value),
            float(self.get_parameter("min_robot_translation_m").value),
            float(self.get_parameter("min_robot_rotation_rad").value),
        )
        if novel:
            prior.append(pose)
            self.object_viewpoints[rec.object_id] = prior[-64:]
        return novel

    def can_merge(self, rec: ObjectRecord, geom: Dict[str, Any]) -> Tuple[bool, float]:
        dist = math.hypot(rec.x - geom["x"], rec.y - geom["y"])
        # Same concept as upstream: extent-adaptive direct distance, then overlap.
        extent_norm = math.hypot(
            0.5 * (rec.extent_x + float(geom["extent_x"])),
            0.5 * (rec.extent_y + float(geom["extent_y"])),
        )
        adaptive = max(
            float(self.get_parameter("minimum_direct_merge_m").value),
            min(1.25, 0.5 * extent_norm + math.sqrt(max(0.0, rec.variance_m2))),
        )
        if dist <= max(float(self.get_parameter("merge_radius_m").value), adaptive):
            return True, dist
        new_box = (
            geom["x"] - 0.5 * geom["extent_x"], geom["y"] - 0.5 * geom["extent_y"],
            geom["x"] + 0.5 * geom["extent_x"], geom["y"] + 0.5 * geom["extent_y"],
        )
        iou, ratio_a, ratio_b = bbox_iou_2d(rec.bbox2d, new_box)
        overlap = float(self.get_parameter("merge_overlap_threshold").value)
        return (
            iou >= float(self.get_parameter("merge_iou_threshold").value)
            or ratio_a >= overlap or ratio_b >= overlap,
            dist,
        )

    def nearest_object(self, label: str, geom: Dict[str, Any]) -> Optional[ObjectRecord]:
        best: Optional[ObjectRecord] = None
        best_dist = float("inf")
        for rec in self.objects.values():
            if rec.label != label:
                continue
            merge, dist = self.can_merge(rec, geom)
            if merge and dist < best_dist:
                best, best_dist = rec, dist
        return best

    def insert_object(
        self, label: str, geom: Dict[str, Any], confidence: float, source: str, now: float
    ) -> ObjectRecord:
        min_conf = int(self.get_parameter("min_confirmations").value)
        values = (
            label, geom["x"], geom["y"], geom["z"], geom["extent_x"], geom["extent_y"],
            geom["extent_z"], confidence, 1, 1, 0.20, now, now, int(min_conf <= 1), source,
        )
        cursor = self.db.execute(
            "INSERT INTO objects(label,x,y,z,extent_x,extent_y,extent_z,confidence,confirmations,"
            "observations,variance_m2,first_seen,last_seen,confirmed,source) "
            "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", values,
        )
        # Dataclass positional construction above is intentionally avoided below for clarity.
        rec = ObjectRecord(
            object_id=int(cursor.lastrowid), label=label, x=float(geom["x"]), y=float(geom["y"]),
            z=float(geom["z"]), extent_x=float(geom["extent_x"]), extent_y=float(geom["extent_y"]),
            extent_z=float(geom["extent_z"]), confidence=confidence, confirmations=1,
            observations=1, variance_m2=0.20, first_seen=now, last_seen=now,
            confirmed=min_conf <= 1, source=source,
        )
        self.objects[rec.object_id] = rec
        return rec

    def update_object(
        self, rec: ObjectRecord, geom: Dict[str, Any], confidence: float,
        independent: bool, source: str, now: float,
    ) -> None:
        old_x, old_y = rec.x, rec.y
        weight_old = max(1.0, float(rec.observations))
        weight_new = max(0.20, confidence) * (1.5 if independent else 0.20)
        total = weight_old + weight_new
        for field in ("x", "y", "z", "extent_x", "extent_y", "extent_z"):
            old = float(getattr(rec, field))
            new = float(geom[field])
            setattr(rec, field, (old * weight_old + new * weight_new) / total)
        residual2 = (float(geom["x"]) - rec.x) ** 2 + (float(geom["y"]) - rec.y) ** 2
        rec.variance_m2 = min(4.0, 0.85 * rec.variance_m2 + 0.15 * residual2)
        rec.confidence = max(rec.confidence, confidence)
        rec.observations += 1
        if independent:
            rec.confirmations += 1
        rec.confirmed = rec.confirmations >= int(self.get_parameter("min_confirmations").value)
        rec.last_seen = now
        rec.source = source
        self.db.execute(
            "UPDATE objects SET x=?,y=?,z=?,extent_x=?,extent_y=?,extent_z=?,confidence=?,"
            "confirmations=?,observations=?,variance_m2=?,last_seen=?,confirmed=?,source=? "
            "WHERE object_id=?",
            (
                rec.x, rec.y, rec.z, rec.extent_x, rec.extent_y, rec.extent_z, rec.confidence,
                rec.confirmations, rec.observations, rec.variance_m2, rec.last_seen,
                int(rec.confirmed), rec.source, rec.object_id,
            ),
        )

    def add_observation(
        self, rec: ObjectRecord, pose: Tuple[float, float, float], geom: Dict[str, Any],
        confidence: float, source: str, track_key: str, independent: bool, now: float,
    ) -> None:
        self.db.execute(
            "INSERT INTO observations(object_id,stamp,robot_x,robot_y,robot_yaw,object_x,object_y,"
            "object_z,range_m,bearing_rad,confidence,source,track_key,independent,geometry_source) "
            "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (
                rec.object_id, now, pose[0], pose[1], pose[2], geom["x"], geom["y"], geom["z"],
                geom["range_m"], geom["bearing_rad"], confidence, source, track_key,
                int(independent), geom["geometry_source"],
            ),
        )

    def on_detections(self, msg: String) -> None:
        pose = self.robot_pose()
        if pose is None:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        detections = extract_detections(msg.data)
        image_width = float(payload.get("image_width", 0.0)) if isinstance(payload, dict) else 0.0
        source_default = str(payload.get("backend", payload.get("sam2_mode", "vision"))) if isinstance(payload, dict) else "vision"
        now = time.time()
        min_score = float(self.get_parameter("min_detection_confidence").value)
        accepted_this_frame: List[Tuple[str, Dict[str, Any], Sequence[float]]] = []

        for index, det in enumerate(detections):
            label = clean_label(str(det.get("label", det.get("class", ""))))
            confidence = finite_float(det.get("confidence", det.get("score", 0.0)), 0.0) or 0.0
            if not label or confidence <= min_score:
                continue
            payload_geometry = str(payload.get("geometry", "")) if isinstance(payload, dict) else ""
            is_3d = bool(det.get("points_map") or det.get("centroid_map") or det.get("bbox3d")) \
                or payload_geometry == "registered_point_cloud"
            if bool(self.get_parameter("require_registered_3d").value) and not is_3d:
                continue
            if is_3d:
                self.last_3d_label_stamp[label] = now
            elif now - self.last_3d_label_stamp.get(label, -1e9) < float(
                self.get_parameter("drop_2d_when_recent_3d_sec").value
            ):
                # The projector republishes the same detector event. Do not also
                # fuse its weaker 2D/scan estimate as a second physical object.
                continue
            geom = self.localize_detection(det, pose, image_width)
            if geom is None:
                continue
            image_box = det.get("bbox", det.get("box_xyxy", []))
            duplicate = False
            for old_label, old_geom, old_box in accepted_this_frame:
                if old_label != label:
                    continue
                close = math.hypot(geom["x"] - old_geom["x"], geom["y"] - old_geom["y"]) <= float(
                    self.get_parameter("same_frame_dedup_radius_m").value
                )
                iou = bbox_iou_2d(image_box, old_box)[0] if image_box and old_box else 0.0
                if close or iou >= float(self.get_parameter("same_frame_bbox_iou").value):
                    duplicate = True
                    break
            if duplicate:
                continue
            accepted_this_frame.append((label, geom, image_box))

            source = str(det.get("source", source_default))
            track_id = det.get("track_id", index)
            track_key = f"{source}:{label}:{track_id}"
            rec = self.nearest_object(label, geom)
            if rec is None:
                rec = self.insert_object(label, geom, confidence, source, now)
                self.object_viewpoints[rec.object_id] = [pose]
                independent = True
            else:
                independent = self.independent_view(rec, pose)
                self.update_object(rec, geom, confidence, independent, source, now)
            self.add_observation(rec, pose, geom, confidence, source, track_key, independent, now)
        self.db.commit()
        self.publish_map()

    def publish_map(self) -> None:
        objects = []
        for rec in sorted(self.objects.values(), key=lambda r: r.object_id):
            item = asdict(rec)
            viewpoints = self.object_viewpoints.get(rec.object_id, [])
            if viewpoints:
                vx, vy, vyaw = viewpoints[-1]
                item["approach_pose"] = {
                    "frame_id": str(self.get_parameter("map_frame").value),
                    "x": float(vx), "y": float(vy), "z": 0.0,
                    "qx": 0.0, "qy": 0.0,
                    "qz": math.sin(0.5 * float(vyaw)),
                    "qw": math.cos(0.5 * float(vyaw)),
                    "source": "last_confirmed_observation_viewpoint",
                }
            objects.append(item)
        self.pub.publish(String(data=json.dumps({
            "stamp_sec": time.time(),
            "mapping_contract": "sysnav_object_relative_view_fusion_v2",
            "objects": objects,
        }, sort_keys=True)))
        marker_array = MarkerArray()
        delete = Marker(); delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)
        publish_tentative = bool(self.get_parameter("publish_tentative").value)
        stamp = self.get_clock().now().to_msg()
        label_positions: List[Tuple[float, float]] = []
        for rec in sorted(self.objects.values(), key=lambda r: r.object_id):
            if not rec.confirmed and not publish_tentative:
                continue
            # A small map-plane sphere is the canonical object location. The
            # translucent cube is only the estimated physical extent.
            point = Marker()
            point.header.frame_id = str(self.get_parameter("map_frame").value)
            point.header.stamp = stamp
            point.ns = "sysnav_object_points"
            point.id = rec.object_id * 3
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x, point.pose.position.y = rec.x, rec.y
            point.pose.position.z = 0.10
            point.pose.orientation.w = 1.0
            point.scale.x = point.scale.y = point.scale.z = 0.22
            point.color.r = 0.10 if rec.confirmed else 1.0
            point.color.g = 0.95 if rec.confirmed else 0.55
            point.color.b = 0.15
            point.color.a = 1.0
            marker_array.markers.append(point)

            extent = Marker()
            extent.header = point.header
            extent.ns = "sysnav_object_extents"
            extent.id = rec.object_id * 3 + 1
            extent.type = Marker.CUBE
            extent.action = Marker.ADD
            extent.pose.position.x, extent.pose.position.y = rec.x, rec.y
            extent.pose.position.z = max(0.10, rec.z)
            extent.pose.orientation.w = 1.0
            extent.scale.x = max(0.10, rec.extent_x)
            extent.scale.y = max(0.10, rec.extent_y)
            extent.scale.z = max(0.10, rec.extent_z)
            extent.color.r = point.color.r
            extent.color.g = point.color.g
            extent.color.b = point.color.b
            extent.color.a = 0.24
            marker_array.markers.append(extent)

            text = Marker()
            text.header = point.header
            text.ns = "sysnav_object_labels"
            text.id = rec.object_id * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = rec.x, rec.y
            text.pose.position.z = max(0.50, rec.z + 0.5 * rec.extent_z + 0.12)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.14
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"{rec.label} #{rec.object_id} ({rec.confirmations}v)"
            marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

    def destroy_node(self) -> bool:
        try:
            self.db.commit(); self.db.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseAwareObjectMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
