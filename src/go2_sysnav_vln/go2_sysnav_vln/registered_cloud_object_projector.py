from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import extract_detections, finite_float


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


class RegisteredCloudObjectProjector(Node):
    """Project 2D detections through an organized, image-aligned PointCloud2.

    This is the closest portable equivalent to SysNav's registered point-cloud
    object construction. It is valid only when the cloud is organized and pixel
    aligned with the detector image. For a LiDAR + monocular camera, a calibrated
    fusion node must produce such a cloud first; this node intentionally refuses
    unorganized or dimension-mismatched input instead of fabricating 3D geometry.
    """

    def __init__(self) -> None:
        super().__init__("go2_registered_cloud_object_projector")
        self.declare_parameter("detections_2d_topic", "/go2_vln/target_detections_2d")
        self.declare_parameter("organized_cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("detections_3d_topic", "/go2_vln/target_detections_3d")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_cloud_age_sec", 0.25)
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("max_sample_points", 2500)
        self.declare_parameter("min_points", 18)
        self.declare_parameter("min_range_m", 0.20)
        self.declare_parameter("max_range_m", 10.0)
        self.declare_parameter("depth_bin_m", 0.20)
        self.declare_parameter("depth_band_m", 0.35)
        self.declare_parameter("extent_percentile_low", 5.0)
        self.declare_parameter("extent_percentile_high", 95.0)

        self.cloud: Optional[PointCloud2] = None
        self.cloud_received = 0.0
        self.last_contract_warning = 0.0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            String, str(self.get_parameter("detections_3d_topic").value), 10
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("organized_cloud_topic").value),
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detections_2d_topic").value),
            self.on_detections,
            20,
        )
        self.get_logger().info(
            "registered cloud projector ready; requires organized image-aligned PointCloud2"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        self.cloud = msg
        self.cloud_received = time.monotonic()

    def warn_contract(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_contract_warning > 5.0:
            self.last_contract_warning = now
            self.get_logger().warning(text)

    def publish(self, source_payload: Dict[str, Any], detections: List[Dict[str, Any]], error: str = "") -> None:
        payload = {
            "success": bool(detections),
            "geometry": "registered_point_cloud",
            "source_geometry": source_payload.get("geometry", "image_2d"),
            "stamp_sec": time.time(),
            "target": source_payload.get("target", ""),
            "target_spec": source_payload.get("target_spec", {}),
            "detections": detections,
        }
        if error:
            payload["error"] = error
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def cloud_points(self, cloud: PointCloud2, uvs: Sequence[Tuple[int, int]]) -> np.ndarray:
        raw = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=list(uvs)
        )
        if isinstance(raw, np.ndarray):
            arr = raw
        else:
            arr = np.asarray(list(raw))
        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if arr.dtype.names:
            return np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
        arr = np.asarray(arr, dtype=np.float64)
        return arr.reshape((-1, 3))

    def foreground_points(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return points
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(ranges)
        valid &= ranges >= float(self.get_parameter("min_range_m").value)
        valid &= ranges <= float(self.get_parameter("max_range_m").value)
        points, ranges = points[valid], ranges[valid]
        minimum = int(self.get_parameter("min_points").value)
        if len(points) < minimum:
            return np.empty((0, 3), dtype=np.float64)
        bin_width = max(0.05, float(self.get_parameter("depth_bin_m").value))
        bins = np.floor(ranges / bin_width).astype(np.int64)
        unique, counts = np.unique(bins, return_counts=True)
        eligible = [(int(b), int(c)) for b, c in zip(unique, counts) if int(c) >= minimum]
        if not eligible:
            return np.empty((0, 3), dtype=np.float64)
        # Prefer the nearest substantial depth component to suppress background walls.
        chosen = min(eligible, key=lambda item: item[0])[0]
        center = (chosen + 0.5) * bin_width
        band = max(bin_width, float(self.get_parameter("depth_band_m").value))
        keep = np.abs(ranges - center) <= band
        selected = points[keep]
        return selected if len(selected) >= minimum else np.empty((0, 3), dtype=np.float64)

    def map_transform(self, cloud: PointCloud2) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        frame = str(cloud.header.frame_id)
        if not frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                frame,
                rclpy.time.Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.25),
            )
        except Exception as exc:
            self.warn_contract(f"registered cloud TF unavailable: {type(exc).__name__}: {exc}")
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z], dtype=np.float64)

    def sample_uvs(self, bbox: Sequence[float], width: int, height: int) -> List[Tuple[int, int]]:
        if len(bbox) < 4:
            return []
        x0, y0, x1, y1 = [int(round(float(v))) for v in bbox[:4]]
        x0, x1 = max(0, min(x0, width - 1)), max(0, min(x1, width))
        y0, y1 = max(0, min(y0, height - 1)), max(0, min(y1, height))
        if x1 <= x0 or y1 <= y0:
            return []
        stride = max(1, int(self.get_parameter("pixel_stride").value))
        uvs = [(u, v) for v in range(y0, y1, stride) for u in range(x0, x1, stride)]
        limit = max(1, int(self.get_parameter("max_sample_points").value))
        if len(uvs) > limit:
            indices = np.linspace(0, len(uvs) - 1, limit, dtype=np.int64)
            uvs = [uvs[int(i)] for i in indices]
        return uvs

    def on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        cloud = self.cloud
        if cloud is None:
            self.publish(payload, [], "no_organized_cloud")
            return
        age = time.monotonic() - self.cloud_received
        if age > float(self.get_parameter("max_cloud_age_sec").value):
            self.publish(payload, [], f"organized_cloud_stale:{age:.3f}")
            return
        image_width = int(payload.get("image_width", 0) or 0)
        image_height = int(payload.get("image_height", 0) or 0)
        if cloud.height <= 1 or cloud.width <= 1:
            self.warn_contract("cloud is unorganized; calibrated image/cloud registration is required")
            self.publish(payload, [], "cloud_unorganized")
            return
        if image_width and image_height and (
            int(cloud.width) != image_width or int(cloud.height) != image_height
        ):
            self.warn_contract(
                f"cloud/image dimensions differ ({cloud.width}x{cloud.height} vs "
                f"{image_width}x{image_height}); refusing invalid projection"
            )
            self.publish(payload, [], "cloud_image_dimension_mismatch")
            return
        transform = self.map_transform(cloud)
        if transform is None:
            self.publish(payload, [], "cloud_to_map_tf_unavailable")
            return
        rotation, translation = transform
        fused: List[Dict[str, Any]] = []
        for detection in extract_detections(msg.data):
            bbox = detection.get("bbox", [])
            uvs = self.sample_uvs(bbox, int(cloud.width), int(cloud.height))
            if not uvs:
                continue
            try:
                points_sensor = self.foreground_points(self.cloud_points(cloud, uvs))
            except Exception as exc:
                self.warn_contract(f"PointCloud2 projection failed: {type(exc).__name__}: {exc}")
                continue
            if len(points_sensor) < int(self.get_parameter("min_points").value):
                continue
            points_map = points_sensor @ rotation.T + translation
            low = float(self.get_parameter("extent_percentile_low").value)
            high = float(self.get_parameter("extent_percentile_high").value)
            lo = np.percentile(points_map, low, axis=0)
            hi = np.percentile(points_map, high, axis=0)
            keep = np.all((points_map >= lo) & (points_map <= hi), axis=1)
            points_map = points_map[keep]
            if len(points_map) < int(self.get_parameter("min_points").value):
                continue
            centroid = np.median(points_map, axis=0)
            lo = np.percentile(points_map, low, axis=0)
            hi = np.percentile(points_map, high, axis=0)
            item = dict(detection)
            item.update(
                {
                    "geometry": "registered_point_cloud",
                    "geometry_source": "organized_image_aligned_cloud",
                    "cloud_frame_id": str(cloud.header.frame_id),
                    "centroid_map": [float(v) for v in centroid],
                    "bbox3d": {
                        "min": [float(v) for v in lo],
                        "max": [float(v) for v in hi],
                    },
                    # Keep a bounded sample for the mapper/debug logs.
                    "points_map": [
                        [float(v) for v in row]
                        for row in points_map[
                            np.linspace(0, len(points_map) - 1, min(160, len(points_map)), dtype=np.int64)
                        ]
                    ],
                    "point_count": int(len(points_map)),
                    "source": str(detection.get("source", "detector")) + "+registered_cloud",
                }
            )
            fused.append(item)
        self.publish(payload, fused, "" if fused else "no_valid_3d_object_points")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RegisteredCloudObjectProjector()
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
