from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import extract_detections


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float64)


class RegisteredCloudObjectProjector(Node):
    """Calibrated LiDAR-camera fusion for YOLO/SAM2 detections.

    Unlike the old implementation, this accepts an ordinary unorganized Go2
    PointCloud2. Points are transformed into the camera optical frame with TF,
    projected with CameraInfo intrinsics, filtered by the SAM polygon (or YOLO
    box), foreground-clustered by depth, then transformed into map coordinates.
    """

    def __init__(self) -> None:
        super().__init__('go2_registered_cloud_object_projector')
        self.declare_parameter('detections_2d_topic', '/object_explorer/sam2_detections')
        self.declare_parameter('pointcloud_topic', '/point_cloud2')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_3d_topic', '/go2_vln/target_detections_3d')
        self.declare_parameter('status_topic', '/go2_vln/projection_status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_cloud_age_sec', 1.25)
        self.declare_parameter('min_points', 5)
        self.declare_parameter('max_points_input', 18000)
        self.declare_parameter('max_points_output', 120)
        self.declare_parameter('min_range_m', 0.25)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('depth_bin_m', 0.20)
        self.declare_parameter('depth_band_m', 0.30)
        self.declare_parameter('extent_percentile_low', 8.0)
        self.declare_parameter('extent_percentile_high', 92.0)
        self.declare_parameter('max_linear_speed_mps', 0.22)
        self.declare_parameter('max_angular_speed_rps', 0.45)
        self.declare_parameter('ignore_velocity_gate', True)

        self.cloud: Optional[PointCloud2] = None
        self.cloud_received = 0.0
        self.camera_info: Optional[CameraInfo] = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_warning = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, str(self.get_parameter('detections_3d_topic').value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(PointCloud2, str(self.get_parameter('pointcloud_topic').value), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter('camera_info_topic').value), self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter('detections_2d_topic').value), self.on_detections, 20)
        self.get_logger().info(
            'Calibrated LiDAR-camera object projector ready: unorganized PointCloud2 + CameraInfo + TF -> map objects'
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        self.cloud = msg
        self.cloud_received = time.monotonic()

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self.linear_speed = math.hypot(float(t.linear.x), float(t.linear.y))
        self.angular_speed = abs(float(t.angular.z))

    def warn(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warning > 3.0:
            self.last_warning = now
            self.get_logger().warning(text)

    def status(self, state: str, **extra: Any) -> None:
        payload = {'state': state, 'stamp_sec': time.time(), **extra}
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish(self, source: Dict[str, Any], detections: List[Dict[str, Any]], error: str = '') -> None:
        payload = {
            'success': bool(detections),
            'geometry': 'registered_point_cloud',
            'stamp_sec': source.get('stamp_sec', time.time()),
            'image_width': int(source.get('image_width', 0) or 0),
            'image_height': int(source.get('image_height', 0) or 0),
            'detections': detections,
        }
        if error:
            payload['error'] = error
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def transform_parts(tf) -> Tuple[np.ndarray, np.ndarray]:
        q = tf.transform.rotation
        t = tf.transform.translation
        return quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z], dtype=np.float64)

    def lookup(self, target: str, source: str, stamp) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if not target or not source:
            return None
        attempts = []
        try:
            if stamp.sec != 0 or stamp.nanosec != 0:
                attempts.append(rclpy.time.Time.from_msg(stamp))
        except Exception:
            pass
        attempts.append(rclpy.time.Time())
        last_exc = None
        for when in attempts:
            try:
                tf = self.tf_buffer.lookup_transform(target, source, when, timeout=Duration(seconds=0.20))
                return self.transform_parts(tf)
            except Exception as exc:
                last_exc = exc
        self.warn(f'TF unavailable {source}->{target}: {type(last_exc).__name__}: {last_exc}')
        return None

    @staticmethod
    def cloud_xyz(msg: PointCloud2, limit: int) -> np.ndarray:
        raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if isinstance(raw, np.ndarray):
            arr = raw
        else:
            arr = np.asarray(list(raw))
        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if arr.dtype.names:
            arr = np.column_stack([arr['x'], arr['y'], arr['z']])
        arr = np.asarray(arr, dtype=np.float64).reshape((-1, 3))
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr) > limit:
            idx = np.linspace(0, len(arr) - 1, limit, dtype=np.int64)
            arr = arr[idx]
        return arr

    def foreground(self, cam_points: np.ndarray, source_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if len(cam_points) == 0:
            return cam_points, source_points
        z = cam_points[:, 2]
        min_r = float(self.get_parameter('min_range_m').value)
        max_r = float(self.get_parameter('max_range_m').value)
        valid = np.isfinite(z) & (z >= min_r) & (z <= max_r)
        cam_points, source_points, z = cam_points[valid], source_points[valid], z[valid]
        minimum = int(self.get_parameter('min_points').value)
        if len(z) < minimum:
            return np.empty((0, 3)), np.empty((0, 3))
        bw = max(0.08, float(self.get_parameter('depth_bin_m').value))
        bins = np.floor(z / bw).astype(np.int64)
        unique, counts = np.unique(bins, return_counts=True)
        eligible = [(int(b), int(c)) for b, c in zip(unique, counts) if int(c) >= minimum]
        if not eligible:
            return np.empty((0, 3)), np.empty((0, 3))
        chosen = min(eligible, key=lambda p: p[0])[0]
        center = (chosen + 0.5) * bw
        band = max(bw, float(self.get_parameter('depth_band_m').value))
        keep = np.abs(z - center) <= band
        return cam_points[keep], source_points[keep]

    def on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        cloud = self.cloud
        info = self.camera_info
        if cloud is None:
            self.publish(payload, [], 'no_pointcloud')
            self.status('waiting_pointcloud')
            return
        if info is None or len(info.k) < 6 or float(info.k[0]) <= 1.0 or float(info.k[4]) <= 1.0:
            self.publish(payload, [], 'no_valid_camera_info')
            self.status('waiting_camera_info')
            return
        age = time.monotonic() - self.cloud_received
        if age > float(self.get_parameter('max_cloud_age_sec').value):
            self.publish(payload, [], f'pointcloud_stale:{age:.3f}')
            self.status('pointcloud_stale', age_sec=round(age, 3))
            return
        if (not bool(self.get_parameter('ignore_velocity_gate').value)) and (self.linear_speed > float(self.get_parameter('max_linear_speed_mps').value) or self.angular_speed > float(self.get_parameter('max_angular_speed_rps').value)):
            self.publish(payload, [], 'robot_moving_too_fast_for_persistent_mapping')
            self.status('motion_gated', linear=round(self.linear_speed, 3), angular=round(self.angular_speed, 3))
            return

        source_frame = str(cloud.header.frame_id).strip()
        camera_frame = str(self.get_parameter('camera_frame').value).strip() or str(info.header.frame_id).strip()
        map_frame = str(self.get_parameter('map_frame').value).strip()
        cam_tf = self.lookup(camera_frame, source_frame, cloud.header.stamp)
        map_tf = self.lookup(map_frame, source_frame, cloud.header.stamp)
        if cam_tf is None or map_tf is None:
            self.publish(payload, [], 'required_tf_unavailable')
            self.status('tf_unavailable', cloud_frame=source_frame, camera_frame=camera_frame)
            return

        try:
            points_source = self.cloud_xyz(cloud, int(self.get_parameter('max_points_input').value))
        except Exception as exc:
            self.warn(f'PointCloud2 decode failed: {type(exc).__name__}: {exc}')
            self.publish(payload, [], 'pointcloud_decode_failed')
            return
        if len(points_source) == 0:
            self.publish(payload, [], 'empty_pointcloud')
            return

        r_cam, t_cam = cam_tf
        r_map, t_map = map_tf
        points_cam = points_source @ r_cam.T + t_cam
        depth = points_cam[:, 2]
        valid = np.isfinite(points_cam).all(axis=1) & (depth > 0.10)
        points_cam = points_cam[valid]
        points_source = points_source[valid]
        if len(points_cam) == 0:
            self.publish(payload, [], 'no_points_in_front_of_camera')
            return

        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        u = fx * (points_cam[:, 0] / points_cam[:, 2]) + cx
        v = fy * (points_cam[:, 1] / points_cam[:, 2]) + cy
        width = int(payload.get('image_width', 0) or info.width or 0)
        height = int(payload.get('image_height', 0) or info.height or 0)
        if width <= 1 or height <= 1:
            self.publish(payload, [], 'invalid_image_dimensions')
            return
        in_image = (u >= 0.0) & (u < width) & (v >= 0.0) & (v < height)
        points_cam = points_cam[in_image]
        points_source = points_source[in_image]
        u = np.rint(u[in_image]).astype(np.int32)
        v = np.rint(v[in_image]).astype(np.int32)

        fused: List[Dict[str, Any]] = []
        minimum = int(self.get_parameter('min_points').value)
        for det in extract_detections(msg.data):
            bbox = det.get('bbox', det.get('box_xyxy', []))
            if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
                continue
            x0, y0, x1, y1 = [int(round(float(x))) for x in bbox[:4]]
            x0, x1 = max(0, x0), min(width - 1, x1)
            y0, y1 = max(0, y0), min(height - 1, y1)
            inside = (u >= x0) & (u <= x1) & (v >= y0) & (v <= y1)

            poly = det.get('mask_polygon')
            if isinstance(poly, list) and len(poly) >= 3 and np.any(inside):
                mask = np.zeros((height, width), dtype=np.uint8)
                pts = np.asarray(poly, dtype=np.int32).reshape((-1, 2))
                cv2.fillPoly(mask, [pts], 1)
                idx = np.where(inside)[0]
                inside2 = np.zeros_like(inside)
                inside2[idx] = mask[v[idx], u[idx]] > 0
                inside = inside2

            cam_sel = points_cam[inside]
            src_sel = points_source[inside]
            cam_sel, src_sel = self.foreground(cam_sel, src_sel)
            if len(src_sel) < minimum:
                continue

            points_map = src_sel @ r_map.T + t_map
            low_p = float(self.get_parameter('extent_percentile_low').value)
            high_p = float(self.get_parameter('extent_percentile_high').value)
            lo = np.percentile(points_map, low_p, axis=0)
            hi = np.percentile(points_map, high_p, axis=0)
            keep = np.all((points_map >= lo) & (points_map <= hi), axis=1)
            points_map = points_map[keep]
            if len(points_map) < minimum:
                continue
            centroid = np.median(points_map, axis=0)
            lo = np.percentile(points_map, low_p, axis=0)
            hi = np.percentile(points_map, high_p, axis=0)
            out = dict(det)
            out.update({
                'geometry': 'registered_point_cloud',
                'geometry_source': 'lidar_camera_tf_projection',
                'cloud_frame_id': source_frame,
                'camera_frame_id': camera_frame,
                'centroid_map': [float(x) for x in centroid],
                'bbox3d': {'min': [float(x) for x in lo], 'max': [float(x) for x in hi]},
                'point_count': int(len(points_map)),
                'source': str(det.get('source', 'yolo_sam2')) + '+lidar_projection',
            })
            limit = int(self.get_parameter('max_points_output').value)
            idx = np.linspace(0, len(points_map) - 1, min(limit, len(points_map)), dtype=np.int64)
            out['points_map'] = [[float(x) for x in row] for row in points_map[idx]]
            fused.append(out)

        self.publish(payload, fused, '' if fused else 'no_valid_lidar_points_in_detection_masks')
        self.status('ok' if fused else 'no_fused_objects', fused=len(fused), cloud_points=int(len(points_source)))


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


if __name__ == '__main__':
    main()
