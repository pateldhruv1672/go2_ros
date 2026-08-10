#!/usr/bin/env python3

import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _extract_list_payload(msg: String) -> List[Dict[str, Any]]:
    try:
        data = json.loads(msg.data)
    except Exception:
        return []

    if isinstance(data, list):
        return [x for x in data if isinstance(x, dict)]

    if isinstance(data, dict):
        for key in ("detections", "objects", "items", "results"):
            if isinstance(data.get(key), list):
                return [x for x in data[key] if isinstance(x, dict)]
        return [data]

    return []


def _label(det: Dict[str, Any]) -> str:
    for key in ("label", "name", "class_name", "class", "object", "target"):
        if key in det and det[key] is not None:
            return str(det[key])
    return "object"


def _conf(det: Dict[str, Any]) -> Optional[float]:
    for key in ("confidence", "conf", "score", "probability"):
        if key in det:
            return _as_float(det[key], 0.0)
    return None


def _bbox(det: Dict[str, Any]) -> Optional[Tuple[int, int, int, int]]:
    candidates = []

    for key in ("bbox", "box", "xyxy", "bbox_xyxy", "pixel_bbox", "rect"):
        if key in det:
            candidates.append(det[key])

    if all(k in det for k in ("x1", "y1", "x2", "y2")):
        candidates.append([det["x1"], det["y1"], det["x2"], det["y2"]])

    if all(k in det for k in ("xmin", "ymin", "xmax", "ymax")):
        candidates.append([det["xmin"], det["ymin"], det["xmax"], det["ymax"]])

    for b in candidates:
        if isinstance(b, dict):
            if all(k in b for k in ("x1", "y1", "x2", "y2")):
                return (
                    int(_as_float(b["x1"])),
                    int(_as_float(b["y1"])),
                    int(_as_float(b["x2"])),
                    int(_as_float(b["y2"])),
                )
            if all(k in b for k in ("xmin", "ymin", "xmax", "ymax")):
                return (
                    int(_as_float(b["xmin"])),
                    int(_as_float(b["ymin"])),
                    int(_as_float(b["xmax"])),
                    int(_as_float(b["ymax"])),
                )

        if isinstance(b, (list, tuple)) and len(b) >= 4:
            return (
                int(_as_float(b[0])),
                int(_as_float(b[1])),
                int(_as_float(b[2])),
                int(_as_float(b[3])),
            )

    return None


def _polygon(det: Dict[str, Any]) -> Optional[np.ndarray]:
    for key in ("mask_polygon", "mask_contour", "segmentation", "polygon", "contour"):
        pts = det.get(key)
        if not isinstance(pts, list) or len(pts) < 3:
            continue

        parsed = []

        # Format: [[x,y], [x,y], ...]
        if all(isinstance(p, (list, tuple)) and len(p) >= 2 for p in pts):
            for p in pts:
                parsed.append([int(_as_float(p[0])), int(_as_float(p[1]))])

        # Format: [x1,y1,x2,y2,...]
        elif all(isinstance(p, (int, float)) for p in pts) and len(pts) >= 6:
            for i in range(0, len(pts) - 1, 2):
                parsed.append([int(_as_float(pts[i])), int(_as_float(pts[i + 1]))])

        if len(parsed) >= 3:
            return np.array(parsed, dtype=np.int32)

    return None


def _map_xy(det: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    key_pairs = [
        ("map_x", "map_y"),
        ("world_x", "world_y"),
        ("x_map", "y_map"),
        ("target_x", "target_y"),
        ("x", "y"),
    ]

    for kx, ky in key_pairs:
        if kx in det and ky in det:
            return _as_float(det[kx]), _as_float(det[ky])

    pose = det.get("pose")
    if isinstance(pose, dict):
        if "x" in pose and "y" in pose:
            return _as_float(pose["x"]), _as_float(pose["y"])

    return None


class ObjectRvizOverlayNode(Node):
    def __init__(self):
        super().__init__("object_rviz_overlay_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/object_explorer/detections")
        self.declare_parameter("memory_topic", "/object_explorer/memory")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("annotated_image_topic", "/object_explorer/annotated_image")
        self.declare_parameter("marker_topic", "/object_explorer/detection_markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_memory_markers", True)

        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        memory_topic = self.get_parameter("memory_topic").value
        plan_topic = self.get_parameter("plan_topic").value
        annotated_topic = self.get_parameter("annotated_image_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.bridge = CvBridge()
        self.last_detections: List[Dict[str, Any]] = []
        self.last_memory: List[Dict[str, Any]] = []
        self.last_plan: Optional[Path] = None
        self.last_detection_time = 0.0

        self.image_pub = self.create_publisher(Image, annotated_topic, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, detections_topic, self.on_detections, 10)
        self.create_subscription(String, memory_topic, self.on_memory, 10)
        self.create_subscription(Path, plan_topic, self.on_plan, 10)

        self.timer = self.create_timer(0.5, self.publish_markers)

        self.get_logger().info(
            f"RViz overlay ready: image={image_topic}, detections={detections_topic}, "
            f"annotated={annotated_topic}, markers={marker_topic}"
        )

    def on_detections(self, msg: String):
        detections = _extract_list_payload(msg)
        self.last_detections = detections
        self.last_detection_time = time.time()
        self.publish_markers()

    def on_memory(self, msg: String):
        self.last_memory = _extract_list_payload(msg)

    def on_plan(self, msg: Path):
        self.last_plan = msg

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        h, w = frame.shape[:2]

        # Fade stale detections after 2 seconds.
        detections = self.last_detections if time.time() - self.last_detection_time < 2.0 else []

        overlay = frame.copy()

        for det in detections:
            label = _label(det)
            conf = _conf(det)
            text = f"{label}"
            if conf is not None:
                text += f" {conf:.2f}"

            box = _bbox(det)
            poly = _polygon(det)

            if poly is not None:
                cv2.fillPoly(overlay, [poly], (0, 255, 255))
                cv2.polylines(frame, [poly], True, (0, 180, 255), 2)

            if box is not None:
                x1, y1, x2, y2 = box
                x1 = max(0, min(w - 1, x1))
                x2 = max(0, min(w - 1, x2))
                y1 = max(0, min(h - 1, y1))
                y2 = max(0, min(h - 1, y2))

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    text,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # Blend segmentation masks.
        frame = cv2.addWeighted(overlay, 0.28, frame, 0.72, 0.0)

        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self.image_pub.publish(out)

    def make_delete_all(self) -> Marker:
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(self.make_delete_all())

        map_frame = str(self.get_parameter("map_frame").value)
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        # Current detections as bright markers.
        for det in self.last_detections:
            xy = _map_xy(det)
            if xy is None:
                continue

            x, y = xy
            label = _label(det)
            conf = _conf(det)

            sphere = Marker()
            sphere.header.frame_id = map_frame
            sphere.header.stamp = stamp
            sphere.ns = "live_detections"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.35
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = map_frame
            text.header.stamp = stamp
            text.ns = "live_detection_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.75
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = label if conf is None else f"{label} {conf:.2f}"
            marker_array.markers.append(text)

        # Memory markers as smaller blue-ish markers.
        if bool(self.get_parameter("publish_memory_markers").value):
            for det in self.last_memory:
                xy = _map_xy(det)
                if xy is None:
                    continue

                x, y = xy
                label = _label(det)

                cube = Marker()
                cube.header.frame_id = map_frame
                cube.header.stamp = stamp
                cube.ns = "memory_objects"
                cube.id = marker_id
                marker_id += 1
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = x
                cube.pose.position.y = y
                cube.pose.position.z = 0.15
                cube.pose.orientation.w = 1.0
                cube.scale.x = 0.18
                cube.scale.y = 0.18
                cube.scale.z = 0.18
                cube.color.r = 0.2
                cube.color.g = 0.5
                cube.color.b = 1.0
                cube.color.a = 0.75
                marker_array.markers.append(cube)

                text = Marker()
                text.header.frame_id = map_frame
                text.header.stamp = stamp
                text.ns = "memory_labels"
                text.id = marker_id
                marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = 0.45
                text.pose.orientation.w = 1.0
                text.scale.z = 0.16
                text.color.r = 0.7
                text.color.g = 0.9
                text.color.b = 1.0
                text.color.a = 0.95
                text.text = label
                marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = ObjectRvizOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
