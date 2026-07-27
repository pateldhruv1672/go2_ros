#!/usr/bin/env python3

import json
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def iou_xyxy(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter

    if denom <= 1e-6:
        return 0.0
    return float(inter / denom)


def contour_from_mask(mask: np.ndarray, max_points: int = 80) -> List[List[int]]:
    if mask is None:
        return []

    m = mask.astype(np.uint8)
    if m.max() <= 1:
        m = m * 255

    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 3:
        return []

    epsilon = 0.01 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)

    if len(approx) > max_points:
        idx = np.linspace(0, len(approx) - 1, max_points).astype(int)
        approx = approx[idx]

    return [[int(x), int(y)] for x, y in approx]


class SAM2TrackerOverlayNode(Node):
    def __init__(self):
        super().__init__("sam2_tracker_overlay_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("annotated_image_topic", "/object_explorer/annotated_image")
        self.declare_parameter("sam2_detections_topic", "/object_explorer/sam2_detections")

        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("yolo_conf", 0.35)
        self.declare_parameter("max_detections", 8)
        self.declare_parameter("target_classes", "person,chair,dining table,tv,laptop,backpack,bottle,cup,book")

        self.declare_parameter("enable_sam2", True)
        self.declare_parameter("sam2_model", "sam2_t.pt")
        self.declare_parameter("sam2_imgsz", 512)

        self.declare_parameter("inference_period_sec", 0.70)
        self.declare_parameter("track_iou_threshold", 0.25)
        self.declare_parameter("track_ttl_sec", 3.0)

        image_topic = str(self.get_parameter("image_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        detections_topic = str(self.get_parameter("sam2_detections_topic").value)

        self.bridge = CvBridge()
        self.last_inference_t = 0.0
        self.next_track_id = 1
        self.tracks: Dict[int, Dict[str, Any]] = {}

        self.yolo = None
        self.sam = None
        self.sam_dynamic = None
        self.sam_mode = "disabled"

        self.image_pub = self.create_publisher(Image, annotated_topic, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(String, detections_topic, 10)

        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

        self.load_models()

        self.get_logger().info(
            f"SAM2 tracker overlay ready: image={image_topic}, annotated={annotated_topic}, "
            f"detections={detections_topic}, sam_mode={self.sam_mode}"
        )

    def load_models(self):
        yolo_model = str(self.get_parameter("yolo_model").value)
        enable_sam2 = bool(self.get_parameter("enable_sam2").value)
        sam2_model = str(self.get_parameter("sam2_model").value)

        try:
            from ultralytics import YOLO
            self.yolo = YOLO(yolo_model)
            self.get_logger().info(f"YOLO loaded: {yolo_model}")
        except Exception as exc:
            self.yolo = None
            self.get_logger().error(f"Could not load YOLO model={yolo_model}: {exc}")

        if not enable_sam2:
            self.sam_mode = "disabled"
            self.get_logger().warn("SAM2 disabled by parameter; publishing YOLO-only overlay")
            return

        # Best path: Ultralytics SAM2 dynamic predictor with object IDs / memory.
        try:
            from ultralytics.models.sam import SAM2DynamicInteractivePredictor

            overrides = {
                "conf": 0.25,
                "task": "segment",
                "mode": "predict",
                "imgsz": int(self.get_parameter("sam2_imgsz").value),
                "model": sam2_model,
                "save": False,
                "verbose": False,
            }
            self.sam_dynamic = SAM2DynamicInteractivePredictor(overrides=overrides, max_obj_num=20)
            self.sam_mode = "dynamic"
            self.get_logger().info(f"SAM2 dynamic tracker loaded: {sam2_model}")
            return
        except Exception as exc:
            self.sam_dynamic = None
            self.get_logger().warn(f"SAM2 dynamic tracker unavailable, trying prompt SAM fallback: {exc}")

        # Fallback: per-frame SAM2 prompt segmentation from YOLO boxes.
        try:
            from ultralytics import SAM

            self.sam = SAM(sam2_model)
            self.sam_mode = "prompt"
            self.get_logger().info(f"SAM2 prompt segmenter loaded: {sam2_model}")
            return
        except Exception as exc:
            self.sam = None
            self.sam_mode = "yolo_only"
            self.get_logger().warn(f"SAM2 unavailable; using YOLO-only overlay: {exc}")

    def allowed_class(self, name: str) -> bool:
        raw = str(self.get_parameter("target_classes").value).strip()
        if not raw:
            return True
        allowed = {x.strip().lower() for x in raw.split(",") if x.strip()}
        return name.lower() in allowed

    def run_yolo(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        if self.yolo is None:
            return []

        conf = float(self.get_parameter("yolo_conf").value)
        max_det = int(self.get_parameter("max_detections").value)

        try:
            results = self.yolo.predict(frame, conf=conf, verbose=False, max_det=max_det)
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return []

        dets: List[Dict[str, Any]] = []

        if not results:
            return dets

        r = results[0]
        boxes = getattr(r, "boxes", None)
        names = getattr(r, "names", {}) or {}

        if boxes is None:
            return dets

        for i, box in enumerate(boxes):
            try:
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float).tolist()
                cls_id = int(box.cls[0].detach().cpu().item())
                score = float(box.conf[0].detach().cpu().item())
                label = str(names.get(cls_id, cls_id))
            except Exception:
                continue

            if not self.allowed_class(label):
                continue

            dets.append(
                {
                    "label": label,
                    "confidence": score,
                    "bbox": [float(v) for v in xyxy],
                }
            )

        return dets

    def assign_track_ids(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        now = time.time()
        ttl = float(self.get_parameter("track_ttl_sec").value)
        threshold = float(self.get_parameter("track_iou_threshold").value)

        # Drop stale tracks.
        self.tracks = {
            tid: tr for tid, tr in self.tracks.items()
            if now - float(tr.get("last_seen", 0.0)) <= ttl
        }

        used_tracks = set()

        for det in detections:
            box = det["bbox"]
            label = det["label"]

            best_tid = None
            best_iou = 0.0

            for tid, tr in self.tracks.items():
                if tid in used_tracks:
                    continue
                if tr.get("label") != label:
                    continue

                score = iou_xyxy(box, tr.get("bbox", box))
                if score > best_iou:
                    best_iou = score
                    best_tid = tid

            if best_tid is not None and best_iou >= threshold:
                track_id = best_tid
            else:
                track_id = self.next_track_id
                self.next_track_id += 1

            used_tracks.add(track_id)
            det["track_id"] = int(track_id)

            self.tracks[track_id] = {
                "bbox": box,
                "label": label,
                "last_seen": now,
                "confidence": det.get("confidence", 0.0),
            }

        return detections

    def run_sam2(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not detections:
            return detections

        boxes = [d["bbox"] for d in detections]
        obj_ids = [int(d["track_id"]) for d in detections]

        if self.sam_mode == "dynamic" and self.sam_dynamic is not None:
            try:
                results = self.sam_dynamic(
                    source=frame,
                    bboxes=boxes,
                    obj_ids=obj_ids,
                    update_memory=True,
                )
                return self.attach_masks_from_results(detections, results, frame.shape[:2])
            except Exception as exc:
                self.get_logger().warn(f"SAM2 dynamic inference failed: {exc}")
                return detections

        if self.sam_mode == "prompt" and self.sam is not None:
            try:
                results = self.sam.predict(frame, bboxes=boxes, verbose=False)
                return self.attach_masks_from_results(detections, results, frame.shape[:2])
            except Exception as exc:
                self.get_logger().warn(f"SAM2 prompt inference failed: {exc}")
                return detections

        return detections

    def attach_masks_from_results(
        self,
        detections: List[Dict[str, Any]],
        results: Any,
        hw: Tuple[int, int],
    ) -> List[Dict[str, Any]]:
        if not results:
            return detections

        r = results[0] if isinstance(results, list) else results
        masks = getattr(r, "masks", None)

        if masks is None:
            return detections

        # Preferred: polygon output.
        xy_polys = getattr(masks, "xy", None)
        if xy_polys is not None:
            for det, poly in zip(detections, xy_polys):
                try:
                    arr = np.asarray(poly, dtype=np.float32)
                    if arr.ndim == 2 and arr.shape[0] >= 3:
                        det["mask_polygon"] = [[int(x), int(y)] for x, y in arr.tolist()]
                except Exception:
                    pass
            return detections

        # Fallback: binary masks.
        data = getattr(masks, "data", None)
        if data is not None:
            try:
                mask_np = data.detach().cpu().numpy()
                h, w = hw
                for det, m in zip(detections, mask_np):
                    if m.shape[0] != h or m.shape[1] != w:
                        m = cv2.resize(m.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)
                    poly = contour_from_mask(m > 0.5)
                    if poly:
                        det["mask_polygon"] = poly
            except Exception:
                pass

        return detections

    def draw_overlay(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        out = frame.copy()
        mask_layer = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            label = str(det.get("label", "object"))
            conf = float(det.get("confidence", 0.0))
            tid = int(det.get("track_id", -1))

            poly = det.get("mask_polygon")
            if isinstance(poly, list) and len(poly) >= 3:
                pts = np.asarray(poly, dtype=np.int32)
                cv2.fillPoly(mask_layer, [pts], (0, 255, 255))
                cv2.polylines(out, [pts], True, (0, 180, 255), 2)

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)

            text = f"id{tid} {label} {conf:.2f}"
            cv2.putText(
                out,
                text,
                (x1, max(24, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        out = cv2.addWeighted(mask_layer, 0.30, out, 0.70, 0.0)

        mode_text = f"SAM2 mode: {self.sam_mode}"
        cv2.putText(
            out,
            mode_text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return out

    def publish_image(self, frame: np.ndarray, header):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header = header
        self.image_pub.publish(msg)

    def publish_detections(self, detections: List[Dict[str, Any]]):
        msg = String()
        msg.data = json.dumps(
            {
                "stamp_sec": time.time(),
                "sam2_mode": self.sam_mode,
                "detections": detections,
            }
        )
        self.det_pub.publish(msg)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        now = time.time()
        period = float(self.get_parameter("inference_period_sec").value)

        if now - self.last_inference_t >= period:
            self.last_inference_t = now

            detections = self.run_yolo(frame)
            detections = self.assign_track_ids(detections)
            detections = self.run_sam2(frame, detections)

            # Save track metadata for drawing between inference ticks.
            for det in detections:
                tid = int(det.get("track_id", -1))
                if tid > 0 and tid in self.tracks:
                    self.tracks[tid]["mask_polygon"] = det.get("mask_polygon")
                    self.tracks[tid]["confidence"] = det.get("confidence", 0.0)

            self.publish_detections(detections)

        # Draw last known tracks on every frame so RViz always has an image.
        active: List[Dict[str, Any]] = []
        ttl = float(self.get_parameter("track_ttl_sec").value)

        for tid, tr in self.tracks.items():
            if now - float(tr.get("last_seen", 0.0)) <= ttl:
                active.append(
                    {
                        "track_id": tid,
                        "label": tr.get("label", "object"),
                        "confidence": tr.get("confidence", 0.0),
                        "bbox": tr.get("bbox", [0, 0, 1, 1]),
                        "mask_polygon": tr.get("mask_polygon"),
                    }
                )

        annotated = self.draw_overlay(frame, active)
        self.publish_image(annotated, msg.header)


def main():
    rclpy.init()
    node = SAM2TrackerOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
