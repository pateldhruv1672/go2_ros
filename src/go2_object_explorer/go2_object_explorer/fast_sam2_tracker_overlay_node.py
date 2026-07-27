#!/usr/bin/env python3

import json
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def sensor_qos(depth: int = 1):
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


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
    return float(inter / denom) if denom > 1e-6 else 0.0


def contour_from_mask(mask: np.ndarray, max_points: int = 120) -> List[List[int]]:
    m = mask.astype(np.uint8)
    if m.max() <= 1:
        m *= 255

    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    c = max(contours, key=cv2.contourArea)
    if len(c) < 3:
        return []

    eps = 0.006 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, eps, True).reshape(-1, 2)

    if len(approx) > max_points:
        idx = np.linspace(0, len(approx) - 1, max_points).astype(int)
        approx = approx[idx]

    return [[int(x), int(y)] for x, y in approx]


class FastSAM2TrackerOverlayNode(Node):
    def __init__(self):
        super().__init__("fast_sam2_tracker_overlay_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("annotated_image_topic", "/object_explorer/annotated_image")
        self.declare_parameter("detections_topic", "/object_explorer/sam2_detections")

        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("half", True)

        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_conf", 0.18)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("target_classes", "")

        self.declare_parameter("enable_sam2", True)
        self.declare_parameter("sam2_model", "sam2_t.pt")
        self.declare_parameter("sam2_imgsz", 512)
        self.declare_parameter("sam2_every_n", 1)

        self.declare_parameter("inference_period_sec", 0.20)
        self.declare_parameter("track_iou_threshold", 0.22)
        self.declare_parameter("track_ttl_sec", 2.0)

        self.bridge = CvBridge()
        self.yolo = None
        self.sam = None
        self.sam2_ready = False

        self.last_inference_time = 0.0
        self.frame_count = 0
        self.next_track_id = 1
        self.tracks: Dict[int, Dict[str, Any]] = {}
        self.last_detections: List[Dict[str, Any]] = []

        image_topic = str(self.get_parameter("image_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        self.image_pub = self.create_publisher(Image, annotated_topic, sensor_qos(1))
        self.det_pub = self.create_publisher(String, detections_topic, 10)

        self.create_subscription(Image, image_topic, self.on_image, sensor_qos(1))

        self.load_models()

        self.get_logger().info(
            f"Fast SAM2 overlay ready image={image_topic} annotated={annotated_topic} "
            f"detections={detections_topic} device={self.get_parameter('device').value}"
        )

    def load_models(self):
        device = str(self.get_parameter("device").value)
        yolo_model = str(self.get_parameter("yolo_model").value)
        sam2_model = str(self.get_parameter("sam2_model").value)

        try:
            import torch
            self.get_logger().info(
                f"torch cuda_available={torch.cuda.is_available()} "
                f"cuda_device={torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none'}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not inspect torch CUDA: {exc}")

        try:
            from ultralytics import YOLO
            self.yolo = YOLO(yolo_model)
            self.get_logger().info(f"YOLO loaded: {yolo_model}")
        except Exception as exc:
            self.yolo = None
            self.get_logger().error(f"YOLO load failed: {exc}")

        if not bool(self.get_parameter("enable_sam2").value):
            self.get_logger().warn("SAM2 disabled; YOLO boxes only")
            return

        try:
            from ultralytics import SAM
            self.sam = SAM(sam2_model)
            self.sam2_ready = True
            self.get_logger().info(f"SAM2 prompt segmenter loaded: {sam2_model} on {device}")
        except Exception as exc:
            self.sam = None
            self.sam2_ready = False
            self.get_logger().warn(f"SAM2 load failed; YOLO-only overlay: {exc}")

    def allowed(self, label: str) -> bool:
        raw = str(self.get_parameter("target_classes").value).strip()
        if raw == "":
            return True
        allowed = {x.strip().lower() for x in raw.split(",") if x.strip()}
        return label.lower() in allowed

    def run_yolo(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        if self.yolo is None:
            return []

        device = str(self.get_parameter("device").value)
        half = bool(self.get_parameter("half").value)
        imgsz = int(self.get_parameter("yolo_imgsz").value)
        conf = float(self.get_parameter("yolo_conf").value)
        max_det = int(self.get_parameter("max_detections").value)

        try:
            results = self.yolo.predict(
                frame,
                conf=conf,
                imgsz=imgsz,
                max_det=max_det,
                device=device,
                half=half,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"YOLO failed on {device}; retrying CPU: {exc}")
            try:
                results = self.yolo.predict(
                    frame,
                    conf=conf,
                    imgsz=imgsz,
                    max_det=max_det,
                    device="cpu",
                    half=False,
                    verbose=False,
                )
            except Exception as exc2:
                self.get_logger().warn(f"YOLO CPU fallback failed: {exc2}")
                return []

        if not results:
            return []

        r = results[0]
        boxes = getattr(r, "boxes", None)
        names = getattr(r, "names", {}) or {}
        if boxes is None:
            return []

        dets = []
        for b in boxes:
            try:
                xyxy = b.xyxy[0].detach().cpu().numpy().astype(float).tolist()
                cls_id = int(b.cls[0].detach().cpu().item())
                score = float(b.conf[0].detach().cpu().item())
                label = str(names.get(cls_id, cls_id))
            except Exception:
                continue

            if not self.allowed(label):
                continue

            dets.append(
                {
                    "label": label,
                    "confidence": score,
                    "bbox": [float(v) for v in xyxy],
                }
            )

        return dets

    def assign_tracks(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        now = time.time()
        ttl = float(self.get_parameter("track_ttl_sec").value)
        iou_threshold = float(self.get_parameter("track_iou_threshold").value)

        self.tracks = {
            tid: tr for tid, tr in self.tracks.items()
            if now - float(tr.get("last_seen", 0.0)) <= ttl
        }

        used = set()

        for det in detections:
            box = det["bbox"]
            label = det["label"]

            best_tid = None
            best_iou = 0.0

            for tid, tr in self.tracks.items():
                if tid in used:
                    continue
                if tr.get("label") != label:
                    continue

                score = iou_xyxy(box, tr.get("bbox", box))
                if score > best_iou:
                    best_iou = score
                    best_tid = tid

            if best_tid is not None and best_iou >= iou_threshold:
                tid = best_tid
            else:
                tid = self.next_track_id
                self.next_track_id += 1

            used.add(tid)
            det["track_id"] = int(tid)

            old_mask = self.tracks.get(tid, {}).get("mask_polygon")

            self.tracks[tid] = {
                "track_id": tid,
                "label": label,
                "confidence": det["confidence"],
                "bbox": box,
                "mask_polygon": old_mask,
                "last_seen": now,
            }

        return detections

    def run_sam2_prompt(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not self.sam2_ready or self.sam is None or not detections:
            return detections

        self.frame_count += 1
        every_n = max(1, int(self.get_parameter("sam2_every_n").value))
        if self.frame_count % every_n != 0:
            for det in detections:
                tid = int(det.get("track_id", -1))
                if tid in self.tracks:
                    det["mask_polygon"] = self.tracks[tid].get("mask_polygon")
            return detections

        device = str(self.get_parameter("device").value)
        imgsz = int(self.get_parameter("sam2_imgsz").value)
        boxes = [d["bbox"] for d in detections]

        try:
            results = self.sam.predict(
                frame,
                bboxes=boxes,
                device=device,
                imgsz=imgsz,
                retina_masks=True,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"SAM2 prompt failed on {device}; retrying CPU: {exc}")
            try:
                results = self.sam.predict(
                    frame,
                    bboxes=boxes,
                    device="cpu",
                    imgsz=imgsz,
                    retina_masks=True,
                    verbose=False,
                )
            except Exception as exc2:
                self.get_logger().warn(f"SAM2 CPU fallback failed: {exc2}")
                return detections

        if not results:
            return detections

        r = results[0]
        masks = getattr(r, "masks", None)
        if masks is None:
            return detections

        # Prefer polygons if Ultralytics provides them.
        xy_polys = getattr(masks, "xy", None)
        if xy_polys is not None:
            for det, poly in zip(detections, xy_polys):
                arr = np.asarray(poly, dtype=np.float32)
                if arr.ndim == 2 and arr.shape[0] >= 3:
                    det["mask_polygon"] = [[int(x), int(y)] for x, y in arr.tolist()]
            return detections

        data = getattr(masks, "data", None)
        if data is None:
            return detections

        try:
            mask_np = data.detach().cpu().numpy()
            h, w = frame.shape[:2]

            for det, m in zip(detections, mask_np):
                if m.shape[:2] != (h, w):
                    m = cv2.resize(m.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)

                poly = contour_from_mask(m > 0.5)
                if poly:
                    det["mask_polygon"] = poly
        except Exception as exc:
            self.get_logger().warn(f"mask extraction failed: {exc}")

        return detections

    def draw(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        out = frame.copy()
        mask_layer = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det.get("bbox", [0, 0, 1, 1])]
            label = str(det.get("label", "object"))
            conf = float(det.get("confidence", 0.0))
            tid = int(det.get("track_id", -1))

            poly = det.get("mask_polygon")
            if isinstance(poly, list) and len(poly) >= 3:
                pts = np.asarray(poly, dtype=np.int32)
                cv2.fillPoly(mask_layer, [pts], (0, 255, 255))
                cv2.polylines(out, [pts], True, (0, 180, 255), 2)

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                out,
                f"id{tid} {label} {conf:.2f}",
                (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        out = cv2.addWeighted(mask_layer, 0.32, out, 0.68, 0.0)

        cv2.putText(
            out,
            f"YOLO+SAM2 prompt GPU | det={len(detections)} | tracks={len(self.tracks)}",
            (10, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return out

    def publish_detections(self, detections: List[Dict[str, Any]]):
        msg = String()
        msg.data = json.dumps(
            {
                "stamp_sec": time.time(),
                "sam2_mode": "prompt_gpu" if self.sam2_ready else "yolo_only",
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

        if now - self.last_inference_time >= period:
            self.last_inference_time = now

            dets = self.run_yolo(frame)
            dets = self.assign_tracks(dets)
            dets = self.run_sam2_prompt(frame, dets)

            for det in dets:
                tid = int(det.get("track_id", -1))
                if tid in self.tracks:
                    self.tracks[tid]["mask_polygon"] = det.get("mask_polygon")
                    self.tracks[tid]["bbox"] = det.get("bbox")
                    self.tracks[tid]["confidence"] = det.get("confidence", 0.0)
                    self.tracks[tid]["last_seen"] = now

            self.last_detections = dets
            self.publish_detections(dets)

        # Draw last tracks every incoming frame so RViz stays smooth.
        ttl = float(self.get_parameter("track_ttl_sec").value)
        active = []

        for tid, tr in self.tracks.items():
            if now - float(tr.get("last_seen", 0.0)) <= ttl:
                active.append(dict(tr))

        annotated = self.draw(frame, active)
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.image_pub.publish(out)


def main():
    rclpy.init()
    node = FastSAM2TrackerOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
