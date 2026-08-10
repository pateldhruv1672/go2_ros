from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import clean_label, target_from_text


class TargetConditionedDetector(Node):
    """Language-conditioned 2D detector for the active SysNav target.

    The node consumes the structured target specification emitted by the
    supervisor. Its output is deliberately marked as 2D geometry. A calibrated
    image/point-cloud projector should add map-frame 3D points before persistent
    object fusion. LaserScan projection in the mapper remains an explicit
    degraded fallback, not an equivalent replacement for SysNav's registered
    point-cloud mapping path.
    """

    def __init__(self) -> None:
        super().__init__("go2_target_conditioned_detector")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("goal_topic", "/go2_vln/goal")
        self.declare_parameter("target_spec_topic", "/go2_vln/target_spec")
        self.declare_parameter("detections_topic", "/go2_vln/target_detections_2d")
        self.declare_parameter("model_name", "IDEA-Research/grounding-dino-tiny")
        self.declare_parameter("threshold", 0.30)
        self.declare_parameter("period_sec", 0.70)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("max_detections", 12)

        self.bridge = CvBridge()
        self.target = ""
        self.target_spec: Dict[str, Any] = {}
        self.latest_image: Optional[Image] = None
        self.processor = None
        self.model = None
        self.backend = "unloaded"
        self.loading = False
        self.last_run = 0.0

        self.pub = self.create_publisher(
            String, str(self.get_parameter("detections_topic").value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, str(self.get_parameter("goal_topic").value), self.on_goal, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_spec_topic").value),
            self.on_target_spec,
            10,
        )
        self.create_timer(0.10, self.tick)
        self.get_logger().info("target-conditioned detector ready; model loads lazily")

    def ensure_model_loading(self) -> None:
        if self.target and self.model is None and not self.loading and self.backend != "unavailable":
            self.loading = True
            threading.Thread(target=self.load_model, daemon=True).start()

    def on_goal(self, msg: String) -> None:
        # Heuristic fallback while the structured decomposition is pending.
        candidate = target_from_text(msg.data)
        if candidate:
            self.target = candidate
        self.ensure_model_loading()

    def on_target_spec(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        self.target_spec = payload
        candidate = clean_label(str(payload.get("target_object", "")))
        if candidate:
            self.target = candidate
        self.ensure_model_loading()

    def on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def load_model(self) -> None:
        try:
            import torch
            from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

            name = str(self.get_parameter("model_name").value)
            self.processor = AutoProcessor.from_pretrained(name)
            self.model = AutoModelForZeroShotObjectDetection.from_pretrained(name)
            requested = str(self.get_parameter("device").value)
            device = requested if requested.startswith("cuda") and torch.cuda.is_available() else "cpu"
            self.model.to(device)
            self.model.eval()
            self.backend = "grounding_dino" if "dino" in name.lower() else "open_vocab"
            self.get_logger().info(f"open-vocabulary model loaded: {name} on {device}")
        except Exception as exc:
            self.processor = None
            self.model = None
            self.backend = "unavailable"
            self.get_logger().error(
                "open-vocabulary detector unavailable; install transformers/torch or "
                f"connect an existing detector: {type(exc).__name__}: {exc}"
            )
        finally:
            self.loading = False

    def publish(self, payload: Dict[str, Any]) -> None:
        payload.setdefault("stamp_sec", time.time())
        payload.setdefault("target", self.target)
        payload.setdefault("target_spec", self.target_spec)
        payload.setdefault("backend", self.backend)
        payload.setdefault("geometry", "image_2d")
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def build_prompt(self) -> str:
        parts = [self.target.replace("_", " ")]
        for key in ("attribute_condition", "spatial_condition"):
            value = str(self.target_spec.get(key, "")).strip()
            if value and value.lower() not in ("none", "null", "unknown"):
                parts.append(value)
        return ". ".join(parts) + "."

    def tick(self) -> None:
        if not self.target or self.latest_image is None:
            return
        period = max(0.10, float(self.get_parameter("period_sec").value))
        now = time.time()
        if now - self.last_run < period:
            return
        self.last_run = now
        if self.model is None or self.processor is None:
            if self.backend == "unavailable":
                self.publish({"success": False, "detections": [], "error": "model_unavailable"})
            return
        msg = self.latest_image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            from PIL import Image as PILImage
            import cv2
            import torch

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil = PILImage.fromarray(rgb)
            inputs = self.processor(images=pil, text=self.build_prompt(), return_tensors="pt")
            device = next(self.model.parameters()).device
            inputs = {k: v.to(device) if hasattr(v, "to") else v for k, v in inputs.items()}
            with torch.no_grad():
                outputs = self.model(**inputs)
            target_sizes = torch.tensor([pil.size[::-1]], device=device)
            threshold = float(self.get_parameter("threshold").value)
            if hasattr(self.processor, "post_process_grounded_object_detection"):
                result = self.processor.post_process_grounded_object_detection(
                    outputs,
                    inputs.get("input_ids"),
                    box_threshold=threshold,
                    text_threshold=threshold,
                    target_sizes=target_sizes,
                )[0]
            else:
                result = self.processor.post_process_object_detection(
                    outputs=outputs, threshold=threshold, target_sizes=target_sizes
                )[0]
            boxes = result.get("boxes", [])
            scores = result.get("scores", [])
            labels = result.get("text_labels", result.get("labels", []))
            detections: List[Dict[str, Any]] = []
            limit = int(self.get_parameter("max_detections").value)
            for box, score, label in zip(boxes, scores, labels):
                value = float(score)
                if value < threshold:
                    continue
                label_text = clean_label(str(label)) if isinstance(label, str) else self.target
                detections.append(
                    {
                        "label": label_text or self.target,
                        "confidence": value,
                        "bbox": [float(v) for v in box.detach().cpu().tolist()],
                        "track_id": -1,
                        "source": self.backend,
                        "geometry": "image_2d",
                    }
                )
                if len(detections) >= limit:
                    break
            self.publish(
                {
                    "success": True,
                    "image_width": int(msg.width),
                    "image_height": int(msg.height),
                    "image_stamp": {
                        "sec": int(msg.header.stamp.sec),
                        "nanosec": int(msg.header.stamp.nanosec),
                    },
                    "image_frame_id": str(msg.header.frame_id),
                    "detections": detections,
                }
            )
        except Exception as exc:
            self.publish(
                {
                    "success": False,
                    "detections": [],
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetConditionedDetector()
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
