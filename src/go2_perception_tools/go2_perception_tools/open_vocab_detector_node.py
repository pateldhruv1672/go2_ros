from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass
class Detection:
    label: str
    score: float
    box_xyxy: List[float]
    backend: str

    def to_dict(self) -> Dict[str, Any]:
        return {"label": self.label, "score": self.score, "box_xyxy": self.box_xyxy, "backend": self.backend}


class OpenVocabularyDetector:
    """Optional GroundingDINO/OWL-ViT detector with a safe heuristic fallback.

    Install one of these in the project venv for real open-vocabulary detection:
      * transformers + torch with GroundingDINO model IDs
      * transformers + torch with OWLv2/OWL-ViT model IDs

    The fallback is deliberately conservative and only emits image-quality hints;
    it is useful for plumbing tests but marked backend=heuristic.
    """

    def __init__(self, backend: str = "grounding_dino", model_name: str = "IDEA-Research/grounding-dino-tiny", prompts: Optional[List[str]] = None, threshold: float = 0.25):
        self.backend = backend.lower()
        self.model_name = model_name
        self.prompts = prompts or ["door", "person", "chair", "table", "poster", "sign", "hallway", "robot", "obstacle"]
        self.threshold = threshold
        self.processor = None
        self.model = None
        self._loaded_backend = "heuristic"
        self._load_optional_model()

    def _load_optional_model(self) -> None:
        if self.backend in {"off", "disabled", "heuristic"}:
            return
        try:
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection  # type: ignore
            self.processor = AutoProcessor.from_pretrained(self.model_name)
            self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_name)
            self._loaded_backend = "grounding_dino" if "dino" in self.model_name.lower() else "owlv2"
        except Exception:
            self.processor = None
            self.model = None
            self._loaded_backend = "heuristic"

    @staticmethod
    def _image_to_pil(msg: Image):
        try:
            from PIL import Image as PILImage  # type: ignore
        except Exception:
            return None
        enc = (msg.encoding or "").lower()
        data = bytes(msg.data)
        try:
            if enc in {"rgb8", "bgr8"}:
                mode = "RGB"
                img = PILImage.frombytes(mode, (msg.width, msg.height), data)
                if enc == "bgr8":
                    r, g, b = img.split()
                    img = PILImage.merge("RGB", (b, g, r))
                return img
            if enc in {"mono8", "8uc1"}:
                return PILImage.frombytes("L", (msg.width, msg.height), data).convert("RGB")
        except Exception:
            return None
        return None

    def detect(self, msg: Image) -> Dict[str, Any]:
        pil = self._image_to_pil(msg)
        if pil is None:
            return {"success": False, "backend": "none", "detections": [], "error": "could_not_decode_image"}
        if self.processor is not None and self.model is not None:
            try:
                return self._detect_transformers(pil)
            except Exception as exc:
                return {"success": False, "backend": self._loaded_backend, "detections": [], "error": str(exc)}
        return self._detect_heuristic(pil)

    def _detect_transformers(self, pil) -> Dict[str, Any]:
        import torch  # type: ignore
        text = ". ".join(self.prompts) + "."
        inputs = self.processor(images=pil, text=text, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
        target_sizes = torch.tensor([pil.size[::-1]])
        if hasattr(self.processor, "post_process_grounded_object_detection"):
            results = self.processor.post_process_grounded_object_detection(outputs, inputs.input_ids, box_threshold=self.threshold, text_threshold=self.threshold, target_sizes=target_sizes)
        else:
            results = self.processor.post_process_object_detection(outputs=outputs, threshold=self.threshold, target_sizes=target_sizes)
        detections: List[Detection] = []
        first = results[0] if results else {}
        boxes = first.get("boxes", [])
        scores = first.get("scores", [])
        labels = first.get("labels", []) or first.get("text_labels", [])
        for box, score, label in zip(boxes, scores, labels):
            value = float(score)
            if value < self.threshold:
                continue
            if not isinstance(label, str):
                try:
                    label = self.model.config.id2label[int(label)]
                except Exception:
                    label = str(label)
            detections.append(Detection(str(label), value, [float(x) for x in box.tolist()], self._loaded_backend))
        return {"success": True, "backend": self._loaded_backend, "model": self.model_name, "detections": [d.to_dict() for d in detections[:20]], "prompt": self.prompts}

    def _detect_heuristic(self, pil) -> Dict[str, Any]:
        # Simple image quality / motion-plumbing fallback. It does not pretend to
        # recognize objects; it only adds a low-confidence scene hint.
        gray = pil.convert("L")
        hist = gray.histogram()
        total = max(1, sum(hist))
        mean = sum(i * v for i, v in enumerate(hist)) / total
        label = "low_light_scene" if mean < 55 else "indoor_scene_candidate"
        det = Detection(label, 0.2, [0.0, 0.0, float(pil.size[0]), float(pil.size[1])], "heuristic")
        return {"success": True, "backend": "heuristic", "detections": [det.to_dict()], "note": "install transformers+torch for GroundingDINO/OWLv2"}


class OpenVocabularyDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_open_vocab_detector")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("backend", "grounding_dino")
        self.declare_parameter("model_name", "IDEA-Research/grounding-dino-tiny")
        self.declare_parameter("prompts", "door,person,chair,table,poster,sign,hallway,obstacle,robot,dog")
        self.declare_parameter("threshold", 0.25)
        self.declare_parameter("min_period_sec", 1.0)
        prompts = [p.strip() for p in str(self.get_parameter("prompts").value).split(",") if p.strip()]
        self.detector = OpenVocabularyDetector(str(self.get_parameter("backend").value), str(self.get_parameter("model_name").value), prompts, float(self.get_parameter("threshold").value))
        self.last_ts = 0.0
        self.pub = self.create_publisher(String, "/go2_perception/open_vocab_detections", 10)
        self.create_subscription(Image, str(self.get_parameter("camera_topic").value), self._on_image, qos_profile_sensor_data)
        self.get_logger().info("Open-vocabulary detector ready: backend=%s model=%s" % (self.detector._loaded_backend, self.detector.model_name))

    def _on_image(self, msg: Image) -> None:
        now = time.time()
        if now - self.last_ts < float(self.get_parameter("min_period_sec").value):
            return
        self.last_ts = now
        payload = self.detector.detect(msg)
        payload["stamp"] = {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenVocabularyDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
