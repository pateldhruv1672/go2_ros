from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backends.artifact_store_files import ArtifactStoreFiles
from .memory_api import UnifiedMemoryAPI
from .memory_schema import new_id
from .session_resolution import resolve_semantic_session_name
from .vlm_client import VLMClient

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None
    np = None


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class VLMCheckpointNode(Node):
    """Writes timed multimodal checkpoints from camera + odom + VLM.

    This node is feature-flagged and additive. It does not control motion. It only
    observes the current ROS stream, stores a raw image artifact when possible,
    calls the configured VLM provider, and writes a checkpoint through the unified
    memory API.
    """

    def __init__(self) -> None:
        super().__init__("go2_vlm_checkpoint_node")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "default")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("write_period_sec", 5.0)
        self.declare_parameter("auto_write_checkpoints", True)
        self.declare_parameter("vlm_provider", "offline")
        self.declare_parameter("vlm_model", "google/gemini-2.5-flash")
        self.declare_parameter("vlm_api_key", "")
        self.declare_parameter("vlm_base_url", "")
        self.declare_parameter("vlm_prompt", "")
        self.declare_parameter("enable_graph_memory", True)
        self.declare_parameter("enable_voxel_memory", True)
        self.declare_parameter("enable_vector_memory", True)
        session_root = self.get_parameter("session_root").value
        requested_session_name = self.get_parameter("session_name").value
        self.session_name = resolve_semantic_session_name(session_root, requested_session_name)
        if self.session_name != str(requested_session_name):
            self.get_logger().info(f"Resolved semantic session '{requested_session_name}' -> '{self.session_name}'")
        self.api = UnifiedMemoryAPI(
            session_root=session_root,
            enable_graph_memory=_as_bool(self.get_parameter("enable_graph_memory").value),
            enable_voxel_memory=_as_bool(self.get_parameter("enable_voxel_memory").value),
            enable_vector_memory=_as_bool(self.get_parameter("enable_vector_memory").value),
        )
        self.artifacts = ArtifactStoreFiles(session_root)
        self.vlm = VLMClient(
            provider=str(self.get_parameter("vlm_provider").value),
            model=str(self.get_parameter("vlm_model").value),
            api_key=str(self.get_parameter("vlm_api_key").value),
            base_url=str(self.get_parameter("vlm_base_url").value),
        )
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Odometry] = None
        self.status_pub = self.create_publisher(String, "/go2_vlm_checkpoint/status", 10)
        self.create_subscription(Image, str(self.get_parameter("camera_topic").value), self._on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)
        self.create_subscription(String, "/go2_vlm_checkpoint/write_now", self._on_write_now, 10)
        self.create_timer(float(self.get_parameter("write_period_sec").value), self._timer)
        self.get_logger().info(f"VLM checkpoint writer ready; session={self.session_name}")

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _timer(self) -> None:
        if _as_bool(self.get_parameter("auto_write_checkpoints").value):
            self._write_checkpoint("timer")

    def _on_write_now(self, msg: String) -> None:
        self._write_checkpoint(msg.data or "manual")

    def _write_checkpoint(self, trigger: str) -> None:
        checkpoint_id = new_id("vlm_ckpt")
        image_bytes, mime_type, image_ref = self._snapshot_image(checkpoint_id)
        prompt = str(self.get_parameter("vlm_prompt").value or "")
        vlm_result = self.vlm.summarize_image(image_bytes, mime_type=mime_type, prompt=prompt)
        payload: Dict[str, Any] = {
            "checkpoint_id": checkpoint_id,
            "label": f"vlm_checkpoint_{trigger}",
            "layer": "temporary",
            "source": ["vlm", "camera", "odom", trigger],
            "image_ref": image_ref,
            "vlm_summary": vlm_result.summary,
            "vlm_provider": vlm_result.provider,
            "vlm_model": vlm_result.model,
            "vlm_success": vlm_result.success,
            "vlm_error": vlm_result.error,
            "confidence": {
                "perception_confidence": 0.75 if vlm_result.success and vlm_result.provider != "offline" else 0.2,
                "odom_confidence": 1.0 if self.latest_odom is not None else 0.0,
            },
        }
        if self.latest_odom is not None:
            payload.update(self._odom_payload(self.latest_odom))
        if payload.get("odom_pose") and not payload.get("map_pose"):
            # Safe fallback for early teach runs before map pose is available.
            payload["map_pose"] = dict(payload["odom_pose"], frame_id="map")
            payload["confidence"]["map_pose_confidence"] = 0.1
        if vlm_result.raw:
            raw_rel = f"artifacts/vlm_raw/{checkpoint_id}.json"
            self.artifacts.write_json(self.session_name, raw_rel, vlm_result.raw)
            payload["vlm_raw_ref"] = str(self.artifacts.session_dir(self.session_name) / raw_rel)
        if payload.get("map_pose"):
            p = payload["map_pose"]
            payload["semantic_voxel_observation"] = {
                "center_xyz": {"x": p.get("x", 0.0), "y": p.get("y", 0.0), "z": p.get("z", 0.0)},
                "semantic_labels": self._labels_from_summary(vlm_result.summary),
                "properties": {"source": "vlm_checkpoint", "checkpoint_id": checkpoint_id, "source_confidence": {"vlm": payload["confidence"]["perception_confidence"]}},
            }
        result = self.api.write_checkpoint(self.session_name, payload)
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "success": True,
                        "checkpoint_id": result.get("id"),
                        "trigger": trigger,
                        "vlm_success": vlm_result.success,
                        "vlm_error": vlm_result.error,
                        "summary": vlm_result.summary,
                        "image_ref": image_ref,
                    },
                    sort_keys=True,
                )
            )
        )

    def _snapshot_image(self, checkpoint_id: str) -> Tuple[Optional[bytes], str, str]:
        if self.latest_image is None:
            return None, "image/jpeg", ""
        image_bytes = self._encode_image(self.latest_image)
        if not image_bytes:
            return None, "image/jpeg", ""
        rel = f"artifacts/images/{checkpoint_id}.jpg"
        path = self.artifacts.session_dir(self.session_name) / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(image_bytes)
        return image_bytes, "image/jpeg", str(path)

    def _encode_image(self, msg: Image) -> Optional[bytes]:
        if cv2 is None or np is None:
            self.get_logger().warn("cv2/numpy unavailable; cannot encode camera Image for VLM")
            return None
        try:
            if msg.encoding in {"rgb8", "bgr8"}:
                channels = 3
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
                if msg.encoding == "rgb8":
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding in {"mono8", "8UC1"}:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            else:
                self.get_logger().warn(f"Unsupported image encoding for direct VLM snapshot: {msg.encoding}")
                return None
            ok, encoded = cv2.imencode(".jpg", arr)
            return encoded.tobytes() if ok else None
        except Exception as exc:
            self.get_logger().warn(f"Image encoding failed: {exc}")
            return None

    @staticmethod
    def _odom_payload(odom: Odometry) -> Dict[str, Any]:
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        t = odom.twist.twist
        return {
            "odom_pose": {"frame_id": odom.header.frame_id or "odom", "x": p.x, "y": p.y, "z": p.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            "velocity_odom": {"linear_x": t.linear.x, "linear_y": t.linear.y, "linear_z": t.linear.z, "angular_x": t.angular.x, "angular_y": t.angular.y, "angular_z": t.angular.z},
        }

    @staticmethod
    def _labels_from_summary(summary: str) -> list[str]:
        text = summary.lower()
        labels = []
        for label in ["doorway", "hallway", "lab", "poster", "sign", "chair", "table", "person", "obstacle", "stairs", "elevator"]:
            if label in text:
                labels.append(label)
        return labels or ["vlm_checkpoint"]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLMCheckpointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
