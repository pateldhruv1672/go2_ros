from __future__ import annotations

import json
import threading
import time
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
        self.declare_parameter("query_topic", "/go2_vlm/query")
        self.declare_parameter("query_result_topic", "/go2_vlm/query_result")
        self.declare_parameter("max_live_image_age_sec", 2.5)
        self.declare_parameter("fresh_frame_wait_timeout_sec", 1.5)
        self.declare_parameter("detector_context_wait_sec", 0.25)
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
        self.latest_image_monotonic = 0.0
        self.latest_odom: Optional[Odometry] = None
        self.latest_object_inventory: Dict[str, Any] = {}
        self._sensor_lock = threading.Lock()
        self._live_query_lock = threading.Lock()
        self.latest_image_received_unix = 0.0
        self.latest_odom_received_unix = 0.0
        self.latest_object_inventory_received_unix = 0.0
        self.status_pub = self.create_publisher(String, "/go2_vlm_checkpoint/status", 10)
        self.query_result_pub = self.create_publisher(String, str(self.get_parameter("query_result_topic").value), 10)
        self.create_subscription(Image, str(self.get_parameter("camera_topic").value), self._on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)
        self.create_subscription(String, "/go2_memory/object_inventory", self._on_object_inventory, 10)
        self.create_subscription(String, "/go2_vlm_checkpoint/write_now", self._on_write_now, 10)
        self.create_subscription(String, str(self.get_parameter("query_topic").value), self._on_query, 10)
        self.create_timer(float(self.get_parameter("write_period_sec").value), self._timer)
        self.get_logger().info(f"VLM checkpoint writer ready; session={self.session_name}")

    def _on_image(self, msg: Image) -> None:
        now_unix = time.time()
        with self._sensor_lock:
            self.latest_image = msg
            self.latest_image_monotonic = time.monotonic()
            self.latest_image_received_unix = now_unix

    def _on_odom(self, msg: Odometry) -> None:
        with self._sensor_lock:
            self.latest_odom = msg
            self.latest_odom_received_unix = time.time()

    def _on_object_inventory(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            value = value if isinstance(value, dict) else {}
        except Exception:
            value = {}
        with self._sensor_lock:
            self.latest_object_inventory = value
            self.latest_object_inventory_received_unix = time.time()

    def _on_query(self, msg: String) -> None:
        """Schedule a live query against sensor data received after this request."""
        try:
            payload = json.loads(msg.data) if str(msg.data or '').strip().startswith('{') else {'question': msg.data}
        except Exception:
            payload = {'question': msg.data}
        if not isinstance(payload, dict):
            payload = {'question': str(msg.data or '')}
        payload = dict(payload)
        payload.setdefault('request_id', f'vlm_{self.get_clock().now().nanoseconds}')
        try:
            command_received_unix = float(payload.get('command_received_unix') or time.time())
        except Exception:
            command_received_unix = time.time()
        payload['command_received_unix'] = command_received_unix
        # SPARKY_POST_COMMAND_SENSOR_BARRIER_V12_8
        threading.Thread(
            target=self._run_live_query_serialized,
            args=(payload,),
            name=f"vlm-fresh-{str(payload['request_id'])[-12:]}",
            daemon=True,
        ).start()

    def _run_live_query_serialized(self, payload: Dict[str, Any]) -> None:
        # Preserve request/response order when OpenRouter calls overlap.
        with self._live_query_lock:
            self._run_live_query_after_fresh_frame(payload)

    def _run_live_query_after_fresh_frame(self, payload: Dict[str, Any]) -> None:
        request_id = str(payload.get('request_id') or f'vlm_{time.time_ns()}')
        question = str(payload.get('question') or payload.get('text') or 'What do you see in front of me?').strip()
        command_received_unix = float(payload.get('command_received_unix') or time.time())
        sensor_not_before_unix = float(payload.get('sensor_not_before_unix') or command_received_unix)
        wait_timeout = max(0.2, float(self.get_parameter('fresh_frame_wait_timeout_sec').value))
        deadline = time.monotonic() + wait_timeout
        image_msg = None
        image_arrival_unix = 0.0
        while time.monotonic() < deadline:
            with self._sensor_lock:
                candidate = self.latest_image
                candidate_arrival = self.latest_image_received_unix
            if candidate is not None and candidate_arrival >= sensor_not_before_unix:
                image_msg = candidate
                image_arrival_unix = candidate_arrival
                break
            time.sleep(0.02)
        sensor_wait_sec = max(0.0, time.time() - sensor_not_before_unix)
        end_to_end_wait_sec = max(0.0, time.time() - command_received_unix)
        if image_msg is None:
            out = {
                'event':'live_query_result','request_id':request_id,'success':False,'summary':'',
                'provider':self.vlm.provider,'model':self.vlm.model,
                'error':f'no camera frame arrived after the command within {wait_timeout:.2f}s',
                'stamp_sec':self.get_clock().now().nanoseconds / 1e9,
                'command_received_unix':command_received_unix,'sensor_not_before_unix':sensor_not_before_unix,'sensor_wait_sec':sensor_wait_sec,'end_to_end_wait_sec':end_to_end_wait_sec,
                'freshness_policy':'post_command_frame_required',
            }
            encoded=json.dumps(out,sort_keys=True,default=str)
            self.query_result_pub.publish(String(data=encoded)); self.status_pub.publish(String(data=encoded)); return

        # Give the detector/object inventory a short bounded window to catch up to
        # the same operator request. The RGB image remains the source of truth.
        detector_wait = max(0.0, float(self.get_parameter('detector_context_wait_sec').value))
        detector_deadline = time.monotonic() + detector_wait
        detector_context: Dict[str, Any] = {}
        detector_arrival_unix = 0.0
        while True:
            with self._sensor_lock:
                detector_context = dict(self.latest_object_inventory) if isinstance(self.latest_object_inventory, dict) else {}
                detector_arrival_unix = self.latest_object_inventory_received_unix
            if detector_arrival_unix >= sensor_not_before_unix or time.monotonic() >= detector_deadline:
                break
            time.sleep(0.02)
        detector_fresh = detector_arrival_unix >= sensor_not_before_unix
        image_age = max(0.0, time.time() - image_arrival_unix)
        max_age = max(0.2, float(self.get_parameter('max_live_image_age_sec').value))
        image_bytes = self._encode_image(image_msg) if image_age <= max_age else None
        if image_bytes is None:
            out = {
                'event':'live_query_result','request_id':request_id,'success':False,'summary':'',
                'provider':self.vlm.provider,'model':self.vlm.model,
                'error':f'post-command camera frame could not be encoded or became stale (age_sec={image_age:.2f})',
                'stamp_sec':self.get_clock().now().nanoseconds / 1e9,
                'command_received_unix':command_received_unix,'sensor_not_before_unix':sensor_not_before_unix,'image_received_unix':image_arrival_unix,
                'sensor_wait_sec':sensor_wait_sec,'end_to_end_wait_sec':end_to_end_wait_sec,'detector_fresh':detector_fresh,
                'freshness_policy':'post_command_frame_required',
            }
            encoded=json.dumps(out,sort_keys=True,default=str)
            self.query_result_pub.publish(String(data=encoded)); self.status_pub.publish(String(data=encoded)); return
        prompt = (
            'You are the live vision system for a Unitree Go2 robot. Analyze ONLY the CURRENT attached camera image. '
            'The image was received after the operator command, so answer about that fresh scene only. '
            'Use detector context only as supporting hints, correct it if the image disagrees, and mention uncertainty when needed. '
            f'Operator question: {question}\n'
            f'Live detector context (fresh={detector_fresh}): {json.dumps(detector_context, default=str)[:5000]}'
        )
        self.status_pub.publish(String(data=json.dumps({
            'event':'live_query_started','request_id':request_id,'provider':self.vlm.provider,
            'model':self.vlm.model,'command_received_unix':command_received_unix,'sensor_not_before_unix':sensor_not_before_unix,
            'image_received_unix':image_arrival_unix,'sensor_wait_sec':sensor_wait_sec,
            'detector_fresh':detector_fresh,'freshness_policy':'post_command_frame_required',
        },sort_keys=True,default=str)))
        try:
            result = self.vlm.summarize_image(image_bytes, mime_type='image/jpeg', prompt=prompt)
            out = {
                'event':'live_query_result','request_id':request_id,'success':bool(result.success),
                'summary':str(result.summary or ''),'provider':str(result.provider or self.vlm.provider),
                'model':str(result.model or self.vlm.model),'error':str(result.error or ''),
            }
        except Exception as exc:
            out = {'event':'live_query_result','request_id':request_id,'success':False,'summary':'',
                   'provider':self.vlm.provider,'model':self.vlm.model,'error':str(exc)}
        out.update({
            'stamp_sec':self.get_clock().now().nanoseconds / 1e9,
            'command_received_unix':command_received_unix,'sensor_not_before_unix':sensor_not_before_unix,'image_received_unix':image_arrival_unix,
            'sensor_wait_sec':sensor_wait_sec,'detector_context':detector_context,
            'detector_fresh':detector_fresh,'freshness_policy':'post_command_frame_required',
        })
        encoded=json.dumps(out,sort_keys=True,default=str)
        self.query_result_pub.publish(String(data=encoded)); self.status_pub.publish(String(data=encoded))

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
            "object_inventory": self.latest_object_inventory,
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
