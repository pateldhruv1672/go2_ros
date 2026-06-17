from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from go2_omi_voice_bridge.intent import decode_json_or_text, transcript_confidence, transcript_text
from go2_langgraph_agent.voice_io import OmiBleAudioSource, VoiceCommand


class Go2VoiceSttNode(Node):
    """Transcript-only STT adapter for Omi/mobile/simulated text input."""

    def __init__(self) -> None:
        super().__init__("go2_voice_stt_node")
        self.declare_parameter("adapter_mode", "transcript_only")
        self.declare_parameter("ble_device_name", "Omi")
        self.declare_parameter("ble_device_address", "")
        self.declare_parameter("stt_backend", "faster_whisper")
        self.declare_parameter("stt_model", "tiny.en")
        self.declare_parameter("omi_chunk_sec", 4.0)
        self.declare_parameter("language", "en-US")
        self.declare_parameter("default_confidence", 0.95)
        self.transcript_pub = self.create_publisher(String, "/go2_voice/transcript", 10)
        self.status_pub = self.create_publisher(String, "/go2_voice/stt_status", 10)
        self.ble_source: OmiBleAudioSource | None = None

        mode = str(self.get_parameter("adapter_mode").value).strip().lower()
        if mode in {"transcript_only", "text", "simulated"}:
            self.create_subscription(String, "/omi/transcript_raw", self._on_raw_transcript, 10)
            self.get_logger().info("Voice STT node ready; transcript_only input is /omi/transcript_raw")
        elif mode in {"ble_audio", "omi_ble", "ble"}:
            self.ble_source = OmiBleAudioSource(
                self._on_ble_command,
                device_name=str(self.get_parameter("ble_device_name").value),
                device_address=str(self.get_parameter("ble_device_address").value),
                stt_backend=str(self.get_parameter("stt_backend").value),
                stt_model=str(self.get_parameter("stt_model").value),
                chunk_sec=float(self.get_parameter("omi_chunk_sec").value),
            )
            self.ble_source.start()
            self._publish_status({"ok": True, "state": "ble_starting", "address": str(self.get_parameter("ble_device_address").value)})
            self.get_logger().info("Voice STT node starting Omi BLE audio source")
        else:
            self.create_subscription(String, "/omi/transcript_raw", self._on_raw_transcript, 10)
            self._publish_status({"ok": False, "state": "unsupported_adapter_mode", "adapter_mode": mode})
            self.get_logger().warn(f"Unsupported adapter_mode={mode}; falling back to transcript_only")

    def destroy_node(self):
        if self.ble_source is not None:
            try:
                self.ble_source.stop()
            except Exception:
                pass
        super().destroy_node()

    def _publish_status(self, payload: dict) -> None:
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _on_raw_transcript(self, msg: String) -> None:
        payload = decode_json_or_text(msg.data)
        text = transcript_text(payload)
        confidence = transcript_confidence(payload)
        if confidence == 1.0 and "confidence" not in payload:
            confidence = float(self.get_parameter("default_confidence").value)
        out = {
            "source": payload.get("source", "omi"),
            "text": text,
            "confidence": confidence,
            "start_time": payload.get("start_time", 0.0),
            "end_time": payload.get("end_time", time.time()),
            "is_final": bool(payload.get("is_final", True)),
            "language": payload.get("language", str(self.get_parameter("language").value)),
            "raw": payload,
        }
        self.transcript_pub.publish(String(data=json.dumps(out, sort_keys=True)))
        self._publish_status({"ok": bool(text), "text_len": len(text), "confidence": confidence, "source": out["source"]})

    def _on_ble_command(self, command: VoiceCommand) -> None:
        if not command.text:
            payload = {
                "ok": command.source not in {"omi_ble_error", "local_stt_error"},
                "source": command.source,
                "confidence": command.confidence,
                "raw": command.raw or {},
            }
            self._publish_status(payload)
            if command.source == "omi_ble_status":
                self.get_logger().info(f"Omi BLE status: {json.dumps(payload, sort_keys=True, default=str)}")
            elif command.source.endswith("_error"):
                self.get_logger().warn(f"Omi BLE/STT error: {json.dumps(payload, sort_keys=True, default=str)}")
            return

        out = {
            "source": command.source,
            "text": command.text,
            "confidence": command.confidence,
            "start_time": 0.0,
            "end_time": time.time(),
            "is_final": True,
            "language": str(self.get_parameter("language").value),
            "raw": command.raw or {},
        }
        self.transcript_pub.publish(String(data=json.dumps(out, sort_keys=True)))
        self._publish_status({"ok": True, "text_len": len(command.text), "confidence": command.confidence, "source": command.source})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2VoiceSttNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
