from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_langgraph_agent.voice_io import LaptopMicSTTSource, OmiBleAudioSource, VoiceCommand


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class VoiceInputNode(Node):
    """Unified text, laptop mic, and Omi BLE local-STT command bridge."""

    def __init__(self) -> None:
        super().__init__("go2_voice_input_node")
        self.declare_parameter("input_mode", "text_topic")  # text_topic | laptop_mic | omi_ble | all
        self.declare_parameter("text_topic", "/go2_voice/text_command")
        self.declare_parameter("publish_topic", "/go2_agent/user_command")
        self.declare_parameter("stt_backend", "faster_whisper")
        self.declare_parameter("stt_model", "tiny.en")
        self.declare_parameter("phrase_time_limit", 6.0)
        self.declare_parameter("omi_device_name", "Omi")
        self.declare_parameter("omi_device_address", "")
        self.declare_parameter("omi_chunk_sec", 4.0)
        self.declare_parameter("publish_errors", True)
        self.pub = self.create_publisher(String, str(self.get_parameter("publish_topic").value), 10)
        self.status_pub = self.create_publisher(String, "/go2_voice/status", 10)
        self.sources = []
        mode = str(self.get_parameter("input_mode").value).lower()
        if mode in {"text_topic", "text", "all"}:
            self.create_subscription(String, str(self.get_parameter("text_topic").value), self._on_text, 10)
        if mode in {"laptop_mic", "mic", "all"}:
            src = LaptopMicSTTSource(
                self._on_voice_command,
                backend=str(self.get_parameter("stt_backend").value),
                model_name=str(self.get_parameter("stt_model").value),
                phrase_time_limit=float(self.get_parameter("phrase_time_limit").value),
            )
            src.start()
            self.sources.append(src)
        if mode in {"omi", "omi_ble", "all"}:
            src = OmiBleAudioSource(
                self._on_voice_command,
                device_name=str(self.get_parameter("omi_device_name").value),
                device_address=str(self.get_parameter("omi_device_address").value),
                stt_backend=str(self.get_parameter("stt_backend").value),
                stt_model=str(self.get_parameter("stt_model").value),
                chunk_sec=float(self.get_parameter("omi_chunk_sec").value),
            )
            src.start()
            self.sources.append(src)
        self.get_logger().info(f"Voice input node ready in mode={mode}; Omi path uses BLE + local STT, not webhooks")

    def destroy_node(self):
        for source in self.sources:
            try:
                source.stop()
            except Exception:
                pass
        super().destroy_node()

    def _on_text(self, msg: String) -> None:
        self._on_voice_command(VoiceCommand(text=msg.data.strip(), source="text_topic", confidence=1.0, raw={}))

    def _on_voice_command(self, command: VoiceCommand) -> None:
        payload: Dict[str, object] = {
            "text": command.text,
            "source": command.source,
            "confidence": command.confidence,
            "raw": command.raw or {},
        }
        if command.text:
            self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        elif _as_bool(self.get_parameter("publish_errors").value):
            self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
