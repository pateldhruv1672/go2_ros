from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_langgraph_agent.voice_io import LocalTTS


class TTSOutputNode(Node):
    """Broadcast agent speech through local laptop/robot speaker."""

    def __init__(self) -> None:
        super().__init__("go2_tts_output_node")
        self.declare_parameter("speech_topic", "/go2_agent/speech")
        self.declare_parameter("tts_backend", "piper")
        self.declare_parameter("piper_model_path", "")
        self.declare_parameter("speaker_device", "")
        self.tts = LocalTTS(
            backend=str(self.get_parameter("tts_backend").value),
            model_path=str(self.get_parameter("piper_model_path").value),
            speaker_device=str(self.get_parameter("speaker_device").value),
        )
        self.status_pub = self.create_publisher(String, "/go2_voice/tts_status", 10)
        self.create_subscription(String, str(self.get_parameter("speech_topic").value), self._on_speech, 10)
        self.get_logger().info("TTS output node ready: local piper/pyttsx3/espeak fallback")

    def _on_speech(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _speak(self, text: str) -> None:
        result = self.tts.speak(text)
        self.status_pub.publish(String(data=json.dumps({"text_len": len(text), **result}, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TTSOutputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
