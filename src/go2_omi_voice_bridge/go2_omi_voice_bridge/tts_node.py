from __future__ import annotations

import json
import shutil
import subprocess
import threading
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _message_text(data: str) -> tuple[str, dict[str, Any]]:
    raw = data.strip()
    if not raw:
        return "", {}
    try:
        payload = json.loads(raw)
        if isinstance(payload, dict):
            text = str(payload.get("text") or payload.get("message") or payload.get("summary") or payload.get("data") or "").strip()
            if payload.get("vlm_success") is False:
                text = str(payload.get("vlm_error") or "I could not get a camera summary from the VLM.").strip()
            return text, payload
    except Exception:
        pass
    return raw, {"text": raw}


class Go2TtsNode(Node):
    """Simple interruptible TTS adapter.

    It always republishes status and logs speech text. If espeak is installed and
    tts_backend is local/espeak, it speaks locally; otherwise it remains a safe
    no-audio bridge for demos and CI.
    """

    def __init__(self) -> None:
        super().__init__("go2_tts_node")
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_backend", "local")
        self.declare_parameter("espeak_voice", "en")
        self.status_pub = self.create_publisher(String, "/go2_tts/status", 10)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        for topic in ("/go2_tts/say", "/go2_agent/response", "/go2_agent/speech", "/go2_tour/narration", "/go2_vlm_checkpoint/status"):
            self.create_subscription(String, topic, self._on_speech, 10)
        self.get_logger().info("Go2 TTS node ready; subscribes to /go2_tts/say, /go2_agent/response, /go2_agent/speech, /go2_tour/narration, /go2_vlm_checkpoint/status")

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _on_speech(self, msg: String) -> None:
        text, payload = _message_text(msg.data)
        if not text:
            return
        interrupt = bool(payload.get("interrupt", False)) or payload.get("priority") in {"high", "urgent"}
        if interrupt:
            self._stop_current()
        self.get_logger().info(f"TTS: {text}")
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "ok": True,
                        "text_len": len(text),
                        "category": payload.get("category", "status"),
                        "backend": str(self.get_parameter("tts_backend").value),
                    },
                    sort_keys=True,
                )
            )
        )
        if self._param_bool("tts_enabled"):
            threading.Thread(target=self._speak_local, args=(text,), daemon=True).start()

    def _stop_current(self) -> None:
        with self._lock:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
            self._proc = None

    def _speak_local(self, text: str) -> None:
        backend = str(self.get_parameter("tts_backend").value).lower()
        if backend in {"off", "disabled", "none"}:
            return
        exe = shutil.which("espeak")
        if exe is None:
            return
        voice = str(self.get_parameter("espeak_voice").value)
        with self._lock:
            self._proc = subprocess.Popen([exe, "-v", voice, text])
            proc = self._proc
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            proc.terminate()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2TtsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_current()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
