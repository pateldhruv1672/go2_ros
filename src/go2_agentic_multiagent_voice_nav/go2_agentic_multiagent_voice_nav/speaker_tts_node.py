from __future__ import annotations

import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeakerTtsNode(Node):
    def __init__(self) -> None:
        super().__init__("speaker_tts_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("voice", "en-us")
        self.declare_parameter("rate_wpm", 170)
        self.declare_parameter("pitch", 50)
        self.declare_parameter("volume", 100)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.voice = str(self.get_parameter("voice").value).strip() or "en-us"
        self.rate_wpm = int(self.get_parameter("rate_wpm").value)
        self.pitch = int(self.get_parameter("pitch").value)
        self.volume = int(self.get_parameter("volume").value)

        self.lock = threading.Lock()
        self.espeak_path = shutil.which("espeak") or shutil.which("espeak-ng")
        if self.enabled and not self.espeak_path:
            self.get_logger().error("No espeak/espeak-ng found. Install with: sudo apt install espeak espeak-ng")
            self.enabled = False

        self.create_subscription(String, "/agent/reply", self.reply_cb, 20)
        self.get_logger().info(
            f"ready | enabled={self.enabled} | espeak={self.espeak_path} | voice={self.voice}"
        )

    def reply_cb(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text or not self.enabled or not self.espeak_path:
            return
        with self.lock:
            try:
                subprocess.run(
                    [
                        self.espeak_path,
                        "-v", self.voice,
                        "-s", str(self.rate_wpm),
                        "-p", str(self.pitch),
                        "-a", str(self.volume),
                        text,
                    ],
                    check=False,
                )
            except Exception as exc:
                self.get_logger().error(f"TTS failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerTtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
