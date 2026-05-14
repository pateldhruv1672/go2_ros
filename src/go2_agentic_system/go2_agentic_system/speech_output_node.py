from __future__ import annotations

import shutil
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechOutputNode(Node):
    def __init__(self) -> None:
        super().__init__('speech_output_node')
        self.declare_parameter('speak_topic', '/agent/speak_text')
        self.declare_parameter('use_espeak', False)
        self.subscription = self.create_subscription(String, self.get_parameter('speak_topic').value, self.on_text, 20)
        self.espeak_path = shutil.which('espeak') if self.get_parameter('use_espeak').value else None
        self.get_logger().info(f'speech output ready; espeak={bool(self.espeak_path)}')

    def on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'say: {text}')
        if self.espeak_path:
            try:
                subprocess.Popen([self.espeak_path, text])
            except Exception as exc:
                self.get_logger().error(f'espeak failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
