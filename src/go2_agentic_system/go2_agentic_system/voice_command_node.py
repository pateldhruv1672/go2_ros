from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommandNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_command_node')
        self.declare_parameter('stdin_enabled', True)
        self.declare_parameter('input_topic', '/agent/voice_text_raw')
        self.declare_parameter('output_topic', '/agent/voice_text')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.subscription = self.create_subscription(String, input_topic, self.forward_callback, 10)

        if self.get_parameter('stdin_enabled').get_parameter_value().bool_value:
            self.stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self.stdin_thread.start()
            self.get_logger().info('stdin voice bridge enabled. Type commands and press Enter.')
        else:
            self.get_logger().info('stdin voice bridge disabled. Waiting for /agent/voice_text_raw input.')

    def forward_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.publisher.publish(String(data=text))
        self.get_logger().info(f'forwarded voice text: {text}')

    def _stdin_loop(self) -> None:
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            text = line.strip()
            if not text:
                continue
            self.publisher.publish(String(data=text))
            self.get_logger().info(f'stdin voice text: {text}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
