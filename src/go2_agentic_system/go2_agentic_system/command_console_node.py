from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('command_console_node')
        self.declare_parameter('input_topic', '/agent/input_text')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('stdin_enabled', True)
        self.declare_parameter('show_status', True)
        self.input_pub = self.create_publisher(String, self.get_parameter('input_topic').value, 20)
        self.status_sub = self.create_subscription(String, self.get_parameter('status_topic').value, self.on_status, 50)
        if self.get_parameter('stdin_enabled').value:
            self.stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self.stdin_thread.start()
            self.get_logger().info('console ready: type commands like "start survey", "go to gpu lab and say hi to all the students", "remember this place as dock", "capture memory", "stop"')

    def on_status(self, msg: String) -> None:
        if self.get_parameter('show_status').value:
            print(f'[agent] {msg.data}', flush=True)

    def _stdin_loop(self) -> None:
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            text = line.strip()
            if not text:
                continue
            self.input_pub.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
