from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_nav_console')
        self.pub = self.create_publisher(String, '/semantic_nav/command', 10)
        self.create_subscription(String, '/semantic_nav/status', self.status_cb, 10)

    def status_cb(self, msg: String) -> None:
        print(f'[{msg.data}]', flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConsoleNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        while rclpy.ok():
            line = input('nav> ').strip()
            if not line:
                continue
            msg = String()
            msg.data = line
            node.pub.publish(msg)
            if line in {'exit', 'quit'}:
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=0.5)
        sys.exit(0)


if __name__ == '__main__':
    main()
