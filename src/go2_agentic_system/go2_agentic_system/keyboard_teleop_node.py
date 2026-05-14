from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = "keys: w forward | s back | a left | d right | x stop | t toggle survey | c capture memory | p remember named place | g goto named place | q quit keyboard node"


class KeyboardTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop_node')
        self.declare_parameter('input_topic', '/agent/input_text')
        self.declare_parameter('mode_topic', '/agent/mode')
        self.input_pub = self.create_publisher(String, self.get_parameter('input_topic').value, 20)
        self.mode_sub = self.create_subscription(String, self.get_parameter('mode_topic').value, self.on_mode, 10)
        self.current_mode = 'idle'
        self.stdin_thread = threading.Thread(target=self._loop, daemon=True)
        self.stdin_thread.start()
        self.get_logger().info(HELP)

    def on_mode(self, msg: String) -> None:
        self.current_mode = msg.data.strip() or 'idle'

    def _send(self, text: str) -> None:
        self.input_pub.publish(String(data=text))

    def _prompt_line(self, prompt: str) -> str:
        sys.stdout.write('\n' + prompt)
        sys.stdout.flush()
        line = sys.stdin.readline().strip()
        print(HELP, flush=True)
        return line

    def _loop(self) -> None:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'w':
                        self._send('forward 0.8')
                    elif ch == 's':
                        self._send('back 0.8')
                    elif ch == 'a':
                        self._send('left 0.7')
                    elif ch == 'd':
                        self._send('right 0.7')
                    elif ch == 'x':
                        self._send('stop')
                    elif ch == 't':
                        self._send('stop survey' if self.current_mode == 'survey' else 'start survey')
                    elif ch == 'c':
                        self._send('capture memory')
                    elif ch == 'p':
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        name = self._prompt_line('Place name: ')
                        tty.setcbreak(fd)
                        if name:
                            self._send(f'remember this place as {name}')
                    elif ch == 'g':
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        name = self._prompt_line('Go to place: ')
                        tty.setcbreak(fd)
                        if name:
                            self._send(f'go to {name}')
                    elif ch == 'q':
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
