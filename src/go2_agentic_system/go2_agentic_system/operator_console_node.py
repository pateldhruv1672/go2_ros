from __future__ import annotations

import curses
import json
import threading
import time
from collections import deque
from typing import Deque, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP_LINES = [
    'Hotkeys: w/s/a/d drive  space stop  t survey  i AI mode  c capture  p save place  g go to  m save map  r recall  / command  q quit UI',
    'Examples: go to gpu lab and say hi to all the students | remember this place as gpu lab | save map as building_a',
]


class OperatorConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('operator_console_node')
        self.declare_parameter('input_topic', '/agent/input_text')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('mode_topic', '/agent/mode')
        self.declare_parameter('telemetry_topic', '/agent/telemetry')
        self.declare_parameter('refresh_hz', 12.0)

        self.input_pub = self.create_publisher(String, self.get_parameter('input_topic').value, 20)
        self.create_subscription(String, self.get_parameter('status_topic').value, self.on_status, 50)
        self.create_subscription(String, self.get_parameter('mode_topic').value, self.on_mode, 20)
        self.create_subscription(String, self.get_parameter('telemetry_topic').value, self.on_telemetry, 20)

        self.mode = 'unknown'
        self.telemetry: Dict[str, object] = {}
        self.logs: Deque[str] = deque(maxlen=14)
        self.banner = 'Go2 Agent Operator Console'
        self.last_prompt: str = ''
        self._shutdown = False
        self._ui_ready = threading.Event()

    def on_status(self, msg: String) -> None:
        self.logs.appendleft(msg.data)

    def on_mode(self, msg: String) -> None:
        self.mode = msg.data or 'unknown'

    def on_telemetry(self, msg: String) -> None:
        try:
            self.telemetry = json.loads(msg.data)
        except json.JSONDecodeError:
            self.telemetry = {'raw': msg.data}

    def send(self, text: str) -> None:
        self.input_pub.publish(String(data=text))
        self.last_prompt = text

    def request_text(self, stdscr, prompt: str) -> Optional[str]:
        curses.echo()
        curses.curs_set(1)
        h, w = stdscr.getmaxyx()
        stdscr.attron(curses.A_REVERSE)
        stdscr.addnstr(h - 1, 0, ' ' * max(1, w - 1), max(1, w - 1))
        stdscr.addnstr(h - 1, 0, prompt, max(1, w - 1))
        stdscr.attroff(curses.A_REVERSE)
        stdscr.refresh()
        try:
            raw = stdscr.getstr(h - 1, min(len(prompt), max(0, w - 2)), max(1, w - len(prompt) - 2))
        except Exception:
            raw = b''
        curses.noecho()
        curses.curs_set(0)
        text = raw.decode('utf-8', errors='ignore').strip()
        return text or None

    def draw(self, stdscr) -> None:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        top = 0
        stdscr.attron(curses.A_BOLD)
        stdscr.addnstr(top, 2, self.banner, max(1, w - 4))
        stdscr.attroff(curses.A_BOLD)
        top += 2

        tel = self.telemetry
        map_name = str(tel.get('active_map') or '-')
        localized = 'YES' if tel.get('localized') else 'NO'
        nav_active = 'YES' if tel.get('nav_active') else 'NO'
        survey = 'YES' if tel.get('survey_active') else 'NO'
        pose = tel.get('pose') or {}
        pose_txt = '-'
        if isinstance(pose, dict) and 'x' in pose and 'y' in pose:
            pose_txt = f"x={pose.get('x', 0.0):.2f}  y={pose.get('y', 0.0):.2f}  yaw={pose.get('yaw', 0.0):.2f}"
        memory = tel.get('memory') or {}
        front_txt = tel.get('front_clearance_m')
        rear_txt = tel.get('rear_clearance_m')
        front_s = f"{front_txt:.2f}m" if isinstance(front_txt, (int, float)) else '-'
        rear_s = f"{rear_txt:.2f}m" if isinstance(rear_txt, (int, float)) else '-'

        lines = [
            f'Mode: {self.mode:12}  Survey: {survey:3}  Nav active: {nav_active:3}  Localized: {localized:3}',
            f'Active map: {map_name}',
            f'Pose: {pose_txt}',
            f'Front clearance: {front_s:8}  Rear clearance: {rear_s:8}',
            f"Memory: {memory.get('observations', 0)} observations  {memory.get('places', 0)} places",
            f"Last command: {str(tel.get('last_command') or '-')}",
        ]
        for line in lines:
            stdscr.addnstr(top, 2, line, max(1, w - 4))
            top += 1

        top += 1
        stdscr.attron(curses.A_UNDERLINE)
        stdscr.addnstr(top, 2, 'Recent status', max(1, w - 4))
        stdscr.attroff(curses.A_UNDERLINE)
        top += 1

        log_height = max(4, h - top - 5)
        logs = list(self.logs)[:log_height]
        for line in logs:
            stdscr.addnstr(top, 2, f'- {line}', max(1, w - 4))
            top += 1

        footer_y = h - 3
        for line in HELP_LINES:
            stdscr.addnstr(footer_y, 2, line, max(1, w - 4))
            footer_y += 1
        stdscr.attron(curses.A_REVERSE)
        stdscr.addnstr(h - 1, 0, (' Last input: ' + (self.last_prompt or '-')).ljust(max(1, w - 1)), max(1, w - 1))
        stdscr.attroff(curses.A_REVERSE)
        stdscr.refresh()

    def run_ui(self, stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(int(1000.0 / max(1.0, float(self.get_parameter('refresh_hz').value))))
        self._ui_ready.set()
        while rclpy.ok() and not self._shutdown:
            self.draw(stdscr)
            ch = stdscr.getch()
            if ch == -1:
                continue
            if ch in (ord('q'), ord('Q')):
                self._shutdown = True
                break
            if ch in (ord('w'), ord('W')):
                self.send('forward 0.5')
            elif ch in (ord('s'), ord('S')):
                self.send('back 0.5')
            elif ch in (ord('a'), ord('A')):
                self.send('left 0.4')
            elif ch in (ord('d'), ord('D')):
                self.send('right 0.4')
            elif ch == ord(' '):
                self.send('stop')
            elif ch in (ord('t'), ord('T')):
                if self.mode == 'survey':
                    self.send('stop survey')
                else:
                    self.send('start survey')
            elif ch in (ord('i'), ord('I')):
                self.send('stop survey')
            elif ch in (ord('c'), ord('C')):
                self.send('capture memory')
            elif ch in (ord('p'), ord('P')):
                text = self.request_text(stdscr, 'Save current place as: ')
                if text:
                    self.send(f'remember this place as {text}')
            elif ch in (ord('m'), ord('M')):
                text = self.request_text(stdscr, 'Save map as: ')
                if text:
                    self.send(f'save map as {text}')
            elif ch in (ord('g'), ord('G')):
                text = self.request_text(stdscr, 'Navigate to: ')
                if text:
                    self.send(f'go to {text}')
            elif ch in (ord('r'), ord('R')):
                text = self.request_text(stdscr, 'Recall memories about: ')
                if text:
                    self.send(f'what do you remember about {text}')
            elif ch == ord('/'):
                text = self.request_text(stdscr, 'Command: ')
                if text:
                    self.send(text)
        self.draw(stdscr)
        time.sleep(0.1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorConsoleNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        curses.wrapper(node.run_ui)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown = True
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
