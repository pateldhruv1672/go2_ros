#!/usr/bin/env python3

import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


def norm(s: str) -> str:
    return re.sub(r"[^a-z0-9 ]+", " ", (s or "").lower()).strip()


def load_places(path: str) -> List[Dict[str, Any]]:
    p = Path(path).expanduser()
    if not p.exists():
        return []
    data = yaml.safe_load(p.read_text()) or {}
    if isinstance(data, dict):
        places = data.get("places", [])
    elif isinstance(data, list):
        places = data
    else:
        places = []
    return [x for x in places if isinstance(x, dict)]


class OmiDemoBridge(Node):
    def __init__(self):
        super().__init__("omi_demo_bridge")

        self.declare_parameter("places_file", "")
        self.declare_parameter("omi_text_topic", "/omi/transcript")
        self.declare_parameter("semantic_command_topic", "/semantic_nav/command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_omi")
        self.declare_parameter("direct_motion_duration_sec", 1.0)
        self.declare_parameter("direct_x", 0.12)
        self.declare_parameter("direct_y", 0.10)
        self.declare_parameter("direct_yaw", 0.35)

        self.places_file = str(self.get_parameter("places_file").value)
        self.omi_text_topic = str(self.get_parameter("omi_text_topic").value)

        self.semantic_pub = self.create_publisher(
            String,
            str(self.get_parameter("semantic_command_topic").value),
            10,
        )
        self.twist_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )

        self.sub = self.create_subscription(
            String,
            self.omi_text_topic,
            self.on_text,
            10,
        )

        self.active_twist: Optional[Twist] = None
        self.active_until = 0.0
        self.timer = self.create_timer(0.05, self.tick_motion)

        self.places = load_places(self.places_file)
        self.phrases = self.build_phrases(self.places)

        self.get_logger().info(
            f"omi demo bridge ready: input={self.omi_text_topic} places={len(self.places)}"
        )

    def build_phrases(self, places: List[Dict[str, Any]]) -> List[Tuple[str, str]]:
        pairs = []
        for p in places:
            name = str(p.get("name", "")).strip()
            if name:
                pairs.append((norm(name.replace("_", " ")), name))
                pairs.append((norm(name), name))
            for a in p.get("aliases", []) or []:
                if a:
                    pairs.append((norm(str(a).replace("_", " ")), name))
                    pairs.append((norm(str(a)), name))
            room = str(p.get("room", "")).strip()
            if room and room != "unknown" and name:
                pairs.append((norm(room.replace("_", " ")), name))
        # Longest phrase first.
        pairs = sorted(set(pairs), key=lambda x: len(x[0]), reverse=True)
        return pairs

    def publish_semantic(self, command: str):
        msg = String()
        msg.data = command
        self.semantic_pub.publish(msg)
        self.get_logger().info(f"semantic_command: {command}")

    def start_direct_motion(self, x=0.0, y=0.0, yaw=0.0):
        t = Twist()
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.angular.z = float(yaw)
        self.active_twist = t
        self.active_until = time.time() + float(self.get_parameter("direct_motion_duration_sec").value)
        self.get_logger().info(
            f"direct_motion: x={t.linear.x:.2f} y={t.linear.y:.2f} yaw={t.angular.z:.2f}"
        )

    def tick_motion(self):
        if self.active_twist is not None and time.time() < self.active_until:
            self.twist_pub.publish(self.active_twist)
        elif self.active_twist is not None:
            self.twist_pub.publish(Twist())
            self.active_twist = None

    def find_place(self, text: str) -> Optional[str]:
        t = norm(text.replace("_", " "))
        for phrase, name in self.phrases:
            if phrase and phrase in t:
                return name
        return None

    def on_text(self, msg: String):
        raw = msg.data or ""
        text = norm(raw)
        if not text:
            return

        self.get_logger().info(f"heard: {raw}")

        # Demo direct motion commands. These go through /cmd_vel_omi -> arbiter -> collision monitor.
        if any(k in text for k in ["stop", "halt", "freeze"]):
            self.active_twist = None
            self.twist_pub.publish(Twist())
            self.publish_semantic("cancel")
            return

        dx = float(self.get_parameter("direct_x").value)
        dy = float(self.get_parameter("direct_y").value)
        dyaw = float(self.get_parameter("direct_yaw").value)

        if "move forward" in text or "go forward" in text or "forward a little" in text:
            self.start_direct_motion(x=dx)
            return
        if "move back" in text or "back up" in text or "go backward" in text:
            self.start_direct_motion(x=-dx)
            return
        if "move left" in text or "strafe left" in text:
            self.start_direct_motion(y=dy)
            return
        if "move right" in text or "strafe right" in text:
            self.start_direct_motion(y=-dy)
            return
        if "turn left" in text or "rotate left" in text:
            self.start_direct_motion(yaw=dyaw)
            return
        if "turn right" in text or "rotate right" in text:
            self.start_direct_motion(yaw=-dyaw)
            return

        # Navigation commands.
        wants_nav = any(k in text for k in [
            "go to", "goto", "navigate to", "take me to", "bring me to",
            "walk to", "lead me to", "show me"
        ])

        place = self.find_place(text)
        if wants_nav and place:
            self.publish_semantic(f"go to {place}")
            return

        if "start tour" in text or "begin tour" in text:
            self.publish_semantic("start tour")
            return
        if "pause tour" in text:
            self.publish_semantic("pause tour")
            return
        if "resume tour" in text or "continue tour" in text:
            self.publish_semantic("resume tour")
            return

        if place:
            # Useful for short Omi commands like "lecture hall".
            self.publish_semantic(f"go to {place}")
            return

        known = ", ".join([p.get("name", "") for p in self.places if p.get("name")])
        self.get_logger().warn(f"no_place_match text='{raw}' known=[{known}]")


def main(args=None):
    rclpy.init(args=args)
    node = OmiDemoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
