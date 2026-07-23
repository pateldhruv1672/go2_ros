from __future__ import annotations

import re
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def clean_text(text: str) -> str:
    text = (text or "").strip().lower()
    text = re.sub(r"[,.?!]", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def strip_wake_words(text: str) -> str:
    text = clean_text(text)
    for word in ("sparky", "sparkie", "robot", "go2", "dog", "please"):
        text = re.sub(rf"\b{re.escape(word)}\b", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def slug_label(text: str) -> str:
    text = clean_text(text)
    text = re.sub(r"[^a-z0-9]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")
    return text or "place"


class SemanticOmiCommandRouter(Node):
    """
    Sticky Omi/text router.

    Flow:
      real Omi/STT topic
        -> this router
        -> /omi/transcript canonical debug topic
        -> /semantic_nav/command
        -> existing semantic_nav_node

    This node does not launch Nav2, AMCL, SLAM, map_server, RViz, or robot driver.
    """

    def __init__(self) -> None:
        super().__init__("semantic_omi_command_router")

        self.declare_parameter("explicit_omi_topic", "")
        self.declare_parameter(
            "candidate_omi_topics",
            "/omi/transcript,/omi/transcript_raw,/go2_voice/transcript,"
            "/voice/transcript,/stt/transcript,/speech_to_text/transcript,"
            "/transcript,/speech/text",
        )
        self.declare_parameter("auto_discover_omi", True)
        self.declare_parameter("discovery_regex", "omi|transcript|speech|voice|stt")

        self.declare_parameter("semantic_command_topic", "/semantic_nav/command")
        self.declare_parameter("status_topic", "/semantic_nav/omi_status")
        self.declare_parameter("canonical_transcript_topic", "/omi/transcript")

        self.declare_parameter("require_wake_word", False)

        # For tiny demo motions like "turn left" or "move forward a little".
        # Keep this as pre-arbiter/pre-collision-monitor topic. Do not use /cmd_vel_out.
        self.declare_parameter("enable_direct_motion", True)
        self.declare_parameter("motion_cmd_topic", "/cmd_vel_nav2")
        self.declare_parameter("direct_linear_speed", 0.08)
        self.declare_parameter("direct_angular_speed", 0.35)
        self.declare_parameter("direct_short_duration_sec", 0.8)
        self.declare_parameter("direct_forward_duration_sec", 1.2)
        self.declare_parameter("direct_backup_duration_sec", 0.9)

        self.command_pub = self.create_publisher(
            String,
            str(self.get_parameter("semantic_command_topic").value),
            20,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            20,
        )
        self.canonical_transcript_pub = self.create_publisher(
            String,
            str(self.get_parameter("canonical_transcript_topic").value),
            20,
        )
        self.motion_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("motion_cmd_topic").value),
            10,
        )

        self._topic_subscriptions: Dict[str, object] = {}
        self._connected_topic: Optional[str] = None
        self._last_wait_log_sec = 0.0

        self._motion_active = False
        self._motion_end_ns = 0
        self._motion_twist = Twist()

        self._subscribe_initial_topics()

        self.create_timer(1.0, self.discovery_tick)
        self.create_timer(0.05, self.motion_tick)

        self.publish_status("omi_router_started waiting_for_omi_topic")
        self.get_logger().info("semantic_omi_command_router started")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_semantic_command(self, command: str, source_text: str) -> None:
        command = command.strip()
        if not command:
            return

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        self.publish_status(
            f"routed_omi_command raw='{source_text}' semantic='{command}'"
        )

    def _subscribe_initial_topics(self) -> None:
        explicit = str(self.get_parameter("explicit_omi_topic").value).strip()
        if explicit:
            self._subscribe_topic(explicit)
            return

        candidates = str(self.get_parameter("candidate_omi_topics").value)
        for topic in [x.strip() for x in candidates.split(",") if x.strip()]:
            self._subscribe_topic(topic)

    def _subscribe_topic(self, topic: str) -> None:
        if not topic or topic in self._topic_subscriptions:
            return

        self._topic_subscriptions[topic] = self.create_subscription(
            String,
            topic,
            lambda msg, t=topic: self.transcript_cb(msg, t),
            10,
        )
        self.get_logger().info(f"subscribed_candidate_omi_topic topic={topic}")

    def discovery_tick(self) -> None:
        if not bool(self.get_parameter("auto_discover_omi").value):
            return

        pattern = str(self.get_parameter("discovery_regex").value)
        regex = re.compile(pattern, re.IGNORECASE)

        try:
            topics = self.get_topic_names_and_types()
        except Exception as exc:
            self.get_logger().warn(f"topic_discovery_failed error={exc}")
            return

        discovered = []
        for topic, types in topics:
            if not regex.search(topic):
                continue
            if "std_msgs/msg/String" not in list(types):
                continue
            discovered.append(topic)
            self._subscribe_topic(topic)

        now = time.monotonic()
        if self._connected_topic is None and now - self._last_wait_log_sec > 3.0:
            self._last_wait_log_sec = now
            if discovered:
                self.publish_status(
                    "omi_router_waiting_for_message discovered="
                    + ",".join(sorted(set(discovered)))
                )
            else:
                self.publish_status("omi_router_waiting_for_omi_topic")

    def transcript_cb(self, msg: String, topic: str) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return

        if self._connected_topic != topic:
            self._connected_topic = topic
            self.publish_status(f"omi_connected topic={topic}")

        canonical = String()
        canonical.data = raw
        self.canonical_transcript_pub.publish(canonical)

        raw_clean = clean_text(raw)
        if bool(self.get_parameter("require_wake_word").value):
            if "sparky" not in raw_clean and "sparkie" not in raw_clean:
                self.publish_status(f"ignored_no_wake_word raw='{raw}'")
                return

        command, direct_motion = self.parse_command(raw)

        if direct_motion is not None:
            linear_x, angular_z, duration = direct_motion
            self.publish_semantic_command("cancel", raw)
            self.start_direct_motion(linear_x, angular_z, duration, raw)
            return

        if command:
            self.publish_semantic_command(command, raw)
        else:
            self.publish_status(f"ignored_unrecognized_omi_text raw='{raw}'")

    def parse_command(self, raw: str) -> Tuple[Optional[str], Optional[Tuple[float, float, float]]]:
        text = strip_wake_words(raw)

        if not text:
            return None, None

        if any(x in text for x in ("emergency stop", "stop", "halt", "freeze", "cancel", "abort")):
            return "cancel", (0.0, 0.0, 0.05)

        if any(x in text for x in ("status", "are you ready", "system check")):
            return "status", None

        if any(x in text for x in ("what do you see", "describe", "explain this", "where are we")):
            return "describe", None

        if "start tour" in text or "begin tour" in text:
            return "start_tour", None

        if "pause tour" in text or text == "pause":
            return "pause_tour", None

        if "resume tour" in text or "continue tour" in text:
            return "resume_tour", None

        if "next stop" in text or "advance tour" in text:
            return "next_stop", None

        if "save spawn" in text or "save this as spawn" in text:
            return "save_spawn", None

        m = re.search(r"\bsave\s+(this|here|current location)?\s*(as)?\s+(.+)$", text)
        if m:
            label = slug_label(m.group(3))
            return f"save {label}", None

        if bool(self.get_parameter("enable_direct_motion").value):
            direct = self.parse_direct_motion(text)
            if direct is not None:
                return None, direct

        nav_target = self.extract_nav_target(text)
        if nav_target:
            return f"go {nav_target}", None

        return None, None

    def extract_nav_target(self, text: str) -> Optional[str]:
        text = clean_text(text)

        if text in ("spawn", "home", "start"):
            return "spawn"

        if "return to spawn" in text or "go back to spawn" in text or "go home" in text:
            return "spawn"

        patterns = [
            r"^go to (.+)$",
            r"^go (.+)$",
            r"^navigate to (.+)$",
            r"^navigate (.+)$",
            r"^take me to (.+)$",
            r"^drive to (.+)$",
            r"^move to (.+)$",
            r"^bring me to (.+)$",
        ]

        for pattern in patterns:
            m = re.search(pattern, text)
            if m:
                target = m.group(1).strip()
                target = re.sub(r"^(the|a|an)\s+", "", target)
                return target if target else None

        return None

    def parse_direct_motion(self, text: str) -> Optional[Tuple[float, float, float]]:
        linear = float(self.get_parameter("direct_linear_speed").value)
        angular = float(self.get_parameter("direct_angular_speed").value)
        short_dur = float(self.get_parameter("direct_short_duration_sec").value)
        forward_dur = float(self.get_parameter("direct_forward_duration_sec").value)
        backup_dur = float(self.get_parameter("direct_backup_duration_sec").value)

        if any(x in text for x in ("move forward", "go forward", "forward a little", "forward")):
            return linear, 0.0, forward_dur

        if any(x in text for x in ("back up", "backup", "move back", "go back a little")):
            return -linear, 0.0, backup_dur

        if any(x in text for x in ("turn left", "rotate left", "left a little")):
            return 0.0, angular, short_dur

        if any(x in text for x in ("turn right", "rotate right", "right a little")):
            return 0.0, -angular, short_dur

        return None

    def start_direct_motion(self, linear_x: float, angular_z: float, duration_sec: float, raw: str) -> None:
        self._motion_twist = Twist()
        self._motion_twist.linear.x = float(linear_x)
        self._motion_twist.angular.z = float(angular_z)
        self._motion_end_ns = self.get_clock().now().nanoseconds + int(max(0.05, duration_sec) * 1e9)
        self._motion_active = True

        self.publish_status(
            f"direct_motion_start raw='{raw}' linear_x={linear_x:.3f} "
            f"angular_z={angular_z:.3f} duration={duration_sec:.2f} "
            f"topic={self.get_parameter('motion_cmd_topic').value}"
        )

    def motion_tick(self) -> None:
        if not self._motion_active:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self._motion_end_ns:
            self.motion_pub.publish(Twist())
            self._motion_active = False
            self.publish_status("direct_motion_done")
            return

        self.motion_pub.publish(self._motion_twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticOmiCommandRouter()
    try:
        rclpy.spin(node)
    finally:
        node.motion_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
