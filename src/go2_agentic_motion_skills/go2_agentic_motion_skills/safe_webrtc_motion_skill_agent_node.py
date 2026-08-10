from __future__ import annotations

import json
import threading
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from go2_agentic_motion_skills.webrtc_motion_skill_agent_node import (
    QUERY_COMMANDS,
    SCRIPTS,
    WebRtcMotionSkillAgentNode,
    normalize_command,
)


class SafeWebRtcMotionSkillAgentNode(WebRtcMotionSkillAgentNode):
    """WebRTC Sport executor with semantic-nav interlock and cancelable scripts."""

    def __init__(self) -> None:
        self.navigation_active = False
        self._script_cancel = threading.Event()
        self._execution_lock = threading.Lock()
        super().__init__()
        self.declare_parameter("allow_during_navigation", False)
        self.declare_parameter("semantic_nav_status_topic", "/semantic_nav/status")
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_nav_status_topic").value),
            self._on_semantic_nav_status,
            20,
        )
        self.get_logger().info(
            "Safe WebRTC motion wrapper active: semantic-nav interlock enabled; "
            f"allow_medium_risk={self.get_parameter('allow_medium_risk').value} "
            f"allow_high_risk={self.get_parameter('allow_high_risk').value}"
        )

    def _on_semantic_nav_status(self, msg: String) -> None:
        text = (msg.data or "").strip().lower()
        if not text:
            return
        if "sending_goal" in text or "navigating target=" in text or text.startswith("navigating"):
            self.navigation_active = True
            return
        terminal = (
            "goal succeeded", "goal failed", "goal rejected", "goal canceled",
            "tour_stop_complete", "route_complete", "tour_paused", "route cancelled",
            "cancelling_goal", "cancel_goal",
        )
        if any(token in text for token in terminal):
            self.navigation_active = False

    def command_cb(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        cmd, _payload = self.parse_payload(raw)
        skill = normalize_command(cmd)
        if skill == "stop_move":
            self._script_cancel.set()
        threading.Thread(target=self.handle_command, args=(raw,), daemon=True).start()

    def handle_command(self, raw: str) -> None:
        cmd, _payload = self.parse_payload(raw)
        skill = normalize_command(cmd)
        with self._execution_lock:
            if skill != "stop_move":
                self._script_cancel.clear()
            super().handle_command(raw)

    def execute_script(self, label: str, steps: list[str]) -> None:
        self.publish_text(self.status_pub, f"executing script {label}")
        for step in steps:
            if self._script_cancel.is_set():
                self.publish_text(self.status_pub, f"canceled script {label}")
                return
            self.execute_skill(step, parameter=None)
            pause = float(self.get_parameter("script_step_pause_sec").value)
            if self._script_cancel.wait(timeout=max(0.0, pause)):
                self.publish_text(self.status_pub, f"canceled script {label}")
                return
        self.publish_text(self.reply_pub, f"Completed {label}.")
        self.publish_text(self.status_pub, f"completed {label}")

    def execute_skill(self, skill: str, parameter: Any = None) -> None:
        if (
            self.navigation_active
            and not bool(self.get_parameter("allow_during_navigation").value)
            and skill not in QUERY_COMMANDS
            and skill != "stop_move"
        ):
            raise RuntimeError(
                f"{skill} blocked while semantic navigation is active; stop_move remains available"
            )
        return super().execute_skill(skill, parameter=parameter)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeWebRtcMotionSkillAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
