from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from go2_memory_core.session_resolution import resolve_semantic_session_name

SAFE_MOVES_ALLOWLIST = {"hello", "stretch", "heart", "finger_heart", "sit", "stand_up", "balance_stand", "content"}


class TourHostScriptNode(Node):
    """Executes verified guest-facing Tour Host scripts without owning navigation."""
    def __init__(self) -> None:
        super().__init__("go2_tour_host_script")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "latest")
        self.declare_parameter("speech_topic", "/go2_speech/request")
        self.declare_parameter("motion_topic", "/motion_skills/command")
        self.speech_pub = self.create_publisher(String, str(self.get_parameter("speech_topic").value), 10)
        self.motion_pub = self.create_publisher(String, str(self.get_parameter("motion_topic").value), 10)
        self.status_pub = self.create_publisher(String, "/go2_tour/host_status", 10)
        self.create_subscription(String, "/go2_tour/host_command", self._on_command, 10)
        self.get_logger().info("Tour Host script executor ready; navigation remains owned by semantic_nav_node")

    def _session_dir(self) -> Path:
        root = str(self.get_parameter("session_root").value)
        requested = str(self.get_parameter("session_name").value)
        resolved = resolve_semantic_session_name(root, requested)
        return Path(os.path.expanduser(root)).resolve() / resolved

    def _scripts(self) -> dict[str, Any]:
        path = self._session_dir() / "tour_host.yaml"
        if path.is_file():
            value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if isinstance(value, dict) and isinstance(value.get("scripts"), dict):
                return value["scripts"]
        return {
            "welcome": [{"say": "Welcome. I am Sparky, your Unitree Go2 tour guide."}],
            "full_intro": [{"say": "Welcome. I am Sparky. I can guide you through saved tour stops, answer questions from memory, and describe what my camera sees live."}],
            "lab_intro": [{"say": "This stop is part of my saved lab tour. I will use the verified tour memory and current observations when I describe it."}],
            "research_intro": [{"say": "I can explain verified research information stored for this tour and distinguish it from what I can currently see."}],
            "sparky_intro": [{"say": "I am Sparky, a Unitree Go2 using ROS 2, semantic memory, live vision, and Nav2 for this tour."}],
            "capabilities": [{"say": "You can ask me where we are, what I remember, what I can see now, where an object was observed, or ask me to navigate after confirmation."}],
            "safe_moves": [
                {"say": "Here are a few of my safe demonstration moves."},
                {"motion": "hello"},
                {"motion": "stretch"},
                {"motion": "heart"},
            ],
        }

    def _say(self, text: str, script: str, request_id: str) -> None:
        if not text.strip():
            return
        payload = {"text": text.strip(), "source": "tour_host_script", "category": "tour_host", "priority": "normal", "script": script, "request_id": request_id}
        self.speech_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data) if str(msg.data or "").strip().startswith("{") else {"script": msg.data}
        except Exception:
            payload = {"script": msg.data}
        if not isinstance(payload, dict):
            return
        script = str(payload.get("script") or "full_intro").strip()
        verified = bool(payload.get("verified", False))
        safety_checked = bool(payload.get("safety_checked", False))
        request_id = str(payload.get("request_id") or f"tour_host_{time.time_ns()}")
        steps = self._scripts().get(script)
        if steps is None:
            self._say(f"I do not have a verified Tour Host script named {script.replace('_', ' ')}.", script, request_id)
            self.status_pub.publish(String(data=json.dumps({"event": "unknown_script", "script": script}, sort_keys=True)))
            return
        if isinstance(steps, (str, dict)):
            steps = [steps]
        if not isinstance(steps, list):
            return
        motion_count = 0
        speech_count = 0
        for step in steps:
            if isinstance(step, str):
                self._say(step, script, request_id); speech_count += 1; continue
            if not isinstance(step, dict):
                continue
            text = str(step.get("say") or step.get("speech") or step.get("narration") or "").strip()
            if text:
                self._say(text, script, request_id); speech_count += 1
            skill = str(step.get("motion") or step.get("skill") or "").strip()
            if skill:
                if script == "safe_moves" and skill not in SAFE_MOVES_ALLOWLIST:
                    self.status_pub.publish(String(data=json.dumps({"event": "safe_moves_blocked_skill", "skill": skill}, sort_keys=True)))
                    self._say(f"I skipped {skill.replace(chr(95), chr(32))} because it is not in my no-confirm safe-moves allowlist.", script, request_id)
                    continue
                if not (verified and safety_checked):
                    self._say("I skipped the motion part because it was not verified by the safety gate.", script, request_id)
                    continue
                self.motion_pub.publish(String(data=json.dumps({"command": skill, "source": "tour_host_script", "verified": True, "safety_checked": True}, sort_keys=True)))
                motion_count += 1
        self.status_pub.publish(String(data=json.dumps({"event": "script_queued", "script": script, "speech_steps": speech_count, "motion_steps": motion_count, "request_id": request_id}, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TourHostScriptNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
