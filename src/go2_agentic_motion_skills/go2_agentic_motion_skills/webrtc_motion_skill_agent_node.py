from __future__ import annotations

import json
import re
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_interfaces.msg import WebRtcReq

from go2_robot_sdk.domain.constants.robot_commands import ROBOT_CMD


SPORT_TOPIC = "rt/api/sport/request"


# canonical_command -> WebRTC ROBOT_CMD name
CANONICAL_TO_WEBRTC_NAME = {
    "damp": "Damp",
    "balance_stand": "BalanceStand",
    "stop_move": "StopMove",
    "stand_up": "StandUp",
    "stand_down": "StandDown",
    "recovery_stand": "RecoveryStand",

    "euler": "Euler",
    "move": "Move",

    "sit": "Sit",
    "rise_sit": "RiseSit",

    "switch_gait": "SwitchGait",
    "trigger": "Trigger",
    "body_height": "BodyHeight",
    "foot_raise_height": "FootRaiseHeight",
    "speed_level": "SpeedLevel",

    "hello": "Hello",
    "stretch": "Stretch",
    "trajectory_follow": "TrajectoryFollow",
    "continuous_gait": "ContinuousGait",
    "content": "Content",
    "wallow": "Wallow",
    "dance1": "Dance1",
    "dance2": "Dance2",

    "get_body_height": "GetBodyHeight",
    "get_foot_raise_height": "GetFootRaiseHeight",
    "get_speed_level": "GetSpeedLevel",
    "switch_joystick": "SwitchJoystick",
    "pose": "Pose",

    "scrape": "Scrape",
    "front_flip": "FrontFlip",
    "front_jump": "FrontJump",
    "front_pounce": "FrontPounce",
    "wiggle_hips": "WiggleHips",
    "get_state": "GetState",
    "economic_gait": "EconomicGait",
    "heart": "FingerHeart",
    "finger_heart": "FingerHeart",

    "stand_out": "StandOut",
    "free_walk": "FreeWalk",
    "standup": "Standup",
    "cross_walk": "CrossWalk",

    "bound": "Bound",
    "moon_walk": "MoonWalk",
    "onesided_step": "OnesidedStep",
    "cross_step": "CrossStep",
    "hand_stand": "Handstand",
}


# These need a real parameter payload; do not run from simple voice.
PARAMETER_REQUIRED = {
    "euler",
    "move",
    "switch_gait",
    "trigger",
    "body_height",
    "foot_raise_height",
    "speed_level",
    "trajectory_follow",
    "continuous_gait",
    "switch_joystick",
    "pose",
    "economic_gait",
}


QUERY_COMMANDS = {
    "get_body_height",
    "get_foot_raise_height",
    "get_speed_level",
    "get_state",
}


HIGH_RISK = {
    "front_flip",
    "front_jump",
    "front_pounce",
    "hand_stand",
    "cross_step",
    "bound",
    "moon_walk",
    "onesided_step",
    "cross_walk",
}


MEDIUM_RISK = {
    "dance1",
    "dance2",
    "scrape",
    "wiggle_hips",
    "wallow",
    "free_walk",
    "stand_out",
    "standup",
}


ALIASES = {
    "stop": "stop_move",
    "stop move": "stop_move",
    "halt": "stop_move",
    "freeze": "stop_move",

    "damp": "damp",
    "relax": "damp",

    "balance": "balance_stand",
    "balance stand": "balance_stand",
    "hold still": "balance_stand",

    "stand": "stand_up",
    "stand up": "stand_up",
    "get up": "stand_up",

    "stand down": "stand_down",
    "lie down": "stand_down",
    "sleep": "stand_down",

    "recover": "recovery_stand",
    "recovery": "recovery_stand",
    "recovery stand": "recovery_stand",

    "sit": "sit",
    "sit down": "sit",
    "rise sit": "rise_sit",

    "hello": "hello",
    "say hello": "hello",
    "wave": "hello",
    "greet": "hello",

    "stretch": "stretch",
    "content": "content",
    "happy": "content",

    "wallow": "wallow",
    "roll": "wallow",

    "dance": "dance1",
    "dance one": "dance1",
    "dance two": "dance2",

    "scrape": "scrape",

    "front flip": "front_flip",
    "frontflip": "front_flip",
    "do a front flip": "front_flip",
    "front jump": "front_jump",
    "jump": "front_jump",
    "front pounce": "front_pounce",
    "pounce": "front_pounce",

    "wiggle hips": "wiggle_hips",
    "wiggle": "wiggle_hips",

    "heart": "heart",
    "finger heart": "finger_heart",

    "stand out": "stand_out",
    "free walk": "free_walk",
    "special standup": "standup",
    "cross walk": "cross_walk",
    "bound": "bound",
    "moon walk": "moon_walk",
    "moonwalk": "moon_walk",
    "one sided step": "onesided_step",
    "onesided step": "onesided_step",
    "cross step": "cross_step",
    "handstand": "hand_stand",
    "hand stand": "hand_stand",

    "get state": "get_state",
    "state": "get_state",
    "get body height": "get_body_height",
    "get foot raise height": "get_foot_raise_height",
    "get speed level": "get_speed_level",
}


SCRIPTS = {
    "tour_greet": ["stand_up", "hello", "stretch", "balance_stand"],
    "tour_handshake": ["stand_up", "hello", "balance_stand"],
    "tour_attention": ["hello", "content", "balance_stand"],
    "tour_pause": ["balance_stand"],
    "tour_resume": ["stand_up", "balance_stand", "hello"],
    "tour_handoff": ["stand_up", "hello", "content", "balance_stand"],
    "tour_ack": ["hello", "content"],
    "tour_settle": ["sit"],
    "stand_ready": ["stand_up", "balance_stand"],
    "wait": ["balance_stand"],
    "blocked_route_recovery": ["recovery_stand", "stand_up", "balance_stand"],
}


def normalize_command(text: str) -> str:
    value = (text or "").strip().lower()
    value = value.replace("-", " ").replace("_", " ")
    value = re.sub(r"[^a-z0-9\s]", " ", value)
    value = re.sub(r"\s+", " ", value).strip()
    return ALIASES.get(value, value.replace(" ", "_"))


def encode_parameter(parameter: Any) -> str:
    if parameter is None:
        return ""
    if isinstance(parameter, str):
        stripped = parameter.strip()
        if not stripped:
            return ""
        try:
            json.loads(stripped)
            return stripped
        except Exception:
            return json.dumps(stripped)
    return json.dumps(parameter)


class WebRtcMotionSkillAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_motion_skill_agent_node")

        self.declare_parameter("command_topic", "/motion_skills/command")
        self.declare_parameter("webrtc_req_topic", "/webrtc_req")
        self.declare_parameter("sport_topic", SPORT_TOPIC)
        self.declare_parameter("script_step_pause_sec", 0.60)
        self.declare_parameter("allow_medium_risk", True)
        self.declare_parameter("allow_high_risk", False)
        self.declare_parameter("allow_parameter_commands", False)
        self.declare_parameter("allow_query_commands", True)

        self.req_pub = self.create_publisher(
            WebRtcReq,
            str(self.get_parameter("webrtc_req_topic").value),
            20,
        )
        self.reply_pub = self.create_publisher(String, "/agent/reply", 20)
        self.status_pub = self.create_publisher(String, "/motion_skills/status", 20)

        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self.command_cb,
            20,
        )

        self.get_logger().info(
            "ready | WebRTC motion skills enabled | "
            f"command_topic={self.get_parameter('command_topic').value} "
            f"webrtc_req_topic={self.get_parameter('webrtc_req_topic').value}"
        )

    def publish_text(self, pub, text: str) -> None:
        pub.publish(String(data=text))
        self.get_logger().info(text)

    def command_cb(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        threading.Thread(target=self.handle_command, args=(raw,), daemon=True).start()

    def parse_payload(self, raw: str) -> tuple[str, dict[str, Any]]:
        if raw.startswith("{"):
            try:
                payload = json.loads(raw)
                if isinstance(payload, dict):
                    cmd = str(
                        payload.get("command")
                        or payload.get("skill")
                        or payload.get("type")
                        or payload.get("name")
                        or payload.get("script")
                        or ""
                    ).strip()
                    return cmd, payload
            except Exception:
                pass
        return raw, {}

    def handle_command(self, raw: str) -> None:
        cmd, payload = self.parse_payload(raw)
        skill = normalize_command(cmd)

        try:
            if payload.get("sequence") and isinstance(payload["sequence"], list):
                steps = [normalize_command(str(x)) for x in payload["sequence"]]
                self.execute_script("custom_sequence", steps)
                return

            if skill in SCRIPTS:
                self.execute_script(skill, SCRIPTS[skill])
                return

            parameter = payload.get("parameter", None)
            self.execute_skill(skill, parameter=parameter)
            self.publish_text(self.reply_pub, f"Completed {skill}.")
            self.publish_text(self.status_pub, f"completed {skill}")

        except KeyError:
            self.publish_text(self.reply_pub, f"I do not know the motion skill {raw}.")
            self.publish_text(self.status_pub, f"unknown {raw}")

        except Exception as exc:
            self.publish_text(self.reply_pub, f"Motion skill {skill} failed: {exc}")
            self.publish_text(self.status_pub, f"failed {skill}: {exc}")

    def execute_script(self, label: str, steps: list[str]) -> None:
        self.publish_text(self.status_pub, f"executing script {label}")
        for step in steps:
            self.execute_skill(step, parameter=None)
            time.sleep(float(self.get_parameter("script_step_pause_sec").value))
        self.publish_text(self.reply_pub, f"Completed {label}.")
        self.publish_text(self.status_pub, f"completed {label}")

    def execute_skill(self, skill: str, parameter: Any = None) -> None:
        if skill in QUERY_COMMANDS and not bool(self.get_parameter("allow_query_commands").value):
            raise RuntimeError(f"{skill} is blocked because allow_query_commands is false")

        if skill in PARAMETER_REQUIRED:
            if not bool(self.get_parameter("allow_parameter_commands").value):
                raise RuntimeError(f"{skill} requires parameter payload and allow_parameter_commands is false")
            if parameter is None:
                raise RuntimeError(f"{skill} requires a JSON parameter payload")

        if skill in HIGH_RISK and not bool(self.get_parameter("allow_high_risk").value):
            raise RuntimeError(f"{skill} is blocked because allow_high_risk is false")

        if skill in MEDIUM_RISK and not bool(self.get_parameter("allow_medium_risk").value):
            raise RuntimeError(f"{skill} is blocked because allow_medium_risk is false")

        webrtc_name = CANONICAL_TO_WEBRTC_NAME.get(skill)
        if not webrtc_name:
            raise KeyError(skill)

        api_id = ROBOT_CMD.get(webrtc_name)
        if api_id is None:
            raise KeyError(f"{skill} -> {webrtc_name}")

        msg = WebRtcReq()
        msg.id = 0
        msg.topic = str(self.get_parameter("sport_topic").value)
        msg.api_id = int(api_id)
        msg.parameter = encode_parameter(parameter)
        msg.priority = 1

        self.req_pub.publish(msg)

        self.publish_text(
            self.status_pub,
            json.dumps(
                {
                    "event": "webrtc_motion_skill_sent",
                    "skill": skill,
                    "webrtc_name": webrtc_name,
                    "api_id": int(api_id),
                    "topic": msg.topic,
                    "parameter": msg.parameter,
                    "risk": (
                        "high" if skill in HIGH_RISK
                        else "medium" if skill in MEDIUM_RISK
                        else "query" if skill in QUERY_COMMANDS
                        else "parameter" if skill in PARAMETER_REQUIRED
                        else "low"
                    ),
                },
                sort_keys=True,
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebRtcMotionSkillAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
