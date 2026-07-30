from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AgenticExplorerSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("go2_agentic_explorer_supervisor")
        self.declare_parameter("candidates_topic", "/mrkl_explorer/safe_frontier_candidates")
        self.declare_parameter("command_topic", "/go2_autonomy/command")
        self.declare_parameter("nav_command_topic", "/go2_nav/command")
        self.declare_parameter("nav_status_topic", "/go2_nav/status")
        self.declare_parameter("state_topic", "/go2_autonomy/state")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("candidate_ttl_sec", 3.0)
        self.declare_parameter("min_goal_interval_sec", 2.0)
        self.declare_parameter("max_consecutive_failures", 3)
        self.declare_parameter("blacklist_radius_m", 0.75)
        self.declare_parameter("recovery_pause_sec", 5.0)
        self.declare_parameter("state_path", "~/.ros/go2_object_explorer/agentic_explorer_state.json")

        self.active = bool(self.get_parameter("auto_start").value)
        self.state = "SELECT" if self.active else "IDLE"
        self.candidates: List[Dict[str, Any]] = []
        self.candidate_stamp = 0.0
        self.blacklist: List[Tuple[float, float]] = []
        self.active_goal: Optional[Dict[str, Any]] = None
        self.last_goal_sent = 0.0
        self.failures = 0
        self.resume_after = 0.0

        self.nav_pub = self.create_publisher(String, str(self.get_parameter("nav_command_topic").value), 20)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 20)
        self.create_subscription(String, str(self.get_parameter("candidates_topic").value), self.on_candidates, 10)
        self.create_subscription(String, str(self.get_parameter("command_topic").value), self.on_command, 10)
        self.create_subscription(String, str(self.get_parameter("nav_status_topic").value), self.on_nav_status, 20)
        self.create_timer(0.25, self.tick)
        self.publish_state("started")
        self.get_logger().info("agentic explorer supervisor ready; starts idle unless auto_start:=true")

    def publish_json(self, pub, payload: Dict[str, Any]) -> None:
        pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_state(self, reason: str = "") -> None:
        payload = {
            "stamp": time.time(),
            "state": self.state,
            "active": self.active,
            "reason": reason,
            "candidate_count": len(self.candidates),
            "blacklist_count": len(self.blacklist),
            "consecutive_failures": self.failures,
            "active_goal": self.active_goal,
        }
        self.publish_json(self.state_pub, payload)
        path = os.path.expanduser(str(self.get_parameter("state_path").value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        os.replace(tmp, path)

    def on_candidates(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            items = payload.get("candidates", []) if isinstance(payload, dict) else []
            self.candidates = [item for item in items if isinstance(item, dict)]
            self.candidate_stamp = time.time()
        except Exception as exc:
            self.get_logger().warn(f"invalid frontier candidate payload: {exc}")

    def on_command(self, msg: String) -> None:
        raw = msg.data.strip()
        try:
            command = json.loads(raw)
            action = str(command.get("action", "")) if isinstance(command, dict) else raw
        except Exception:
            action = raw
        action = action.lower().strip()
        if action in {"start", "explore", "resume", "start_exploration"}:
            self.active = True
            self.state = "SELECT"
            self.failures = 0
            self.publish_state("operator_start")
        elif action in {"stop", "pause", "cancel"}:
            self.active = False
            self.state = "IDLE"
            self.active_goal = None
            self.publish_json(self.nav_pub, {"action": "stop_robot", "reason": "operator_stop"})
            self.publish_state("operator_stop")
        elif action in {"clear_blacklist", "reset"}:
            self.blacklist.clear()
            self.failures = 0
            self.publish_state("blacklist_cleared")

    def on_nav_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(status, dict):
            return
        action = status.get("action")
        if action == "goal_accepted":
            self.state = "NAVIGATING"
            self.publish_state("goal_accepted")
        elif action in {"preflight_no_path", "preflight_rejected", "goal_rejected", "goal_response_error", "goal_result_error"}:
            self.handle_failure(action)
        elif action == "goal_result":
            if int(status.get("status", -1)) == 4:
                self.failures = 0
                self.active_goal = None
                self.state = "SELECT" if self.active else "IDLE"
                self.publish_state("goal_succeeded")
            else:
                self.handle_failure(f"goal_status_{status.get('status')}")

    def handle_failure(self, reason: str) -> None:
        if self.active_goal is not None:
            self.blacklist.append((float(self.active_goal["x"]), float(self.active_goal["y"])))
            self.blacklist = self.blacklist[-100:]
        self.active_goal = None
        self.failures += 1
        maximum = int(self.get_parameter("max_consecutive_failures").value)
        if self.failures >= maximum:
            self.state = "RECOVERY"
            self.publish_json(self.nav_pub, {"action": "recover_nav_failure", "reason": reason})
            self.resume_after = time.time() + float(self.get_parameter("recovery_pause_sec").value)
        else:
            self.state = "SELECT"
        self.publish_state(reason)

    def is_blacklisted(self, x: float, y: float) -> bool:
        radius = float(self.get_parameter("blacklist_radius_m").value)
        return any(math.hypot(x - bx, y - by) <= radius for bx, by in self.blacklist)

    def choose(self) -> Optional[Dict[str, Any]]:
        valid = []
        for item in self.candidates:
            try:
                x = float(item["x"])
                y = float(item["y"])
                score = float(item.get("score", 0.0))
            except Exception:
                continue
            if not math.isfinite(x) or not math.isfinite(y) or self.is_blacklisted(x, y):
                continue
            candidate = dict(item)
            candidate["x"] = x
            candidate["y"] = y
            candidate["score"] = score
            valid.append(candidate)
        return max(valid, key=lambda item: item["score"], default=None)

    def tick(self) -> None:
        now = time.time()
        if not self.active:
            return
        if self.state == "RECOVERY":
            if now >= self.resume_after:
                self.failures = 0
                self.state = "SELECT"
                self.publish_state("recovery_pause_complete")
            return
        if self.state != "SELECT" or self.active_goal is not None:
            return
        if now - self.candidate_stamp > float(self.get_parameter("candidate_ttl_sec").value):
            return
        if now - self.last_goal_sent < float(self.get_parameter("min_goal_interval_sec").value):
            return
        choice = self.choose()
        if choice is None:
            self.state = "WAITING_FOR_FRONTIER"
            self.publish_state("no_safe_frontier")
            self.state = "SELECT"
            return
        raw_x = float(choice.get("raw_x", choice["x"]))
        raw_y = float(choice.get("raw_y", choice["y"]))
        yaw = math.atan2(raw_y - choice["y"], raw_x - choice["x"])
        self.active_goal = {"x": choice["x"], "y": choice["y"], "yaw": yaw, "score": choice["score"]}
        self.last_goal_sent = now
        self.state = "PREFLIGHT"
        self.publish_json(self.nav_pub, {
            "action": "frontier_explore",
            "pose": {"frame_id": "map", "x": choice["x"], "y": choice["y"], "yaw": yaw},
            "candidate": choice,
        })
        self.publish_state("frontier_selected")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgenticExplorerSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
