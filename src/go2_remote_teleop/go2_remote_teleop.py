#!/usr/bin/env python3
"""
Go2 Manual + LLM Controller (ROS2) - Semantic actions + multi-agent checks
- Manual teleop via keyboard (i/k/j/l/J/L)
- AI mode: VLM outputs semantic action (forward/backward/turn/strafe/stop)
- Planner -> Reviewer -> Reflection (need-based) before executing a move
- We map semantic action -> Twist ourselves
- Continuous /cmd_vel publishing at 20Hz (watchdog-friendly)
- Robust JSON parsing + strong debug
- Saves the exact image sent to VLM at /tmp/go2_last_vlm.jpg
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import base64
import threading
import sys
import termios
import tty
import select
import json
import time
import re
import ast
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

from openai import OpenAI

# ---------------- CONFIG ----------------
CLIENT = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key="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",
)

MODEL = "google/gemini-2.5-flash"  # Fast, multimodal vision model

# Adjust these for your bridge setup:
CMD_TOPIC = "/cmd_vel_joy"       # sometimes "/cmd_vel"
CAM_TOPIC = "/camera/image_raw"  # sometimes "/camera/color/image_raw"

# Camera resize for VLM
VLM_W, VLM_H = 320, 240
JPEG_QUALITY = 70

# Continuous publishing rate + safety timeout
PUBLISH_HZ = 20.0
CMD_TIMEOUT_S = 5.0  # Must be > MAX_MOVE_S to allow full movement duration

# AI pulse timing (used as defaults, but VLM can override)
DEFAULT_MOVE_S = 0.8
DEFAULT_SETTLE_S = 0.5
MIN_MOVE_S = 0.2
MAX_MOVE_S = 3.0

# Lidar (disabled - inaccurate)
# LIDAR_TOPIC = "/point_cloud2"
# MIN_SAFE_DIST_M = 0.15
# LIDAR_SAMPLE_STRIDE = 8
# LIDAR_MAX_POINTS = 20000

# Scout rotation config
SCOUT_ROTATION_DEGREES = 30  # Rotate 30° per scout observation (12 total for 360°)
SCOUT_ROTATION_WZ = 0.6     # Angular velocity for rotation
SCOUT_ROTATION_S = 1.0      # Time per rotation segment

# Multi-agent thresholds
AGENT_REVIEWER_ENABLED = True
AGENT_REFLECTION_ENABLED = True
AGENT_CONFIDENCE_REVIEW = 0.6
AGENT_CONFIDENCE_REFLECT = 0.5

# Manual keyboard -> Twist (tune for your robot)
MOVE_MAP = {
    'i': (0.25, 0.0, 0.0),    # forward
    'k': (-0.25, 0.0, 0.0),   # backward
    'j': (0.0, 0.0, 0.6),    # yaw left
    'l': (0.0, 0.0, -0.6),   # yaw right
    'J': (0.0, 0.4, 0.0),    # strafe left
    'L': (0.0, -0.4, 0.0),   # strafe right
    'stop': (0.0, 0.0, 0.0),
}

# AI semantic actions -> keyboard keys
ACTION_TO_KEY = {
    "forward": "i",
    "backward": "k",
    "turn_left": "j",
    "turn_right": "l",
    "strafe_left": "J",
    "strafe_right": "L",
    "stop": "stop",
}
VALID_ACTIONS = list(ACTION_TO_KEY.keys())


# ---------------- Helpers ----------------
def _strip_code_fences(s: str) -> str:
    s = (s or "").strip()
    if s.startswith("```"):
        s = re.sub(r"^```(?:json)?\s*", "", s, flags=re.IGNORECASE)
        s = re.sub(r"\s*```$", "", s)
    return s.strip()


def parse_loose_json(s: str) -> dict:
    """Parse JSON-ish model output robustly."""
    if s is None:
        raise ValueError("Model returned None content")
    s = s.strip()
    if not s:
        raise ValueError("Model returned empty content")

    s = _strip_code_fences(s)

    # 1) raw_decode (ignores trailing text)
    try:
        first = s.find("{")
        s2 = s[first:] if first != -1 else s
        obj, _ = json.JSONDecoder().raw_decode(s2)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass

    # 2) substring from first { to last }
    try:
        a, b = s.find("{"), s.rfind("}")
        if a != -1 and b != -1 and b > a:
            obj = json.loads(s[a:b + 1])
            if isinstance(obj, dict):
                return obj
    except Exception:
        pass

    # 3) python-ish dict
    try:
        obj = ast.literal_eval(s)
        if isinstance(obj, dict):
            return obj
    except Exception as e:
        raise ValueError(f"Could not parse output as JSON/dict. Last error: {e}")

    raise ValueError("Could not parse output as JSON/dict.")


def parse_step_goal(task: str):
    """
    Deprecated: VLM will now manage task progress dynamically.
    Kept for backwards compatibility but not used.
    """
    return None


# ---------------- Node ----------------
class Go2Agent(Node):
    def __init__(self):
        super().__init__("go2_agent_core")

        self.cmd_topic = CMD_TOPIC
        self.cam_topic = CAM_TOPIC

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Image, self.cam_topic, self.img_cb, qos_profile_sensor_data)
        # self.create_subscription(PointCloud2, LIDAR_TOPIC, self.lidar_cb, qos_profile_sensor_data)

        self.bridge = CvBridge()

        # Debug toggles
        self.debug = True
        self.save_vlm_images = True

        # Camera storage
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._debug_last_cam_print = 0.0

        # Scout mode
        self.scout_mode = False
        self.scout_observations = []  # Store observations at each angle
        self.scout_angle_current = 0  # Current angle (0-360)
        self.scout_locations = {}  # {angle: {description, targets, distance}}
        self.scout_targets_found = []  # List of target locations found during scout

        # Control state
        self.manual_mode = True
        self.estop = False

        self.task = "None"
        self.history = []
        self.subtask_count = 0  # Track subtask iterations (not steps)

        # Continuous publish state
        self._cmd_lock = threading.Lock()
        self._current_twist = Twist()
        self._last_cmd_time = time.monotonic()
        self.cmd_timeout_s = CMD_TIMEOUT_S

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_loop)

    def img_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        with self._frame_lock:
            self._latest_frame = cv_img

        if self.debug:
            now = time.monotonic()
            if now - self._debug_last_cam_print > 1.5:
                self._debug_last_cam_print = now
                h, w = cv_img.shape[:2]
                #print(f"[DBG] Camera frame: {w}x{h} topic={self.cam_topic}")

    def _publish_loop(self):
        if self.estop:
            self.pub.publish(Twist())
            return

        if self._obstacle_too_close():
            self.pub.publish(Twist())
            return

        # auto-stop on stale commands
        if (time.monotonic() - self._last_cmd_time) > self.cmd_timeout_s:
            self.pub.publish(Twist())
            return

        with self._cmd_lock:
            self.pub.publish(self._current_twist)

    def set_cmd(self, key: str, reason: str = ""):
        """Set the current twist (continuously published)."""
        if self.estop:
            key = "stop"

        if self._obstacle_too_close() and key != "stop":
            key = "stop"

        if key not in MOVE_MAP:
            key = "stop"

        vx, vy, wz = MOVE_MAP[key]
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)

        with self._cmd_lock:
            self._current_twist = t
            self._last_cmd_time = time.monotonic()

        if self.debug:
            extra = f" | {reason}" if reason else ""
            print(f"[ACT] key={key} vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}{extra}")

    def _obstacle_too_close(self) -> bool:
        # Disabled - lidar is inaccurate
        return False

    def stop_now(self, reason: str = ""):
        with self._cmd_lock:
            self._current_twist = Twist()
            self._last_cmd_time = time.monotonic()
        self.pub.publish(Twist())
        if self.debug:
            extra = f" | {reason}" if reason else ""
            print(f"[ACT] key=stop{extra}")

    def _get_latest_b64(self):
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            frame = self._latest_frame.copy()

        frame = cv2.resize(frame, (VLM_W, VLM_H))
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return None
        return base64.b64encode(buf).decode("utf-8")

    def _save_last_vlm_jpg(self):
        if not self.save_vlm_images:
            return
        try:
            with self._frame_lock:
                if self._latest_frame is None:
                    return
                frame = self._latest_frame.copy()

            frame = cv2.resize(frame, (VLM_W, VLM_H))
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                return

            Path("/tmp").mkdir(parents=True, exist_ok=True)
            Path("/tmp/go2_last_vlm.jpg").write_bytes(buf.tobytes())
            if self.debug:
                print(f"[DBG] Saved VLM image -> /tmp/go2_last_vlm.jpg ({len(buf)} bytes)")
        except Exception as e:
            if self.debug:
                print(f"[DBG] Failed saving VLM image: {e}")

    def scout_analyze(self) -> str:
        """
        Analyze current view during scout rotation.
        Identifies targets and stores location memory for later navigation.
        """
        scout_prompt = (
            "You are a scout for a Unitree Go2 robot doing a 360-degree environment scan.\n"
            "Identify: (1) Environment layout, (2) Any targets for the task.\n"
            "Return ONLY ONE JSON object.\n"
            "JSON keys:\n"
            "  angle: current rotation angle (0=front, 90=left, 180=back, 270=right)\n"
            "  observation: brief 1-sentence description\n"
            "  notable_features: list of objects, people, doors, items\n"
            "  target_objects: list of objects matching the task\n"
            "  target_distances: estimated distances to targets in meters\n"
            "  safety_hazards: any obstacles or hazards\n"
            "  position_description: spatial description for navigation (e.g., 'left corner with shelves')\n"
        )

        scout_user = (
            f"Robot scout scan at rotation angle {self.scout_angle_current}° (0=front, 90=left, 180=back, 270=right)\n"
            f"Task: {self.task}\n"
            "Identify: (1) Environment description, (2) Any targets for the task.\n"
            "If you see something matching the task, note its position and estimated distance."
        )

        scout_data, scout_err = self._agent_request("scout", scout_prompt, scout_user, b64=self._get_latest_b64(), max_tokens=400)
        if scout_data is None:
            return f"Error at {self.scout_angle_current}°"

        obs = scout_data.get("observation", "")
        features = scout_data.get("notable_features", [])
        targets = scout_data.get("target_objects", [])
        distances = scout_data.get("target_distances", [])
        safety = scout_data.get("safety_hazards", [])
        position_desc = scout_data.get("position_description", "")
        
        # Store location memory for navigation
        self.scout_locations[self.scout_angle_current] = {
            "observation": obs,
            "features": features,
            "targets": targets,
            "distances": distances,
            "safety": safety,
            "position_desc": position_desc
        }
        
        # Track found targets for later backtracking
        if targets and len(targets) > 0:
            for i, target in enumerate(targets):
                dist = distances[i] if i < len(distances) else "unknown"
                self.scout_targets_found.append({
                    "name": target,
                    "angle": self.scout_angle_current,
                    "distance": dist,
                    "position_desc": position_desc
                })
        
        return f"[{self.scout_angle_current}°] {obs} | Targets: {targets} @ {distances}m | Safety: {safety}"

    def _call_agent(self, system_prompt: str, user_text: str, b64: str, use_response_format: bool, max_tokens: int = 800):
        kwargs = dict(
            model=MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": [
                    {"type": "text", "text": user_text},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ]},
            ],
            temperature=0.2,
            max_tokens=max_tokens,
        )

        if use_response_format:
            kwargs["response_format"] = {"type": "json_object"}

        return CLIENT.chat.completions.create(
            extra_headers={
                "HTTP-Referer": "https://github.com",
                "X-OpenRouter-Title": "Go2 Autonomous Navigation"
            },
            **kwargs
        )

    def _agent_request(self, label: str, system_prompt: str, user_text: str, b64: str, max_tokens: int = 800):
        data = None
        last_err = None
        for attempt, use_rf in enumerate([True, False], start=1):
            try:
                resp = self._call_agent(system_prompt, user_text, b64, use_response_format=use_rf, max_tokens=max_tokens)
                choice = resp.choices[0]
                content = getattr(choice.message, "content", "") or ""
                fr = getattr(choice, "finish_reason", None)

                if self.debug:
                    preview = content if len(content) < 800 else content[:800] + "..."
                    print(f"[DBG] {label} attempt={attempt} response_format={use_rf} finish_reason={fr}")
                    print(f"[DBG] {label} RAW OUTPUT:\n{preview!r}")

                data = parse_loose_json(content)
                break
            except Exception as e:
                last_err = e
                if self.debug:
                    print(f"[DBG] {label} attempt={attempt} failed: {e}")

        return data, last_err

    def get_ai_decision(self):
        """
        Multi-stage pipeline:
        1. Perceiver -> analyzes current state and generates detailed perception
        2. Reviewer -> validates/modifies the perception and proposes actions
        3. Executor -> generates final action based on reviewed plan
        4. Reflection -> checks safety if needed
        
        Returns: (action, mapped_key, duration, explanation, subtask_done, task_done, confidence, risk, source)
        """
        b64 = self._get_latest_b64()
        if not b64:
            if self.debug:
                print("[DBG] No camera frame yet -> stop")
            return "stop", "stop", DEFAULT_MOVE_S, "No camera frame yet.", False, False, 0.0, "high", "fallback"

        self._save_last_vlm_jpg()

        if self.debug:
            print(f"[DBG] Starting multi-stage decision pipeline")
            print(f"[DBG] Task='{self.task}' subtask_count={self.subtask_count}")
            print(f"[DBG] b64_len={len(b64)} head='{b64[:24]}...'")

        # Scout observations from earlier 360-degree rotation
        scout_text = "No scout data available" if not self.scout_observations else " | ".join(self.scout_observations[-12:])
        
        # Helper functions
        def _normalize_risk(value: str) -> str:
            v = (value or "").strip().lower()
            if v in ("low", "medium", "high"):
                return v
            if v in ("med", "mid"):
                return "medium"
            return "low"

        def _coerce_action(data: dict) -> str:
            act = str(data.get("action", "")).strip()
            return act if act in VALID_ACTIONS else "stop"

        def _coerce_duration(data: dict) -> float:
            try:
                val = float(data.get("duration", DEFAULT_MOVE_S))
                return max(MIN_MOVE_S, min(MAX_MOVE_S, val))
            except Exception:
                return DEFAULT_MOVE_S

        def _coerce_confidence(data: dict) -> float:
            try:
                val = float(data.get("confidence", 0.0))
                return max(0.0, min(1.0, val))
            except Exception:
                return 0.0

        def _bool(data: dict, key: str) -> bool:
            return bool(data.get(key, False))

        # ===== STAGE 1: PERCEIVER =====
        perceiver_prompt = (
            "You are the Perception agent for a Unitree Go2 robot.\n"
            "Analyze the camera image and sensor data. Generate a detailed perception report.\n"
            "Return ONLY ONE JSON object (no markdown, no code fences).\n"
            "JSON keys:\n"
            "  scene_description: detailed description of what you see in the image\n"
            "  obstacles_detected: list of obstacles and their approximate positions\n"
            "  target_visible: boolean (is the goal/target visible?)\n"
            "  recommended_action: one of [forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop]\n"
            "  reasoning: explanation of perception and recommended action\n"
            "  safety_concerns: list of any safety concerns\n"
        )

        perceiver_user = (
            f"Task: {self.task}\n"
            f"Environmental scan (360° scout):\n{scout_text}\n"
            f"Recent actions: {self.history[-3:]}\n"
            "\n"
            "Analyze the current scene and provide detailed perception based on the scout data."
        )

        perceiver_data, perceiver_err = self._agent_request("perceiver", perceiver_prompt, perceiver_user, b64, max_tokens=1000)
        if perceiver_data is None:
            return "stop", "stop", DEFAULT_MOVE_S, f"Perception error: {perceiver_err}", False, False, 0.0, "high", "fallback"

        if self.debug:
            perception = perceiver_data.get("scene_description", "")[:100]
            print(f"[DBG] Perceiver -> scene: {perception}... | safety: {perceiver_data.get('safety_concerns', [])}")

        # ===== STAGE 2: REVIEWER =====
        reviewer_prompt = (
            "You are the Reviewer for a Unitree Go2 robot decision pipeline.\n"
            "You review the perception and propose a safe, task-aligned action.\n"
            "Return ONLY ONE JSON object.\n"
            "JSON keys:\n"
            "  approve_perception: boolean (do you agree with the perception?)\n"
            "  action: one of [forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop]\n"
            "  duration: seconds 0.2 to 3.0\n"
            "  explanation: why this action\n"
            "  confidence: 0.0 to 1.0\n"
            "  risk: one of [low, medium, high]\n"
            "  needs_reflection: boolean (should reflection double-check this?)\n"
            "Rules:\n"
            "- Safety is priority #1. If lidar distance is very small, choose stop.\n"
            "- Use perception to decide the best next action.\n"
            "- Be conservative with confidence.\n"
        )

        reviewer_user = (
            f"Task: {self.task}\n"
            f"Environmental scan: {scout_text}\n"
            f"Perception report:\n"
            f"  Scene: {perceiver_data.get('scene_description', '')}\n"
            f"  Target visible: {perceiver_data.get('target_visible', False)}\n"
            f"  Safety concerns: {perceiver_data.get('safety_concerns', [])}\n"
            f"  Recommended action: {perceiver_data.get('recommended_action', 'stop')}\n"
            "\n"
            "Review and propose the next action."
        )

        reviewer_data, reviewer_err = self._agent_request("reviewer", reviewer_prompt, reviewer_user, b64, max_tokens=600)
        if reviewer_data is None:
            return "stop", "stop", DEFAULT_MOVE_S, f"Reviewer error: {reviewer_err}", False, False, 0.0, "high", "fallback"

        action = _coerce_action(reviewer_data)
        duration = _coerce_duration(reviewer_data)
        explanation = str(reviewer_data.get("explanation", "")).strip()
        confidence = _coerce_confidence(reviewer_data)
        risk = _normalize_risk(reviewer_data.get("risk", "low"))
        needs_reflection = _bool(reviewer_data, "needs_reflection")
        source = "reviewer"

        if self.debug:
            print(f"[DBG] Reviewer -> action={action} duration={duration:.2f}s conf={confidence:.2f} risk={risk} reflect={needs_reflection}")

        # ===== STAGE 3: REFLECTION (if needed) =====
        need_reflect = needs_reflection or (confidence < AGENT_CONFIDENCE_REFLECT) or (risk == "high")
        if AGENT_REFLECTION_ENABLED and need_reflect:
            reflection_prompt = (
                "You are the Reflection agent. Double-check the proposed action for safety.\n"
                "Return ONLY ONE JSON object.\n"
                "JSON keys: action, duration, explanation, confidence, risk.\n"
                "Rules:\n"
                "- If ANY risk, prefer stop. Safety > progress.\n"
                "- If lidar distance is small, must stop.\n"
            )

            reflection_user = (
                f"Task: {self.task}\n"
                f"Environmental scan: {scout_text}\n"
                f"Proposed action: {action} for {duration:.2f}s\n"
                f"Reason: {explanation}\n"
                f"Confidence: {confidence:.2f}, Risk: {risk}\n"
                f"Safety concerns from perception: {perceiver_data.get('safety_concerns', [])}\n"
                "\n"
                "Reflect on safety. Return the safest action."
            )

            reflection_data, reflection_err = self._agent_request("reflection", reflection_prompt, reflection_user, b64, max_tokens=500)
            if reflection_data is not None:
                refl_action = _coerce_action(reflection_data)
                refl_duration = _coerce_duration(reflection_data)
                refl_expl = str(reflection_data.get("explanation", "")).strip()
                refl_conf = _coerce_confidence(reflection_data)
                refl_risk = _normalize_risk(reflection_data.get("risk", "low"))

                def _risk_score(r: str) -> int:
                    return {"low": 0, "medium": 1, "high": 2}.get(r, 0)

                if self.debug:
                    print(f"[DBG] Reflection -> action={refl_action} duration={refl_duration:.2f}s conf={refl_conf:.2f} risk={refl_risk}")

                # Use reflection if it's safer
                if (_risk_score(refl_risk) < _risk_score(risk)) or (refl_conf > confidence + 0.1):
                    action = refl_action
                    duration = refl_duration
                    explanation = refl_expl or explanation
                    confidence = refl_conf
                    risk = refl_risk
                    source = "reflection"

        # Assume subtask done if action != stop, task done if model explicitly says so
        subtask_done = (action != "stop")
        task_done = False  # Conservative - only set true if model explicitly says

        mapped_key = ACTION_TO_KEY[action]

        if self.debug:
            print(f"[DBG] Final -> action={action} duration={duration:.2f}s mapped_key={mapped_key} "
                  f"conf={confidence:.2f} risk={risk} src={source} expl={explanation!r}")

        return action, mapped_key, duration, explanation, subtask_done, task_done, confidence, risk, source


# ---------------- Main Loop ----------------
def main():
    rclpy.init()
    node = Go2Agent()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    executor = ThreadPoolExecutor(max_workers=1)
    pending = None  # future

    ai_state = "idle"  # idle -> moving -> settling
    scout_state = "not_scouting"  # not_scouting -> rotating -> analyzing -> backtracking -> done
    scout_segments = 12  # 360 / 30 = 12 segments
    scout_segment = 0
    backtrack_segment = 0
    scout_rotate_until = 0.0
    
    move_until = 0.0
    settle_until = 0.0
    last_ai = None  # (action, key, duration, explanation, subtask_done, task_done, confidence, risk, source)

    print("\n" + "=" * 90)
    print(" GO2 CONTROLLER (MANUAL + LLM) - Semantic actions + step counter")
    print(f" cmd_topic: {CMD_TOPIC}")
    print(f" cam_topic: {CAM_TOPIC}")
    print("")
    print(" Keys:")
    print("   m      = toggle MANUAL/AI")
    print("   t      = set task for LLM")
    print("   SPACE  = toggle E-STOP (stop but do NOT exit)")
    print("   q      = quit")
    print("")
    print(" Manual drive keys: i/k/j/l/J/L")
    print("=" * 90 + "\n")

    try:
        while True:
            now = time.monotonic()

            # --- Keyboard ---
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                key = sys.stdin.read(1)

                if key == 'q':
                    break

                if key == ' ':
                    node.estop = not node.estop
                    node.stop_now(reason="E-STOP toggle")
                    print(f"[E-STOP] {'ENGAGED' if node.estop else 'RELEASED'}")
                    pending = None
                    ai_state = "idle"
                    continue

                if key == 'm':
                    node.manual_mode = not node.manual_mode
                    node.stop_now(reason="mode toggle")
                    print(f"[MODE] {'MANUAL' if node.manual_mode else 'AI'} (estop={node.estop})")
                    pending = None
                    ai_state = "idle"
                    continue

                if key == 't':
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    node.stop_now(reason="set task")
                    node.task = input("\nEnter Task (e.g., 'Find the door', 'Move forward 2 meters'): ").strip() or "None"
                    node.history = []
                    node.subtask_count = 0

                    print(f"[TASK] New mission: {node.task} | VLM will determine steps dynamically")
                    tty.setraw(fd)

                    pending = None
                    ai_state = "idle"
                    continue

                # Manual control
                if node.manual_mode and not node.estop:
                    if key in MOVE_MAP:
                        node.set_cmd(key, reason="manual key")

            # --- AI Mode with Scout Phase ---
            if (not node.manual_mode) and (not node.estop):
                # ===== SCOUT PHASE =====
                if scout_state == "not_scouting":
                    # Start scout rotation
                    scout_state = "rotating"
                    scout_segment = 0
                    backtrack_segment = 0
                    node.scout_observations = []
                    node.scout_locations = {}
                    node.scout_targets_found = []
                    node.scout_angle_current = 0
                    node.stop_now(reason="scout start")
                    if node.debug:
                        print("[SCOUT] Starting 360-degree environment scan with location memory...")
                
                if scout_state == "rotating":
                    # Rotate and collect observations every 30 degrees
                    node.set_cmd("l", reason="scout rotate right")  # Rotate right
                    scout_state = "analyzing"
                    scout_rotate_until = time.monotonic() + SCOUT_ROTATION_S
                
                if scout_state == "analyzing" and time.monotonic() >= scout_rotate_until:
                    # Stop rotation and analyze current view
                    node.stop_now(reason="scout analyze")
                    node.scout_angle_current = scout_segment * SCOUT_ROTATION_DEGREES
                    
                    # Capture observation asynchronously
                    if pending is None:
                        pending = executor.submit(node.scout_analyze)
                    elif pending.done():
                        obs = pending.result()
                        pending = None
                        node.scout_observations.append(obs)
                        if node.debug:
                            print(f"[SCOUT] {obs}")
                        
                        scout_segment += 1
                        if scout_segment < scout_segments:
                            scout_state = "rotating"
                        else:
                            # Scout complete, now backtrack
                            scout_state = "backtracking"
                            backtrack_segment = 0
                            node.stop_now(reason="scout backtrack start")
                            if node.debug:
                                print("[SCOUT] Rotation complete. Backtracking to original position...")
                
                if scout_state == "backtracking":
                    # Rotate left to backtrack (opposite of right rotation)
                    node.set_cmd("j", reason="scout backtrack rotate left")
                    scout_rotate_until = time.monotonic() + SCOUT_ROTATION_S
                    scout_state = "backtrack_waiting"
                
                if scout_state == "backtrack_waiting" and time.monotonic() >= scout_rotate_until:
                    node.stop_now(reason="scout backtrack segment")
                    backtrack_segment += 1
                    if backtrack_segment < scout_segments:
                        scout_state = "backtracking"
                    else:
                        scout_state = "done"
                        node.stop_now(reason="scout complete")
                        if node.debug:
                            print("[SCOUT] Backtrack complete!")
                            print(f"[SCOUT] Location memory: {len(node.scout_locations)} positions mapped")
                            if node.scout_targets_found:
                                print(f"[SCOUT] TARGETS FOUND ({len(node.scout_targets_found)}): ")
                                for target in node.scout_targets_found:
                                    print(f"  - {target['name']} at {target['angle']}° (~{target['distance']}m) - {target['position_desc']}")
                            print("[SCOUT] Starting task execution with location memory...")
                        ai_state = "idle"

                # ===== TASK EXECUTION PHASE =====
                if scout_state == "done":
                    # IDLE: Launch LLM request
                    if ai_state == "idle":
                        if pending is None:
                            # If no task was set, use exploration as default
                            if node.task == "None":
                                node.task = "Explore the environment and find interesting targets"
                                print(f"[TASK] No task set. Using default: {node.task}")
                            
                            node.stop_now(reason="thinking")
                            pending = executor.submit(node.get_ai_decision)

                        # When LLM finishes, start pulse
                        if pending is not None and pending.done():
                            try:
                                action, mapped_key, duration, expl, subtask_done, task_done, conf, risk, source = pending.result()
                            except Exception as e:
                                print(f"[AI ERROR] get_ai_decision failed: {e}")
                                action, mapped_key, duration, expl, subtask_done, task_done, conf, risk, source = "stop", "stop", 0.5, f"Decision error: {e}", False, False, 0.0, "high", "error"
                            finally:
                                pending = None
                            last_ai = (action, mapped_key, duration, expl, subtask_done, task_done, conf, risk, source)

                            node.history.append(f"{action}({duration:.2f}s) | {expl} | conf={conf:.2f} risk={risk} src={source}")
                            print(f"[AI] action={action} duration={duration:.2f}s key={mapped_key} | {expl} | "
                                  f"subtask_done={subtask_done} task_done={task_done} conf={conf:.2f} risk={risk} src={source}")

                            if mapped_key != "stop":
                                node.subtask_count += 1

                            node.set_cmd(mapped_key, reason="AI pulse start")
                            ai_state = "moving"
                            move_until = time.monotonic() + duration

                    # MOVING: End pulse
                    elif ai_state == "moving" and time.monotonic() >= move_until:
                        node.set_cmd("stop", reason="AI pulse end")
                        ai_state = "settling"
                        settle_until = time.monotonic() + DEFAULT_SETTLE_S

                    # SETTLING: End settle
                    elif ai_state == "settling" and time.monotonic() >= settle_until:
                        ai_state = "idle"

                        # If model said task_done, switch back to manual
                        if last_ai:
                            _, _, _, _, _, task_done, _, _, _ = last_ai
                            if task_done:
                                node.manual_mode = True
                                node.stop_now(reason="task completed")
                                print("[AI] task_done=true -> switching to MANUAL (task complete!)")
                                last_ai = None
                                scout_state = "not_scouting"  # Reset scout for next task

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.stop_now(reason="shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()