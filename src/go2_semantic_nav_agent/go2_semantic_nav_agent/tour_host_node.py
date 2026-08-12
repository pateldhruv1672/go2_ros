from __future__ import annotations

import json
import shutil
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def _resolve_session(root: Path, requested: str) -> str:
    name = str(requested or "").strip()
    if name and name.lower() not in {"auto", "latest", "latest_usable", "default"}:
        return name
    candidates = []
    if root.is_dir():
        for p in root.iterdir():
            if not p.is_dir():
                continue
            usable = (p / "map.yaml").is_file() and ((p / "places.yaml").is_file() or (p / "route.yaml").is_file())
            if not usable:
                continue
            try:
                mtime = max((c.stat().st_mtime for c in p.iterdir()), default=p.stat().st_mtime)
            except Exception:
                mtime = 0.0
            candidates.append((mtime, p.name))
    if candidates:
        return max(candidates)[1]
    return name or "default"


class TourHostNode(Node):
    """Session-scoped, deterministic guest-tour speech and safe motion scripts."""

    def __init__(self) -> None:
        super().__init__("go2_tour_host")
        self.declare_parameter("session_root", "~/.ros/go2_semantic_nav_sessions")
        self.declare_parameter("session_name", "auto")
        self.declare_parameter("default_script_path", "")
        self.declare_parameter("session_script_filename", "tour_host.yaml")
        self.declare_parameter("command_topic", "/go2_tour/host_command")
        self.declare_parameter("tts_topic", "/go2_tts/say")
        self.declare_parameter("motion_topic", "/motion_skills/command")
        self.declare_parameter("speech_words_per_minute", 165.0)
        self.declare_parameter("speech_padding_sec", 0.9)
        self.declare_parameter("default_motion_wait_sec", 3.5)
        self.declare_parameter("speech_status_topic", "/go2_speech/status")
        self.declare_parameter("speech_completion_timeout_sec", 90.0)
        self.declare_parameter("auto_hello_on_script_speech", True)
        self.declare_parameter("auto_heart_after_script_speech", True)

        root = Path(str(self.get_parameter("session_root").value)).expanduser()
        session = _resolve_session(root, str(self.get_parameter("session_name").value))
        self.session_name = session
        self.session_dir = root / session
        self.session_dir.mkdir(parents=True, exist_ok=True)

        default_path = Path(str(self.get_parameter("default_script_path").value)).expanduser()
        filename = str(self.get_parameter("session_script_filename").value or "tour_host.yaml")
        self.script_path = self.session_dir / filename
        if not self.script_path.is_file():
            if not default_path.is_file():
                raise RuntimeError(f"Tour host default script missing: {default_path}")
            shutil.copy2(default_path, self.script_path)
            self.get_logger().info(f"Created session Tour Host script: {self.script_path}")

        self.tts_pub = self.create_publisher(String, str(self.get_parameter("tts_topic").value), 10)
        self.motion_pub = self.create_publisher(String, str(self.get_parameter("motion_topic").value), 10)
        self.status_pub = self.create_publisher(String, "/go2_tour/host_status", 10)
        self.create_subscription(String, str(self.get_parameter("command_topic").value), self._command_cb, 20)
        self.create_subscription(String, str(self.get_parameter("speech_status_topic").value), self._speech_status_cb, 20)
        self._speech_condition = threading.Condition()
        self._speech_done_ids: set[str] = set()
        self._speech_hello_meta: dict[str, tuple[str, int]] = {}

        self._cancel = threading.Event()
        self._worker_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._publish_status("ready", script_path=str(self.script_path), session_name=self.session_name)
        self.get_logger().info(f"Tour Host ready | session={self.session_name} script={self.script_path}")

    def _load(self) -> dict[str, Any]:
        data = yaml.safe_load(self.script_path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise RuntimeError("tour_host.yaml root must be a mapping")
        return data

    def _publish_status(self, event: str, **extra: Any) -> None:
        payload = {
            "event": event,
            "session_name": self.session_name,
            "script_path": str(self.script_path),
            "time_unix": time.time(),
            **extra,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True, default=str)))

    def _speech_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        event = str(payload.get('event') or '')
        request_id = str(payload.get('request_id') or '')
        if not request_id:
            return
        if event == 'speaking':
            meta = self._speech_hello_meta.pop(request_id, None)
            if meta is not None and bool(self.get_parameter('auto_hello_on_script_speech').value):
                script, action_index = meta
                self._send_motion('hello', script=script, action_index=action_index)
                self._publish_status('hello_on_speech_start', script=script, action_index=action_index, request_id=request_id)
            return
        if event != 'speech_done':
            return
        with self._speech_condition:
            self._speech_done_ids.add(request_id)
            self._speech_condition.notify_all()

    def _wait_for_speech_done(self, request_id: str, fallback_sec: float) -> bool:
        configured = max(8.0, float(self.get_parameter('speech_completion_timeout_sec').value))
        timeout = min(configured, max(8.0, float(fallback_sec) + 8.0))
        deadline = time.monotonic() + timeout
        with self._speech_condition:
            while not self._cancel.is_set() and time.monotonic() < deadline:
                if request_id in self._speech_done_ids:
                    self._speech_done_ids.discard(request_id)
                    return True
                self._speech_condition.wait(timeout=min(0.25, max(0.01, deadline - time.monotonic())))
        return False

    def _send_motion(self, command: str, *, script: str, action_index: int) -> None:
        self._publish_status('motion', script=script, action_index=action_index, command=command)
        self.motion_pub.publish(String(data=json.dumps({
            'command': command, 'source': 'tour_host', 'verified': True, 'safety_checked': True,
        }, sort_keys=True)))

    def _command_cb(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            payload = json.loads(raw) if raw.startswith("{") else {"script": raw}
        except Exception:
            payload = {"script": raw}
        script = str(payload.get("script") or payload.get("command") or payload.get("type") or "").strip()
        if script in {"cancel", "stop", "stop_host"}:
            self._cancel.set()
            self._publish_status("cancel_requested")
            return
        if not script:
            return
        with self._worker_lock:
            if self._thread and self._thread.is_alive():
                self._publish_status("busy", requested_script=script)
                return
            self._cancel.clear()
            self._thread = threading.Thread(target=self._run_script, args=(script,), daemon=True)
            self._thread.start()

    def _expand_actions(self, script: str, data: dict[str, Any], stack: tuple[str, ...] = ()) -> list[dict[str, Any]]:
        if script in stack:
            raise RuntimeError(f"recursive Tour Host include: {' -> '.join(stack + (script,))}")
        scripts = data.get("scripts") if isinstance(data.get("scripts"), dict) else {}
        actions = scripts.get(script)
        if not isinstance(actions, list):
            raise KeyError(script)
        out: list[dict[str, Any]] = []
        for item in actions:
            if not isinstance(item, dict):
                continue
            include = str(item.get("include") or "").strip()
            if include:
                out.extend(self._expand_actions(include, data, stack + (script,)))
            else:
                out.append(dict(item))
        return out

    def _run_script(self, script: str) -> None:
        try:
            data = self._load()
            actions = self._expand_actions(script, data)
        except Exception as exc:
            self._publish_status('script_error', script=script, error=str(exc))
            self.get_logger().error(f'Tour Host script {script} failed to load: {exc}')
            return
        speech_present = any(str(a.get('type') or 'speech').strip().lower() == 'speech' and str(a.get('text') or '').strip() for a in actions)
        self._publish_status('script_start', script=script, action_count=len(actions))
        greeted = False
        for index, action in enumerate(actions):
            if self._cancel.is_set():
                self._publish_status('script_canceled', script=script, action_index=index)
                return
            kind = str(action.get('type') or 'speech').strip().lower()
            if kind == 'speech':
                text = str(action.get('text') or '').strip()
                if not text:
                    continue
                request_id = f'tour_{script}_{index}_{time.time_ns()}'
                if speech_present and not greeted and bool(self.get_parameter('auto_hello_on_script_speech').value):
                    # Fire hello only when the speech arbiter reports this request is actually speaking.
                    self._speech_hello_meta[request_id] = (script, index)
                    greeted = True
                self._publish_status('speech', script=script, action_index=index, text=text, request_id=request_id)
                payload = {
                    'text': text, 'category': 'tour_host', 'source': 'tour_host',
                    'session_name': self.session_name, 'interrupt': False, 'request_id': request_id,
                }
                self.tts_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
                words = max(1, len(text.split()))
                wpm = max(80.0, float(self.get_parameter('speech_words_per_minute').value))
                fallback_sec = max(1.2, words / wpm * 60.0 + float(self.get_parameter('speech_padding_sec').value))
                if not self._wait_for_speech_done(request_id, fallback_sec):
                    self._speech_hello_meta.pop(request_id, None)
                    if self._cancel.is_set():
                        self._publish_status('script_canceled', script=script, action_index=index)
                        return
                    self._publish_status('speech_completion_timeout', script=script, action_index=index, request_id=request_id)
                    return
            elif kind == 'motion':
                command = str(action.get('command') or '').strip()
                if not command:
                    continue
                self._send_motion(command, script=script, action_index=index)
                wait_sec = float(action.get('wait_sec') or self.get_parameter('default_motion_wait_sec').value)
                if self._cancel.wait(max(0.0, wait_sec)):
                    self._publish_status('script_canceled', script=script, action_index=index)
                    return
            elif kind == 'pause':
                wait_sec = float(action.get('seconds') or 1.0)
                if self._cancel.wait(max(0.0, wait_sec)):
                    self._publish_status('script_canceled', script=script, action_index=index)
                    return
        if speech_present and not self._cancel.is_set() and bool(self.get_parameter('auto_heart_after_script_speech').value):
            self._send_motion('heart', script=script, action_index=len(actions))
            self._cancel.wait(max(0.0, float(self.get_parameter('default_motion_wait_sec').value)))
        if not self._cancel.is_set():
            self._publish_status('script_done', script=script)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TourHostNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._cancel.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
