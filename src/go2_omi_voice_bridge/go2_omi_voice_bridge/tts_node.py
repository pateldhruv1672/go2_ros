from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _message_text(data: str) -> tuple[str, dict[str, Any]]:
    raw = data.strip()
    if not raw:
        return "", {}

    try:
        payload = json.loads(raw)
        if isinstance(payload, dict):
            if payload.get("vlm_success") is False:
                text = str(
                    payload.get("vlm_error")
                    or "I could not get a camera summary from the VLM."
                ).strip()
                return text, payload

            for key in (
                "text",
                "message",
                "summary",
                "speech",
                "narration",
                "response",
                "data",
            ):
                value = payload.get(key)
                if value is not None:
                    text = str(value).strip()
                    if text:
                        return text, payload
            return "", payload
    except Exception:
        pass

    return raw, {"text": raw}


def _plain_spoken_text(text: str, max_sentences: int = 0) -> str:
    clean = str(text or "")
    if not clean.strip():
        return ""

    clean = re.sub(r"```(?:\w+)?\s*(.*?)```", r"\1", clean, flags=re.DOTALL)
    clean = re.sub(r"`([^`]*)`", r"\1", clean)
    clean = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", clean)
    clean = re.sub(r"!\[([^\]]*)\]\([^)]+\)", r"\1", clean)
    clean = re.sub(r"(?m)^\s{0,3}#{1,6}\s*", "", clean)
    clean = re.sub(r"(?m)^\s*[-*+]\s+", "", clean)
    clean = re.sub(r"(?m)^\s*\d+[.)]\s+", "", clean)
    clean = re.sub(r"[*_~>#]+", "", clean)
    clean = re.sub(r"\s+", " ", clean).strip()

    if max_sentences > 0:
        sentences = re.findall(r"[^.!?]+[.!?]+|[^.!?]+$", clean)
        clean = " ".join(sentence.strip() for sentence in sentences[:max_sentences]).strip()
    return clean


class Go2TtsNode(Node):
    """Interruptible local speaker mirror for Go2/Omi speech topics.

    This node does not bypass Nav2 or motion safety. It only mirrors speech text to
    the computer's default audio output. It also keeps publishing /go2_tts/status
    for debugging and UI feedback.
    """

    def __init__(self) -> None:
        super().__init__("go2_tts_node")

        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_backend", "local")
        self.declare_parameter("espeak_voice", "en")

        # New computer speaker parameters.
        self.declare_parameter("local_speaker_enabled", True)
        self.declare_parameter("local_speaker_backend", "auto")
        self.declare_parameter("local_speaker_voice", "")
        self.declare_parameter("local_speaker_rate", 170)
        self.declare_parameter("local_speaker_volume", 100)
        self.declare_parameter("local_speaker_timeout_sec", 60.0)
        self.declare_parameter("computer_speaker_device", "")
        self.declare_parameter("interrupt_previous_by_default", False)
        self.declare_parameter("speak_vlm_status", True)
        self.declare_parameter("max_spoken_sentences", 2)
        self.declare_parameter("input_topics", ["/go2_tts/say"])
        self.declare_parameter("duplicate_window_sec", 1.5)

        self.status_pub = self.create_publisher(String, "/go2_tts/status", 10)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen[str] | None = None
        self._recent_speech: dict[str, float] = {}

        configured_topics = self.get_parameter("input_topics").value
        if isinstance(configured_topics, str):
            topics = (configured_topics,)
        else:
            topics = tuple(str(x) for x in (configured_topics or ["/go2_tts/say"]))
        for topic in topics:
            self.create_subscription(
                String,
                topic,
                lambda msg, topic_name=topic: self._on_speech(msg, topic_name),
                10,
            )

        backend = self._select_backend()
        self.get_logger().info(
            "Go2 TTS node ready; mirroring speech to computer speaker. "
            f"topics={list(topics)} backend={backend or 'none'}"
        )
        if backend is None and self._param_bool("local_speaker_enabled"):
            self.get_logger().warn(
                "No local TTS executable found. Install espeak-ng with: "
                "sudo apt-get update && sudo apt-get install -y espeak-ng"
            )

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _param_int(self, name: str, default: int) -> int:
        try:
            return int(self.get_parameter(name).value)
        except Exception:
            return default

    def _param_float(self, name: str, default: float) -> float:
        try:
            return float(self.get_parameter(name).value)
        except Exception:
            return default

    def _on_speech(self, msg: String, topic_name: str) -> None:
        if topic_name == "/go2_vlm_checkpoint/status" and not self._param_bool(
            "speak_vlm_status"
        ):
            return

        text, payload = _message_text(msg.data)
        if not text:
            return
        text = _plain_spoken_text(text, self._param_int("max_spoken_sentences", 2))
        if not text:
            return

        # Final duplicate guard: the speech arbiter should be the normal producer,
        # but identical publications inside this window must not overlap audio.
        dedup_key = re.sub(r"\s+", " ", text).strip().lower()
        now_mono = time.monotonic()
        window = max(0.0, self._param_float("duplicate_window_sec", 1.5))
        last = self._recent_speech.get(dedup_key, -1e9)
        if window > 0.0 and now_mono - last < window:
            self.status_pub.publish(String(data=json.dumps({
                "ok": True, "event": "duplicate_dropped", "source_topic": topic_name,
                "text_len": len(text),
            }, sort_keys=True)))
            return
        self._recent_speech[dedup_key] = now_mono
        self._recent_speech = {
            k: v for k, v in self._recent_speech.items()
            if now_mono - v < max(10.0, 4.0 * window)
        }

        interrupt = (
            bool(payload.get("interrupt", False))
            or payload.get("priority") in {"high", "urgent"}
            or self._param_bool("interrupt_previous_by_default")
        )
        if interrupt:
            self._stop_current()

        backend = self._select_backend()
        status = {
            "ok": True,
            "event": "speech_queued",
            "source_topic": topic_name,
            "text_len": len(text),
            "category": payload.get("category", "speech"),
            "tts_backend": str(self.get_parameter("tts_backend").value),
            "local_speaker_enabled": self._param_bool("local_speaker_enabled"),
            "local_speaker_backend": backend or "none",
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        self.get_logger().info(f"TTS mirror from {topic_name}: {text}")

        if self._param_bool("tts_enabled") and self._param_bool(
            "local_speaker_enabled"
        ):
            threading.Thread(target=self._speak_local, args=(text,), daemon=True).start()

    def _select_backend(self) -> str | None:
        requested = str(self.get_parameter("local_speaker_backend").value).strip()
        requested_l = requested.lower()

        if requested_l in {"off", "disabled", "none", "false", "0"}:
            return None

        if requested_l and requested_l != "auto":
            exe = shutil.which(requested)
            if exe:
                return exe
            # Friendly aliases.
            aliases = {
                "espeak": ("espeak-ng", "espeak"),
                "espeak-ng": ("espeak-ng",),
                "spd-say": ("spd-say",),
                "say": ("say",),
            }
            for candidate in aliases.get(requested_l, (requested,)):
                exe = shutil.which(candidate)
                if exe:
                    return exe
            return None

        for candidate in ("espeak-ng", "espeak", "spd-say", "say"):
            exe = shutil.which(candidate)
            if exe:
                return exe
        return None

    def _build_command(self, exe: str, text: str) -> list[str]:
        name = os.path.basename(exe)
        voice = str(self.get_parameter("local_speaker_voice").value).strip()
        if not voice:
            voice = str(self.get_parameter("espeak_voice").value).strip() or "en"
        rate = str(self._param_int("local_speaker_rate", 170))
        volume = str(self._param_int("local_speaker_volume", 100))

        if name in {"espeak", "espeak-ng"}:
            return [exe, "-v", voice, "-s", rate, "-a", volume, text]
        if name == "spd-say":
            return [exe, "-w", text]
        if name == "say":
            return [exe, text]
        return [exe, text]

    def _stop_current(self) -> None:
        with self._lock:
            proc = self._proc
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
            self._proc = None

    def _speak_local(self, text: str) -> None:
        disabled_backend = str(self.get_parameter("tts_backend").value).strip().lower()
        if disabled_backend in {"off", "disabled", "none", "false", "0"}:
            return

        exe = self._select_backend()
        if exe is None:
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "ok": False,
                            "error": "no_local_tts_executable",
                            "hint": "sudo apt-get update && sudo apt-get install -y espeak-ng",
                        },
                        sort_keys=True,
                    )
                )
            )
            return

        clean_text = " ".join(text.split())
        if len(clean_text) > 4000:
            clean_text = clean_text[:4000]

        env = os.environ.copy()
        device = str(self.get_parameter("computer_speaker_device").value).strip()
        if device:
            env["AUDIODEV"] = device

        cmd = self._build_command(exe, clean_text)
        timeout = self._param_float("local_speaker_timeout_sec", 60.0)
        words = max(1, len(clean_text.split()))
        words_per_minute = max(80, self._param_int("local_speaker_rate", 170))
        estimated_duration_sec = max(2.0, (words / words_per_minute) * 60.0 + 1.0)

        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "ok": True,
                        "event": "local_speaker_start",
                        "text_len": len(clean_text),
                        "estimated_duration_sec": round(estimated_duration_sec, 2),
                        "local_speaker_backend": os.path.basename(exe),
                        "timeout_sec": timeout,
                    },
                    sort_keys=True,
                )
            )
        )

        with self._lock:
            self._proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            proc = self._proc

        ok = True
        error = ""
        try:
            proc.wait(timeout=timeout)
            ok = proc.returncode == 0
            if not ok:
                error = f"local_tts_returncode_{proc.returncode}"
        except subprocess.TimeoutExpired:
            ok = False
            error = "local_tts_timeout"
            proc.terminate()

        with self._lock:
            if self._proc is proc:
                self._proc = None

        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "ok": ok,
                        "event": "local_speaker_done",
                        "error": error,
                        "local_speaker_backend": os.path.basename(exe),
                    },
                    sort_keys=True,
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2TtsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_current()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
