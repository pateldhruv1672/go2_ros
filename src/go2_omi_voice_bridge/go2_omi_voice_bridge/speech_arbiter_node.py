from __future__ import annotations

import json
import re
import time
from collections import deque
from typing import Any, Deque, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _text(raw: str) -> tuple[str, Dict[str, Any]]:
    raw = str(raw or '').strip()
    if not raw:
        return '', {}
    try:
        p = json.loads(raw)
    except Exception:
        return raw, {'text': raw}
    if not isinstance(p, dict):
        return raw, {'text': raw}
    for key in ('text', 'speech', 'narration', 'response', 'message', 'summary', 'data'):
        v = p.get(key)
        if isinstance(v, str) and v.strip():
            return v.strip(), p
    return '', p


def _norm(text: str) -> str:
    return re.sub(r'[^a-z0-9]+', ' ', text.lower()).strip()


class SpeechArbiter(Node):
    def __init__(self) -> None:
        super().__init__('go2_speech_arbiter')
        self.declare_parameter('dedupe_window_sec', 6.0)
        self.declare_parameter('fallback_words_per_minute', 165)
        self.declare_parameter('max_queue', 20)
        self.out = self.create_publisher(String, '/go2_tts/say', 10)
        self.status = self.create_publisher(String, '/go2_speech/status', 10)
        self.queue: Deque[Dict[str, Any]] = deque()
        self.recent: Dict[str, float] = {}
        self.current: Dict[str, Any] | None = None
        self.deadline = 0.0
        for topic in ('/go2_speech/request', '/go2_tour/narration', '/go2_agent/speech', '/agent/reply', '/go2_vlm/query_result'):
            self.create_subscription(String, topic, lambda m, t=topic: self._enqueue(m, t), 10)
        self.create_subscription(String, '/go2_tts/status', self._tts_status, 10)
        self.create_timer(0.10, self._tick)
        self.get_logger().info('Single speech arbiter ready: dedupe + serialized TTS queue -> /go2_tts/say')

    def _enqueue(self, msg: String, source: str) -> None:
        text, payload = _text(msg.data)
        if not text:
            return
        now = time.monotonic()
        key = _norm(text)
        window = float(self.get_parameter('dedupe_window_sec').value)
        if key and now - self.recent.get(key, -1e9) < window:
            self._publish('duplicate_dropped', source=source, text=text[:180], request_id=payload.get('request_id'))
            return
        self.recent[key] = now
        priority = str(payload.get('priority') or 'normal').lower()
        item = {'text': text, 'source': source, 'priority': priority, 'category': payload.get('category', 'speech'), 'request_id': payload.get('request_id')}
        if priority in {'urgent', 'high'}:
            self.queue.clear()
            item['interrupt'] = True
            self.queue.appendleft(item)
            self.current = None
            self.deadline = 0.0
        else:
            if len(self.queue) >= int(self.get_parameter('max_queue').value):
                self._publish('queue_full_drop', source=source)
                return
            self.queue.append(item)
        self._publish('queued', source=source, queued=len(self.queue), text=text[:180])

    def _tts_status(self, msg: String) -> None:
        try:
            p = json.loads(msg.data)
        except Exception:
            return
        if isinstance(p, dict) and str(p.get('event') or '') in {'local_speaker_done', 'speech_done'}:
            done_request_id = (self.current or {}).get('request_id')
            self.current = None
            self.deadline = 0.0
            self._publish('speech_done', queued=len(self.queue), request_id=done_request_id)

    def _tick(self) -> None:
        now = time.monotonic()
        if self.current is not None and now >= self.deadline:
            self.current = None
            self.deadline = 0.0
            self._publish('speech_timeout_release', queued=len(self.queue))
        if self.current is not None or not self.queue:
            return
        item = self.queue.popleft()
        words = max(1, len(str(item['text']).split()))
        wpm = max(80, int(self.get_parameter('fallback_words_per_minute').value))
        self.deadline = now + max(3.0, words / wpm * 60.0 + 3.0)
        self.current = item
        self.out.publish(String(data=json.dumps(item, sort_keys=True)))
        self._publish('speaking', source=item.get('source'), text=item.get('text', '')[:240], queued=len(self.queue), request_id=item.get('request_id'))

    def _publish(self, event: str, **extra: Any) -> None:
        self.status.publish(String(data=json.dumps({'event': event, 'stamp_sec': time.time(), **extra}, sort_keys=True, default=str)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
