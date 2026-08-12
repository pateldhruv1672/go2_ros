from __future__ import annotations

import json
import math
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.request
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, TypedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

from go2_langgraph_agent.tools.memory_tools import MemoryTools
from go2_langgraph_agent.tools.object_memory_tools import SmartObjectMemoryTools
from go2_langgraph_agent.tools.object_navigation_tools import ObjectNavigationTools


class VoiceState(TypedDict, total=False):
    text: str
    source: str
    context: Dict[str, Any]
    plan: Dict[str, Any]
    tool_result: Dict[str, Any]
    response: str


def _extract_text(raw: str) -> tuple[str, str]:
    raw = str(raw or '').strip()
    if not raw:
        return '', 'unknown'
    try:
        payload = json.loads(raw)
    except Exception:
        return raw, 'ros'
    if not isinstance(payload, dict):
        return raw, 'ros'
    text = ''
    for key in ('text', 'utterance', 'transcript', 'command', 'query'):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            text = value.strip()
            break
    return text or raw, str(payload.get('source') or payload.get('input_kind') or 'phone')


def _clean_wake(text: str) -> str:
    value = re.sub(r'^\s*sparky\s*[,.:;-]*\s*', '', str(text or ''), flags=re.I).strip()
    return value or str(text or '').strip()


def _is_yes(text: str) -> bool:
    t = _clean_wake(text).lower().strip(' .!?')
    return t in {'yes', 'yeah', 'yep', 'sure', 'okay', 'ok', 'confirm', 'do it', 'go ahead', 'please do'}


def _is_no(text: str) -> bool:
    t = _clean_wake(text).lower().strip(' .!?')
    return t in {'no', 'nope', 'cancel', 'never mind', 'nevermind', 'do not', "don't"}


def _emergency_stop(text: str) -> bool:
    # This is intentionally deterministic safety, not semantic behavior routing.
    t = _clean_wake(text).lower().strip(' .!?')
    return t in {'stop', 'emergency stop', 'e stop', 'estop', 'freeze', 'stop now'}


MOTION_ALIASES = {
    'wave': 'hello', 'say hello': 'hello', 'greet': 'hello', 'hello': 'hello',
    'stand': 'stand_up', 'stand up': 'stand_up', 'get up': 'stand_up',
    'sit down': 'sit', 'sit': 'sit', 'stretch': 'stretch',
    'heart': 'heart', 'finger heart': 'heart',
    'dance': 'dance1', 'dance one': 'dance1', 'dance two': 'dance2',
    'wiggle': 'wiggle_hips', 'wiggle hips': 'wiggle_hips',
    'tour greet': 'tour_greet', 'tour greeting': 'tour_greet',
    'handshake': 'tour_handshake', 'tour handshake': 'tour_handshake',
    'front flip': 'front_flip', 'frontflip': 'front_flip', 'do a front flip': 'front_flip',
    'tour attention': 'tour_attention', 'tour handoff': 'tour_handoff',
    'tour ack': 'tour_ack', 'tour settle': 'tour_settle',
}
MOTION_LOW_RISK = {
    'hello','stretch','sit','stand_up','stand_down','recovery_stand','heart',
    'tour_greet','tour_handshake','tour_attention','tour_handoff','tour_ack',
    'tour_settle','stand_ready','wait','balance_stand','stop_move',
}
MOTION_MEDIUM_RISK = {'dance1','dance2','scrape','wiggle_hips','wallow','free_walk','stand_out','standup'}
MOTION_HIGH_RISK = {'front_flip','front_jump','front_pounce','hand_stand','cross_step','bound','moon_walk','onesided_step','cross_walk'}

def _normalize_motion_skill(value: str) -> str:
    raw = re.sub(r'[^a-z0-9\s_-]', ' ', str(value or '').lower()).replace('_',' ').replace('-',' ')
    raw = ' '.join(raw.split())
    return MOTION_ALIASES.get(raw, raw.replace(' ', '_'))

def _motion_risk(skill: str) -> str:
    if skill in MOTION_HIGH_RISK: return 'high'
    if skill in MOTION_MEDIUM_RISK: return 'medium'
    return 'low'

def _bounded(value: Any, max_chars: int = 14000) -> Any:
    try:
        text = json.dumps(value, default=str)
    except Exception:
        return str(value)[:max_chars]
    if len(text) <= max_chars:
        return value
    return {'summary_truncated': text[:max_chars]}


class OllamaPlanner:
    def __init__(self, url: str, model: str, timeout_sec: float = 12.0) -> None:
        self.url = url
        self.model = model
        self.timeout_sec = float(timeout_sec)

    def call(self, system: str, user: str, *, temperature: float = 0.05, num_predict: int = 260) -> Dict[str, Any]:
        body = {
            'model': self.model,
            'stream': False,
            'format': 'json',
            'think': False,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': user},
            ],
            'keep_alive': '30m',
            'options': {'temperature': float(temperature), 'num_predict': int(num_predict), 'num_ctx': 8192, 'top_p': 0.9},
        }
        req = urllib.request.Request(
            self.url,
            data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                payload = json.loads(resp.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')[:1000]
            raise RuntimeError(f'Ollama HTTP {exc.code}: {detail}') from exc
        except Exception as exc:
            raise RuntimeError(f'Ollama request failed: {exc}') from exc
        content = ''
        if isinstance(payload, dict):
            content = str((payload.get('message') or {}).get('content') or payload.get('response') or '').strip()
        if not content:
            raise RuntimeError('Ollama returned no content')
        try:
            result = json.loads(content)
        except Exception:
            start, end = content.find('{'), content.rfind('}')
            if start < 0 or end <= start:
                raise RuntimeError(f'Ollama did not return JSON: {content[:500]}')
            result = json.loads(content[start:end + 1])
        if not isinstance(result, dict):
            raise RuntimeError('Ollama JSON result is not an object')
        return result


class AgenticVoiceActionNode(Node):
    """True guest-facing voice/action agent.

    Normal natural-language interpretation is performed by local Ollama inside a
    LangGraph StateGraph.  Hard-coded logic remains only for emergency stop,
    confirmation, and validating tool arguments before ROS execution.
    """

    TOOL_SCHEMA = [
        {'name': 'speak', 'args': {}, 'purpose': 'Natural conversation or grounded answer.'},
        {'name': 'live_vision_summary', 'args': {'question': 'live camera question'}, 'purpose': 'Fresh camera VLM analysis fused with live YOLO/SAM hints; never persistent memory.'},
        {'name': 'navigate_to_place', 'args': {'place': 'saved place/checkpoint name'}, 'purpose': 'Navigate with semantic_nav/Nav2.'},
        {'name': 'start_tour', 'args': {}, 'purpose': 'Start the saved route from the beginning.'},
        {'name': 'continue_tour', 'args': {}, 'purpose': 'Continue/resume the saved tour after a stop.'},
        {'name': 'pause_tour', 'args': {}, 'purpose': 'Pause the tour.'},
        {'name': 'save_tour_stop', 'args': {'name': 'checkpoint name', 'description': 'what Sparky should say', 'room': 'optional room', 'fact': 'optional factual summary'}, 'purpose': 'Save current map pose as a Tour stop.'},
        {'name': 'update_tour_stop', 'args': {'name': 'existing checkpoint', 'description': 'new narration', 'fact': 'optional fact'}, 'purpose': 'Update saved stop narration without changing pose.'},
        {'name': 'reorder_tour_stops', 'args': {'order': ['stop names']}, 'purpose': 'Change saved tour order.'},
        {'name': 'query_object_memory', 'args': {'label': 'object name', 'room': 'optional room'}, 'purpose': 'Grounded lookup of remembered physical objects and their map locations.'},
        {'name': 'find_object', 'args': {'label': 'object name', 'room': 'optional room'}, 'purpose': 'Retrieve a remembered object and navigate with Nav2 to a safe standoff near its stored physical object location.'},
        {'name': 'motion_skill', 'args': {'skill': 'hello|wave|stretch|sit|stand|heart|dance1|dance2|wiggle_hips|tour_greet|tour_handshake|tour_attention|tour_handoff|tour_ack|tour_settle|front_flip'}, 'purpose': 'Execute a known WebRTC motion skill. Use tour_greet for show-me-some-moves and tour_handshake for handshake-style greeting.'},
        {'name': 'stop', 'args': {}, 'purpose': 'Cancel Nav2/tour and stop motion.'},
    ]

    def __init__(self) -> None:
        super().__init__('go2_agentic_voice_action')
        self.declare_parameter('session_root', '~/.ros/go2_semantic_nav_sessions')
        self.declare_parameter('session_name', 'auto')
        self.declare_parameter('ollama_url', os.getenv('OLLAMA_CHAT_URL', 'http://127.0.0.1:11434/api/chat'))
        self.declare_parameter('ollama_model', os.getenv('GO2_AGENT_OLLAMA_MODEL', 'llama3.2:3b'))
        self.declare_parameter('ollama_timeout_sec', 12.0)
        self.declare_parameter('require_motion_skill_confirmation', True)
        self.declare_parameter('motion_confirmation_policy', 'risky_only')
        self.declare_parameter('history_turns', 10)
        self.declare_parameter('max_object_records', 80)
        self.declare_parameter('hybrid_fast_path', True)
        self.declare_parameter('live_vlm_query_topic', '/go2_vlm/query')
        self.declare_parameter('live_vlm_result_topic', '/go2_vlm/query_result')
        self.declare_parameter('live_vlm_timeout_sec', 35.0)
        self.declare_parameter('semantic_dispatch_ack_timeout_sec', 5.0)

        self.session_root = str(self.get_parameter('session_root').value)
        self.memory = MemoryTools(self.session_root, str(self.get_parameter('session_name').value))
        self.session_name = self.memory.session_name
        self.object_memory = SmartObjectMemoryTools(self.memory)
        self.object_nav = ObjectNavigationTools(self)
        self.latest_map = None
        self.latest_robot_pose: Dict[str, Any] = {}
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(String, str(self.get_parameter('live_vlm_result_topic').value), self._vlm_result_cb, 10)
        self.session_dir = Path(self.session_root).expanduser() / self.session_name
        self.history_path = self.session_dir / 'memory' / 'agentic_conversation.jsonl'
        self.history_path.parent.mkdir(parents=True, exist_ok=True)
        self.history = self._load_history()
        self.latest: Dict[str, Any] = {}
        self._vlm_condition = threading.Condition()
        self._vlm_results: Dict[str, Dict[str, Any]] = {}
        self._current_read_only = False
        self._current_text = ''
        self._active_request_id = ''
        self.pending_tool: Optional[Dict[str, Any]] = None

        self.semantic_pub = self.create_publisher(String, '/semantic_nav/command', 10)
        self.vlm_query_pub = self.create_publisher(String, str(self.get_parameter('live_vlm_query_topic').value), 10)
        self.motion_pub = self.create_publisher(String, '/motion_skills/command', 10)
        self.speech_pub = self.create_publisher(String, '/go2_speech/request', 10)
        self.status_pub = self.create_publisher(String, '/go2_agent/status', 10)
        self.event_pub = self.create_publisher(String, '/go2_agent/events', 10)
        self.state_pub = self.create_publisher(String, '/go2_agent/interaction_state', 10)

        # Exactly one normal guest transcript consumer.
        self.create_subscription(String, '/go2_voice/transcript', lambda m: self._enqueue(m, 'voice'), 10)
        self.create_subscription(String, '/go2_agent/query', lambda m: self._enqueue(m, 'query'), 10)

        for topic, key in (
            ('/semantic_nav/status', 'semantic_nav_status'),
            ('/semantic_nav/event', 'semantic_nav_event'),
            ('/go2_memory/object_inventory', 'object_inventory'),
            ('/go2_vln/object_map', 'live_object_map'),
            ('/go2_vlm_checkpoint/status', 'vlm_status'),
            ('/motion_skills/status', 'motion_status'),
            ('/go2_tts/status', 'tts_status'),
            ('/go2_speech/status', 'speech_status'),
        ):
            self.create_subscription(String, topic, lambda m, k=key: self._latest(k, m), 10)

        self.planner = OllamaPlanner(
            str(self.get_parameter('ollama_url').value),
            str(self.get_parameter('ollama_model').value),
            float(self.get_parameter('ollama_timeout_sec').value),
        )
        self.graph = self._build_graph()
        self.work_q: queue.Queue[tuple[str, str, bool]] = queue.Queue(maxsize=16)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self._publish_state('idle', detail='ready')
        self.get_logger().info(
            f'True agentic voice/action ready | session={self.session_name} '
            f'ollama={self.get_parameter("ollama_model").value} | transcript owner=/go2_voice/transcript'
        )

    def _build_graph(self):
        from langgraph.graph import END, START, StateGraph
        b = StateGraph(VoiceState)
        b.add_node('context', self._graph_context)
        b.add_node('plan', self._graph_plan)
        b.add_node('execute', self._graph_execute)
        b.add_node('respond', self._graph_respond)
        b.add_edge(START, 'context')
        b.add_edge('context', 'plan')
        b.add_edge('plan', 'execute')
        b.add_edge('execute', 'respond')
        b.add_edge('respond', END)
        return b.compile()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        yaw=2.0*math.atan2(float(q.z),float(q.w))
        self.latest_robot_pose={'frame_id':msg.header.frame_id or 'map','x':float(p.x),'y':float(p.y),'yaw':yaw}

    def _vlm_result_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        request_id = str(payload.get('request_id') or '')
        if not request_id:
            return
        with self._vlm_condition:
            self._vlm_results[request_id] = payload
            self._vlm_condition.notify_all()
        self.latest['live_vlm_result'] = payload

    @staticmethod
    def _is_side_effect_tool(tool: str) -> bool:
        return tool in {
            'navigate_to_place', 'find_object', 'start_tour', 'continue_tour', 'pause_tour',
            'save_tour_stop', 'update_tour_stop', 'reorder_tour_stops', 'motion_skill', 'stop',
        }

    @staticmethod
    def _fast_object_phrase(text: str, verbs: tuple[str, ...]) -> str:
        t=_clean_wake(text).strip().lower().strip(' .!?')
        for verb in verbs:
            if t.startswith(verb):
                q=t[len(verb):].strip()
                q=re.sub(r'^(the|a|an)\s+','',q).strip()
                return q
        return ''

    def _hybrid_fast_plan(self, text: str) -> Optional[Dict[str, Any]]:
        if not bool(self.get_parameter('hybrid_fast_path').value):
            return None
        t=_clean_wake(text).lower().strip(' .!?')
        # High-confidence commands only. Ambiguous/conversational language still goes to Ollama.
        # Current-vision questions must never be answered from remembered objects.
        if t in {
            'what do you see', 'what can you see', 'what do you see right now',
            'what can you see right now', 'describe what you see', 'tell me what you see',
            'what do you see in front of me', 'what is in front of you', 'what is in front of me',
        }:
            return {'tool':'live_vision_summary','args':{'question': text},'speech':''}
        if t in {'start tour','start the tour','begin tour','begin the tour'}:
            return {'tool':'start_tour','args':{},'speech':'Starting the saved tour.'}
        if t in {'continue tour','continue the tour','resume tour','resume the tour','next stop'}:
            return {'tool':'continue_tour','args':{},'speech':'Continuing to the next stop.'}
        if t in {'pause tour','pause the tour'}:
            return {'tool':'pause_tour','args':{},'speech':'Pausing the tour.'}
        checkpoint_aliases = {
            'welcome': 'welcome_checkpoint', 'welcome checkpoint': 'welcome_checkpoint',
            'entrance': 'welcome_checkpoint',
            'g1': 'g1_checkpoint', 'g1 checkpoint': 'g1_checkpoint',
            'unitree g1': 'g1_checkpoint', 'unitree g1 robot': 'g1_checkpoint',
            'humanoid': 'g1_checkpoint', 'humanoid checkpoint': 'g1_checkpoint',
            'xarm 7': 'xarm7_checkpoint', 'xarm7': 'xarm7_checkpoint',
            'xarm 7 checkpoint': 'xarm7_checkpoint', 'xarm7 checkpoint': 'xarm7_checkpoint',
            'robot arms': 'xarm7_checkpoint', 'robotic arms': 'xarm7_checkpoint',
            'roboarms checkpoint': 'xarm7_checkpoint',
        }
        for prefix in ('go to ','navigate to ','take me to ','take us to '):
            if t.startswith(prefix):
                raw_dest=t[len(prefix):].strip()
                dest=re.sub(r'^(the)\s+','',raw_dest).strip()
                if dest in checkpoint_aliases:
                    target=checkpoint_aliases[dest]
                    return {'tool':'navigate_to_place','args':{'place':target},'speech':f'Navigating to {target.replace("_", " ")}.'}

        # Deterministic object navigation: nearest remembered object -> dedup -> safe standoff -> Nav2.
        nearest_patterns = (
            r'^(?:go|navigate|take me|take us)\s+(?:to|near)\s+(?:the\s+)?nearest\s+(.+)$',
            r'^(?:find|locate)\s+(?:the\s+)?nearest\s+(.+)$',
        )
        for pattern in nearest_patterns:
            mm=re.fullmatch(pattern,t)
            if mm:
                label=re.sub(r'^(the|a|an)\s+','',mm.group(1).strip())
                return {'tool':'find_object','args':{'label':label,'nearest':True},'speech':''}
        # Saved checkpoint navigation is deterministic when the user explicitly names a checkpoint.
        for prefix in ('go to ','navigate to ','take me to '):
            if t.startswith(prefix):
                dest=t[len(prefix):].strip()
                if 'checkpoint' in dest:
                    return {'tool':'navigate_to_place','args':{'place':dest},'speech':f'Navigating to {dest}.'}
        motion={
            'wave':'hello','wave hello':'hello','say hello':'hello','stretch':'stretch',
            'sit':'sit','sit down':'sit','stand':'stand_up','stand up':'stand_up',
            'heart':'heart','show me some moves':'tour_greet','show some moves':'tour_greet',
            'handshake':'tour_handshake','shake hands':'tour_handshake',
            'front flip':'front_flip','frontflip':'front_flip','do a front flip':'front_flip',
        }
        if t in motion:
            return {'tool':'motion_skill','args':{'skill':motion[t]},'speech':''}
        q=self._fast_object_phrase(t,('find ','locate '))
        if q:
            q=re.sub(r'^(?:the\s+)?nearest\s+','',q).strip()
            return {'tool':'find_object','args':{'label':q,'nearest':True},'speech':''}
        q=self._fast_object_phrase(t,('where is ','where are '))
        if q:
            return {'tool':'query_object_memory','args':{'label':q},'speech':''}
        m=re.fullmatch(r'how many (.+?)(?: do you remember| are remembered| in memory)?',t)
        if m:
            label=re.sub(r'^(the|a|an)\s+','',m.group(1).strip())
            return {'tool':'count_object_memory','args':{'label':label},'speech':''}
        return None

    def _latest(self, key: str, msg: String) -> None:
        try:
            self.latest[key] = json.loads(msg.data)
        except Exception:
            self.latest[key] = msg.data

    def _enqueue(self, msg: String, source: str) -> None:
        text, detected_source = _extract_text(msg.data)
        text = str(text or '').strip()
        if not text:
            return
        source = detected_source or source
        read_only = source == 'query'
        try:
            _payload = json.loads(msg.data)
            if isinstance(_payload, dict):
                read_only = bool(_payload.get('read_only', read_only)) or str(_payload.get('input_kind') or '').lower() == 'query'
        except Exception:
            pass
        self._publish_state('listening', text=text, source=source, read_only=read_only)
        if _emergency_stop(text):
            self._execute_stop('emergency_stop')
            self._speak('Stopping now.', category='safety', priority='urgent')
            self._append_history('user', text, source)
            self._append_history('assistant', 'Stopping now.', 'safety')
            self._publish_state('idle', detail='emergency stop executed')
            return
        if self.pending_tool is not None and read_only and (_is_yes(text) or _is_no(text)):
            self._speak('The Ask panel is read-only and cannot confirm a pending motion. Use Command to confirm or cancel it.', category='safety')
            self._publish_state('read_only_confirmation_blocked', text=text, source=source)
            return
        if self.pending_tool is not None and (_is_yes(text) or _is_no(text)):
            self._append_history('user', text, source)
            if _is_no(text):
                self.pending_tool = None
                self._speak('Okay, I will not perform that motion.', category='confirmation')
                self._append_history('assistant', 'Okay, I will not perform that motion.', 'confirmation')
                self._publish_state('idle', detail='motion canceled')
            else:
                pending = self.pending_tool
                self.pending_tool = None
                self._publish_state('acting', tool='motion_skill', args=pending.get('args', {}))
                result = self._execute_tool(pending)
                speech = str(pending.get('speech') or 'Okay.')
                self._speak(speech, category='action')
                self._append_history('assistant', speech, 'agentic_action')
                self._publish_state('idle', tool_result=result)
            return
        try:
            self.work_q.put_nowait((text, source, read_only))
        except queue.Full:
            self._speak('I am still working on the previous request. Please try again in a moment.', category='busy')

    def _worker(self) -> None:
        while rclpy.ok():
            try:
                text, source, read_only = self.work_q.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                self._current_read_only = bool(read_only)
                self._current_text = text
                self._active_request_id = f'req_{time.time_ns()}'
                self._publish_state('request_received', request_id=self._active_request_id, text=text, source=source, read_only=self._current_read_only)
                self._append_history('user', text, source)
                fast_plan = self._hybrid_fast_plan(text)
                if fast_plan is not None:
                    self._publish_state('acting', route='hybrid_fast_path', tool=fast_plan.get('tool'), args=fast_plan.get('args', {}))
                    executed = self._graph_execute({'text': text, 'source': source, 'plan': fast_plan})
                    responded = self._graph_respond({'text': text, 'source': source, 'plan': fast_plan, **executed})
                    result = {'plan': fast_plan, **executed, **responded, 'routing': 'hybrid_fast_path'}
                else:
                    self._publish_state('thinking', text=text, route='ollama')
                    result = self.graph.invoke({'text': text, 'source': source})
                    result['routing'] = 'ollama'
                response = str(result.get('response') or '').strip()
                if response:
                    self._speak(response, category='agentic')
                    self._append_history('assistant', response, 'ollama_agent')
                self._publish_state('idle', plan=result.get('plan', {}), tool_result=result.get('tool_result', {}))
            except Exception as exc:
                self.get_logger().exception(f'agentic request failed: {exc}')
                message = f'My local reasoning agent failed: {exc}. I will stay stopped rather than guess.'
                self._speak(message, category='error')
                self._publish_state('error', error=str(exc))
            finally:
                self._current_read_only = False
                self._current_text = ''
                self._active_request_id = ''
                self.work_q.task_done()

    def _graph_context(self, state: VoiceState) -> Dict[str, Any]:
        text = state.get('text', '')
        try:
            resume_context = self.memory.get_resume_context()
        except Exception as exc:
            resume_context = {'error': str(exc)}
        try:
            query_memory = self.memory.query(text)
        except Exception as exc:
            query_memory = {'error': str(exc)}
        object_hint = self.object_memory.hint_for_utterance(text, limit=5)
        context = {
            'session_name': self.session_name,
            'resume_context': _bounded(resume_context, 2800),
            'memory_query': _bounded(query_memory, 3200),
            'object_memory_hint': object_hint,
            'session_world': self._session_world_summary(),
            'live': _bounded(self.latest, 4500),
            'conversation_history': self.history[-int(self.get_parameter('history_turns').value):],
        }
        return {'context': context}

    def _graph_plan(self, state: VoiceState) -> Dict[str, Any]:
        system = (
            'You are Sparky, a Unitree Go2 autonomous university tour-guide robot. '
            'You are the PRIMARY semantic and action planner, not a keyword router. '
            'Interpret the user naturally using conversation history, saved map/route memory, object memory, and live robot state. '
            'Choose exactly ONE allowed tool. Never invent a checkpoint or object that is not in context. '
            'For normal questions, answer conversationally and choose speak. '
            'For navigation, use a SAVED checkpoint/place name exactly as present in context. '
            'When the user says save this checkpoint/place, extract a concise name and useful narration into save_tour_stop. '
            'When they ask to change what is said at a stop, use update_tour_stop. '
            'For remembered-object questions such as where/count/remember, choose query_object_memory instead of guessing from prose context. '
            'For find/locate/take-me-to an object, choose find_object. The tool owns object retrieval and Nav2 goal generation. '
            'Object map_pose/object_pose means the physical object location, never the robot observation pose. '
            'For show me some moves choose motion_skill tour_greet; for handshake choose tour_handshake; wave maps to hello. '
            'For current-vision questions use live_vision_summary; never infer current visibility from persistent object memory. '
            'Do not claim an action succeeded before ROS executes it; phrase action speech as intent such as "I will go there now." '
            'Return JSON ONLY with keys tool, args, speech, confidence. '
            f'ALLOWED_TOOLS={json.dumps(self.TOOL_SCHEMA, default=str)}'
        )
        user = json.dumps({'utterance': state.get('text', ''), 'context': state.get('context', {})}, default=str)[:30000]
        plan = self.planner.call(system, user)
        tool = str(plan.get('tool') or 'speak').strip()
        allowed = {x['name'] for x in self.TOOL_SCHEMA}
        if tool not in allowed:
            raise RuntimeError(f'Ollama selected unsupported tool: {tool}')
        args = plan.get('args') if isinstance(plan.get('args'), dict) else {}
        speech = str(plan.get('speech') or '').strip()
        try:
            confidence = max(0.0, min(1.0, float(plan.get('confidence', 0.5))))
        except Exception:
            confidence = 0.5
        return {'plan': {'tool': tool, 'args': args, 'speech': speech, 'confidence': confidence}}

    def _graph_execute(self, state: VoiceState) -> Dict[str, Any]:
        plan = dict(state.get('plan') or {})
        tool = str(plan.get('tool') or 'speak')
        if self._current_read_only and self._is_side_effect_tool(tool):
            normalized_request = re.sub(r'[^a-z0-9]+', ' ', self._current_text.lower()).strip()
            if tool == 'stop' and normalized_request in {'stop', 'stop now', 'emergency stop', 'stop moving'}:
                # An explicit stop remains fail-safe even when entered through Ask.
                self._publish_state('read_only_emergency_stop_allowed', request_id=self._active_request_id, tool=tool)
            elif tool == 'find_object':
                safe_plan = dict(plan)
                safe_plan['tool'] = 'query_object_memory'
                safe_plan['speech'] = ''
                self._publish_state('read_only_reroute', request_id=self._active_request_id, blocked_tool=tool, safe_tool='query_object_memory')
                return {'tool_result': self._execute_tool(safe_plan)}
            else:
                return {'tool_result': {
                    'ok': False, 'read_only_blocked': True, 'tool': tool,
                    'speech': 'That Ask panel is read-only, so I did not execute a motion or navigation action. Use Command when you want me to act.'
                }}
        if tool == 'motion_skill':
            args = plan.get('args') if isinstance(plan.get('args'), dict) else {}
            skill = _normalize_motion_skill(str(args.get('skill') or ''))
            args['skill'] = skill
            plan['args'] = args
            risk = _motion_risk(skill)
            policy = str(self.get_parameter('motion_confirmation_policy').value or 'risky_only').strip().lower()
            force = bool(self.get_parameter('require_motion_skill_confirmation').value)
            needs_confirmation = (policy == 'always') or (policy == 'risky_only' and risk in {'medium','high'})
            if force and policy == 'always':
                needs_confirmation = True
            if needs_confirmation:
                self.pending_tool = plan
                return {'tool_result': {'pending_confirmation': True, 'skill': skill, 'risk': risk}}
        self._publish_state('acting' if tool != 'speak' else 'thinking', tool=tool, args=plan.get('args', {}))
        return {'tool_result': self._execute_tool(plan)}

    def _graph_respond(self, state: VoiceState) -> Dict[str, Any]:
        plan = state.get('plan') or {}
        result = state.get('tool_result') or {}
        if result.get('pending_confirmation'):
            skill = result.get('skill') or 'that motion'
            return {'response': f'I can perform {skill}. Say yes to confirm, or no to cancel.'}
        grounded_speech = str(result.get('speech') or '').strip()
        if grounded_speech:
            return {'response': grounded_speech}
        speech = str(plan.get('speech') or '').strip()
        if speech:
            return {'response': speech}
        if result.get('error'):
            return {'response': f'I could not complete that request: {result.get("error")}'}
        return {'response': 'Okay.'}

    def _execute_tool(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        tool = str(plan.get('tool') or 'speak')
        args = plan.get('args') if isinstance(plan.get('args'), dict) else {}
        if tool == 'speak':
            return {'ok': True, 'tool': tool}
        if tool == 'live_vision_summary':
            question = str(args.get('question') or self._current_text or 'What do you see in front of me?').strip()
            request_id = f'{self._active_request_id or "req"}_vlm_{time.time_ns()}'
            inventory = self.latest.get('object_inventory')
            if not isinstance(inventory, dict):
                inventory = {}
            request = {
                'request_id': request_id, 'question': question,
                'object_inventory': inventory, 'source': 'agent_live_vision',
            }
            with self._vlm_condition:
                self._vlm_results.pop(request_id, None)
            self.event_pub.publish(String(data=json.dumps({
                'type': 'vlm_call', 'phase': 'request', 'request_id': self._active_request_id,
                'vlm_request_id': request_id, 'question': question,
            }, sort_keys=True)))
            self.vlm_query_pub.publish(String(data=json.dumps(request, sort_keys=True, default=str)))
            deadline = time.monotonic() + max(1.0, float(self.get_parameter('live_vlm_timeout_sec').value))
            result = None
            with self._vlm_condition:
                while rclpy.ok() and time.monotonic() < deadline:
                    result = self._vlm_results.pop(request_id, None)
                    if result is not None:
                        break
                    self._vlm_condition.wait(timeout=min(0.25, max(0.01, deadline - time.monotonic())))
            if not isinstance(result, dict):
                return {'ok': False, 'tool': tool, 'speech': 'My live camera VLM did not answer in time, so I will not reuse an old visual answer.', 'vlm_request_id': request_id}
            self.event_pub.publish(String(data=json.dumps({
                'type': 'vlm_call', 'phase': 'result', 'request_id': self._active_request_id,
                'vlm_request_id': request_id, 'success': bool(result.get('success')),
                'provider': result.get('provider'), 'model': result.get('model'),
                'error': result.get('error', ''),
            }, sort_keys=True, default=str)))
            summary = str(result.get('summary') or '').strip()
            if not bool(result.get('success')) or not summary:
                err = str(result.get('error') or 'unknown VLM error')
                return {'ok': False, 'tool': tool, 'speech': f'My live vision call failed: {err}. I will not guess from stale memory.', 'vlm': result}
            return {'ok': True, 'tool': tool, 'speech': summary, 'vlm': result, 'fresh_live_image': True}
        if tool == 'stop':
            return self._execute_stop('agent_request')
        if tool == 'navigate_to_place':
            place = str(args.get('place') or args.get('checkpoint') or '').strip()
            if not place:
                return {'ok': False, 'error': 'missing saved place name'}
            self._semantic({'type': 'go', 'place': place, 'source': 'ollama_agent'})
            return {'ok': True, 'tool': tool, 'place': place}
        if tool == 'start_tour':
            self._semantic({'type': 'start_tour', 'reset_index': True, 'source': 'ollama_agent'})
            return {'ok': True, 'tool': tool}
        if tool == 'continue_tour':
            self._semantic({'type': 'resume_tour', 'source': 'ollama_agent'})
            return {'ok': True, 'tool': tool}
        if tool == 'pause_tour':
            self._semantic({'type': 'pause_tour', 'source': 'ollama_agent'})
            return {'ok': True, 'tool': tool}
        if tool == 'save_tour_stop':
            name = str(args.get('name') or '').strip()
            if not name:
                return {'ok': False, 'error': 'checkpoint name is required'}
            event = {
                'type': 'save_tour_stop',
                'name': name,
                'script': str(args.get('description') or args.get('speech') or '').strip(),
                'fact': str(args.get('fact') or '').strip(),
                'room': str(args.get('room') or '').strip(),
                'pause_seconds': float(args.get('pause_seconds') or 0.0),
                'tags': ['tour_stop'],
                'source': 'ollama_agent',
            }
            self._semantic(event)
            return {'ok': True, 'tool': tool, 'name': name}
        if tool == 'update_tour_stop':
            name = str(args.get('name') or '').strip()
            if not name:
                return {'ok': False, 'error': 'checkpoint name is required'}
            self._semantic({
                'type': 'update_tour_stop',
                'name': name,
                'script': str(args.get('description') or args.get('speech') or '').strip(),
                'fact': str(args.get('fact') or '').strip(),
                'source': 'ollama_agent',
            })
            return {'ok': True, 'tool': tool, 'name': name}
        if tool == 'reorder_tour_stops':
            order = args.get('order')
            if not isinstance(order, list) or not order:
                return {'ok': False, 'error': 'order must be a non-empty list of checkpoint names'}
            self._semantic({'type': 'reorder_tour_stops', 'order': [str(x) for x in order], 'source': 'ollama_agent'})
            return {'ok': True, 'tool': tool, 'order': order}
        if tool == 'query_object_memory':
            label = str(args.get('label') or args.get('object') or '').strip()
            room = str(args.get('room') or '').strip()
            if not label:
                return {'ok': False, 'error': 'object label is required'}
            result = self.object_memory.best(label, room)
            result['tool'] = tool
            return result
        if tool == 'count_object_memory':
            label = str(args.get('label') or args.get('object') or '').strip()
            room = str(args.get('room') or '').strip()
            if not label:
                return {'ok': False, 'error': 'object label is required'}
            result = self.object_memory.count_unique(label, room)
            result['tool'] = tool
            return result
        if tool == 'find_object':
            label = str(args.get('label') or args.get('object') or '').strip()
            room = str(args.get('room') or '').strip()
            if not label:
                return {'ok': False, 'error': 'object label is required'}
            searched = self.object_memory.search(label, room, limit=12)
            matches = searched.get('matches') or []
            object_source = 'durable_object_memory'
            if not matches:
                live_map = self.latest.get('live_object_map')
                live_objects = live_map.get('objects', []) if isinstance(live_map, dict) else []
                qlabel = re.sub(r'[^a-z0-9]+', ' ', label.lower()).strip()
                live_matches = []
                for item in live_objects if isinstance(live_objects, list) else []:
                    if not isinstance(item, dict) or not bool(item.get('confirmed')):
                        continue
                    try:
                        conf = float(item.get('confidence') or 0.0)
                    except Exception:
                        conf = 0.0
                    if conf <= 0.60:
                        continue
                    ilabel = re.sub(r'[^a-z0-9]+', ' ', str(item.get('label') or '').lower()).strip()
                    if not ilabel or not (qlabel == ilabel or qlabel in ilabel or ilabel in qlabel):
                        continue
                    try:
                        pose = {'frame_id':'map','x':float(item['x']),'y':float(item['y']),'z':float(item.get('z') or 0.0)}
                    except Exception:
                        continue
                    live_matches.append({
                        'id': f"live_mapper_{item.get('object_id', 'na')}", 'label': str(item.get('label') or label),
                        'object_pose': pose, 'map_pose': dict(pose), 'confidence': conf,
                        'confirmations': int(item.get('confirmations') or 0), 'observations': int(item.get('observations') or 0),
                        'extent_x': item.get('extent_x'), 'extent_y': item.get('extent_y'), 'extent_z': item.get('extent_z'),
                        'confirmed': True, 'source': 'live_confirmed_mapper_fallback',
                    })
                if live_matches:
                    matches = live_matches
                    object_source = 'live_confirmed_mapper_fallback'
                    self._publish_state('object_memory_live_fallback', object_label=label, candidates=len(matches))
            if not matches:
                inv = self.latest.get('object_inventory')
                visible = (inv.get('visible_counts') or {}) if isinstance(inv, dict) else {}
                visible_count = 0
                for k, v in visible.items() if isinstance(visible, dict) else []:
                    if qlabel == re.sub(r'[^a-z0-9]+', ' ', str(k).lower()).strip():
                        try: visible_count += int(v)
                        except Exception: pass
                if visible_count:
                    return {'ok': False, 'tool': tool, 'visible_only': True, 'speech': f'I can currently see {label}, but I do not yet have a confirmed registered-3D map location for it. I need another valid mapped observation before I can navigate safely.'}
                return {'ok': False, 'tool': tool, 'speech': f'I do not have a confirmed mapped location for {label}.'}
            # Prefer the closest high-quality UNIQUE object to the current localized robot pose.
            rx=self.latest_robot_pose.get('x'); ry=self.latest_robot_pose.get('y')
            def rank(o):
                p=o.get('object_pose') or {}
                try: dist=math.hypot(float(p.get('x'))-float(rx),float(p.get('y'))-float(ry))
                except Exception: dist=99.0
                return dist - 0.25*float(o.get('confidence') or 0.0) - 0.03*int(o.get('confirmations') or 0)
            matches=sorted(matches,key=rank)
            obj=matches[0]
            selected=self.object_nav.select(obj,self.latest_map,self.latest_robot_pose)
            if not selected.get('success'):
                return {'ok':False,'tool':tool,'speech':f'I remember {label}, but I cannot find a safe reachable standoff near it right now.','details':selected}
            pose=selected.get('pose') or {}
            self._publish_state('acting', action='object_nav_dispatch', object_label=str(obj.get('label') or label), navigation_pose=pose, planning_verified=bool(selected.get('planning_verified')))
            before_status = json.dumps(self.latest.get('semantic_nav_status'), sort_keys=True, default=str)
            self._semantic({
                'type':'navigate_to_pose','pose':pose,
                'name':str(obj.get('label') or label),'object_id':obj.get('id'),
                'object_label':str(obj.get('label') or label),'object_pose':obj.get('object_pose') or {},
                'room':obj.get('room_id') or room,'verify_live_after_arrival':True,
                'planning_verified':bool(selected.get('planning_verified')),'source':f'{object_source}_nav2_selector',
            })
            deadline = time.monotonic() + max(0.5, float(self.get_parameter('semantic_dispatch_ack_timeout_sec').value))
            nav_ack_status = ''
            last_progress = ''
            while rclpy.ok() and time.monotonic() < deadline:
                current = self.latest.get('semantic_nav_status')
                current_text = json.dumps(current, sort_keys=True, default=str)
                low = current_text.lower()
                if current_text != before_status:
                    if any(token in low for token in (
                        'goal rejected', 'server not available', 'skipping_goal_', 'refusing_go_pose',
                        'goal_send_failed', 'failed',
                    )):
                        nav_ack_status = current_text
                        break
                    if any(token in low for token in ('goal accepted', 'navigating target=')):
                        nav_ack_status = current_text
                        break
                    if any(token in low for token in ('object_memory_goal', 'sending_goal')):
                        last_progress = current_text
                time.sleep(0.05)
            if not nav_ack_status:
                nav_ack_status = last_progress
            low_ack = nav_ack_status.lower()
            accepted = bool(nav_ack_status) and any(x in low_ack for x in ('goal accepted', 'navigating target='))
            if not accepted:
                return {
                    'ok':False,'success':False,'tool':tool,'nav_dispatched':False,'object':obj,
                    'navigation_pose':pose,'planning_verified':bool(selected.get('planning_verified')),
                    'nav_ack_status':nav_ack_status or 'no semantic-nav acknowledgement',
                    'speech':f'I found {obj.get("label", label)}, but Nav2 did not acknowledge the navigation dispatch, so I will not claim that I am moving.'
                }
            return {
                'ok':True,'success':True,'tool':tool,'nav_dispatched':True,'object':obj,
                'navigation_pose':pose,'planning_verified':bool(selected.get('planning_verified')),
                'nav_ack_status':nav_ack_status,
                'speech':f'I found a unique remembered {obj.get("label", label)} and Nav2 accepted the route toward a safe standoff.'
            }
        if tool == 'motion_skill':
            skill = _normalize_motion_skill(str(args.get('skill') or '').strip())
            allowed = MOTION_LOW_RISK | MOTION_MEDIUM_RISK | MOTION_HIGH_RISK
            if skill not in allowed:
                return {'ok': False, 'error': f'unsupported motion skill {skill}'}
            self.motion_pub.publish(String(data=json.dumps({'command': skill, 'skill': skill, 'source': 'ollama_agent'})))
            return {'ok': True, 'tool': tool, 'skill': skill, 'risk': _motion_risk(skill), 'speech': f'Performing {skill.replace("_", " ")}.'}
        return {'ok': False, 'error': f'unhandled tool {tool}'}

    def _execute_stop(self, reason: str) -> Dict[str, Any]:
        self._semantic({'type': 'cancel', 'reason': reason, 'source': 'agentic_safety'})
        self.motion_pub.publish(String(data=json.dumps({'command': 'stop_move', 'skill': 'stop_move', 'source': 'agentic_safety'})))
        return {'ok': True, 'tool': 'stop', 'reason': reason}

    def _semantic(self, payload: Dict[str, Any]) -> None:
        payload = dict(payload)
        if self._active_request_id:
            payload.setdefault('request_id', self._active_request_id)
        self.semantic_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.event_pub.publish(String(data=json.dumps({'type': 'tool_call', 'request_id': self._active_request_id, 'tool': payload.get('type'), 'payload': payload}, sort_keys=True)))

    def _speak(self, text: str, *, category: str = 'speech', priority: str = 'normal') -> None:
        text = str(text or '').strip()
        if not text:
            return
        self._publish_state('speaking', text=text, category=category)
        self.speech_pub.publish(String(data=json.dumps({
            'text': text,
            'source': 'go2_agentic_voice_action',
            'category': category,
            'priority': priority,
        }, sort_keys=True)))

    def _publish_state(self, state: str, **extra: Any) -> None:
        payload = {'state': state, 'session_name': self.session_name, 'stamp_sec': time.time(), **extra}
        if self._active_request_id and 'request_id' not in payload:
            payload['request_id'] = self._active_request_id
        msg = String(data=json.dumps(payload, sort_keys=True, default=str))
        self.state_pub.publish(msg)
        self.status_pub.publish(msg)

    def _append_history(self, role: str, text: str, source: str) -> None:
        row = {'stamp_sec': time.time(), 'role': role, 'text': str(text)[:6000], 'source': source}
        self.history.append(row)
        self.history = self.history[-max(8, int(self.get_parameter('history_turns').value) * 2):]
        try:
            with self.history_path.open('a', encoding='utf-8') as fh:
                fh.write(json.dumps(row, sort_keys=True) + '\n')
        except Exception as exc:
            self.get_logger().warning(f'history write failed: {exc}')

    def _load_history(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        if self.history_path.is_file():
            try:
                for line in self.history_path.read_text(encoding='utf-8').splitlines():
                    try:
                        row = json.loads(line)
                    except Exception:
                        continue
                    if isinstance(row, dict) and row.get('role') and row.get('text'):
                        rows.append(row)
            except Exception:
                pass
        return rows[-40:]

    def _session_world_summary(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {'object_counts': {}, 'route_stops': [], 'places': [], 'vlm_checkpoint_count': 0}
        objects = self.session_dir / 'memory' / 'objects.jsonl'
        counts: Counter[str] = Counter()
        if objects.is_file():
            try:
                lines = objects.read_text(encoding='utf-8').splitlines()[-int(self.get_parameter('max_object_records').value):]
                for line in lines:
                    try:
                        row = json.loads(line)
                    except Exception:
                        continue
                    data = row.get('data') if isinstance(row, dict) and isinstance(row.get('data'), dict) else row
                    if not isinstance(data, dict):
                        continue
                    label = str(data.get('class_name') or data.get('label') or data.get('name') or data.get('class') or '').strip().lower()
                    if label:
                        counts[label] += 1
            except Exception:
                pass
        out['object_counts'] = dict(counts)
        try:
            import yaml
            route = self.session_dir / 'route.yaml'
            if route.is_file():
                data = yaml.safe_load(route.read_text(encoding='utf-8')) or {}
                if isinstance(data, dict) and isinstance(data.get('route'), dict):
                    data = data['route']
                stops = data.get('stops') if isinstance(data, dict) else []
                if isinstance(stops, list):
                    out['route_stops'] = [
                        {
                            'name': str(x.get('name') or x.get('place_name') or ''),
                            'place_name': str(x.get('place_name') or ''),
                            'script': str(x.get('script') or '')[:500],
                            'fact': str(x.get('fact') or '')[:500],
                        }
                        for x in stops if isinstance(x, dict)
                    ][:20]
            places = self.session_dir / 'places.yaml'
            if places.is_file():
                data = yaml.safe_load(places.read_text(encoding='utf-8')) or {}
                if isinstance(data, dict):
                    raw_places = data.get('places', data)
                    if isinstance(raw_places, dict):
                        out['places'] = list(raw_places.keys())[:40]
                    elif isinstance(raw_places, list):
                        out['places'] = [str(x.get('name') or '') for x in raw_places if isinstance(x, dict)][:40]
            vlm = self.session_dir / 'memory' / 'vlm_checkpoints.yaml'
            if vlm.is_file():
                data = yaml.safe_load(vlm.read_text(encoding='utf-8')) or {}
                if isinstance(data, dict):
                    out['vlm_checkpoint_count'] = int(data.get('count') or len(data.get('checkpoints') or []))
        except Exception as exc:
            out['yaml_error'] = str(exc)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgenticVoiceActionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
