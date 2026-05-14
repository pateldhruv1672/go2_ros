from __future__ import annotations

import atexit
import base64
import io
import json
import math
import os
import re
import shlex
import subprocess
from dataclasses import asdict
from typing import Optional

import numpy as np
from PIL import Image as PILImage
import requests
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .place_store import Place, PlaceStore
from .session_store import SessionStore
from .semantic_memory import SemanticMemory, slugify


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def image_msg_to_jpeg_bytes(msg: Image) -> Optional[bytes]:
    try:
        width = int(msg.width)
        height = int(msg.height)
        enc = (msg.encoding or '').lower()
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ('rgb8', '8uc3'):
            arr = raw.reshape((height, msg.step // 3, 3))[:, :width, :]
            pil = PILImage.fromarray(arr, mode='RGB')
        elif enc == 'bgr8':
            arr = raw.reshape((height, msg.step // 3, 3))[:, :width, :]
            pil = PILImage.fromarray(arr[:, :, ::-1], mode='RGB')
        elif enc in ('rgba8', '8uc4'):
            arr = raw.reshape((height, msg.step // 4, 4))[:, :width, :]
            pil = PILImage.fromarray(arr, mode='RGBA').convert('RGB')
        elif enc == 'bgra8':
            arr = raw.reshape((height, msg.step // 4, 4))[:, :width, :]
            arr = arr[:, :, [2, 1, 0, 3]]
            pil = PILImage.fromarray(arr, mode='RGBA').convert('RGB')
        elif enc in ('mono8', '8uc1'):
            arr = raw.reshape((height, msg.step))[:, :width]
            pil = PILImage.fromarray(arr, mode='L').convert('RGB')
        else:
            return None
        buf = io.BytesIO()
        pil.save(buf, format='JPEG', quality=85)
        return buf.getvalue()
    except Exception:
        return None


class SemanticNavNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_nav_node')
        self._declare_parameters()
        self.mode = str(self.get_parameter('mode').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.session_store = SessionStore(str(self.get_parameter('session_root').value))
        self.session = self._choose_session()
        self.place_store = PlaceStore(self.session.places_path)
        self.memory = SemanticMemory()
        if self.mode == 'teach' and bool(self.get_parameter('clear_places_on_start').value):
            self.place_store.places = {}
            self.place_store.save()

        self.latest_image_bytes: Optional[bytes] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_goal: Optional[PoseStamped] = None
        self.last_auto_save_xy: Optional[tuple[float, float]] = None
        self._last_status = ''
        self._goal_place: Optional[Place] = None
        self._fallback_attempts = 0
        self._last_goal_failed = False
        self._restore_count = 0
        self._restore_done = False
        self._motion_active = False
        self._motion_twist = Twist()
        self._motion_end_ns = 0
        self._motion_retry_place: Optional[Place] = None

        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cmd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self.status_pub = self.create_publisher(String, '/semantic_nav/status', status_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_nav/places_markers', marker_qos)
        self.preview_pub = self.create_publisher(MarkerArray, '/semantic_nav/route_preview', marker_qos)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, str(self.get_parameter('fallback_cmd_topic').value), cmd_qos)

        self.create_subscription(String, '/semantic_nav/command', self.cmd_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Image, str(self.get_parameter('camera_image_topic').value), self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, str(self.get_parameter('camera_compressed_topic').value), self.compressed_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_fixed', self.scan_cb, qos_profile_sensor_data)

        self.create_timer(1.0, self.publish_markers)
        self.create_timer(0.05, self.motion_tick)
        self.auto_save_timer = None
        if self.mode == 'teach' and bool(self.get_parameter('auto_save_places').value):
            self.auto_save_timer = self.create_timer(float(self.get_parameter('auto_save_interval_sec').value), self.auto_save_tick)
        if self.mode == 'resume' and bool(self.get_parameter('restore_spawn_on_start').value):
            self.create_timer(0.5, self.restore_spawn_when_tf_ready)

        atexit.register(self.on_shutdown)
        self.publish_status(
            f'ready | mode={self.mode} | session={self.session.session_name} | '
            f'places_file={self.session.places_path} | places_loaded={len(self.place_store.places)} | '
            f'auto_save={"on" if bool(self.get_parameter("auto_save_places").value) and self.mode=="teach" else "off"} '
            f'every={float(self.get_parameter("auto_save_interval_sec").value):.1f}s | '
            f'vlm={"on" if bool(self.get_parameter("auto_save_use_vlm").value) else "off"} | '
            f'fallback={"on" if bool(self.get_parameter("fallback_enable").value) else "off"}'
        )
        self.publish_markers()

    def _declare_parameters(self) -> None:
        self.declare_parameter('mode', 'teach')
        self.declare_parameter('session_root', '~/.ros/go2_semantic_nav_sessions')
        self.declare_parameter('session_name', '')
        self.declare_parameter('map_label', 'session')
        self.declare_parameter('save_map_on_shutdown', True)
        self.declare_parameter('map_saver_cmd', 'ros2 run nav2_map_server map_saver_cli')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_compressed_topic', '/camera/image_raw/compressed')
        self.declare_parameter('openrouter_model', 'google/gemini-2.5-flash')
        self.declare_parameter('openrouter_base_url', 'https://openrouter.ai/api/v1/chat/completions')
        self.declare_parameter('auto_save_places', True)
        self.declare_parameter('auto_save_interval_sec', 5.0)
        self.declare_parameter('auto_save_use_vlm', True)
        self.declare_parameter('auto_save_min_distance_m', 1.5)
        self.declare_parameter('auto_save_merge_distance_m', 1.5)
        self.declare_parameter('auto_save_min_confidence', 0.55)
        self.declare_parameter('clear_places_on_start', False)
        self.declare_parameter('restore_spawn_on_start', True)
        self.declare_parameter('fallback_enable', True)
        self.declare_parameter('fallback_max_attempts', 3)
        self.declare_parameter('fallback_cmd_topic', '/cmd_vel')
        self.declare_parameter('fallback_linear_speed', 0.08)
        self.declare_parameter('fallback_angular_speed', 0.35)
        self.declare_parameter('fallback_forward_distance', 0.25)
        self.declare_parameter('fallback_backup_distance', 0.18)
        self.declare_parameter('fallback_rotate_deg', 18.0)

    def _choose_session(self):
        requested = str(self.get_parameter('session_name').value).strip()
        if self.mode == 'teach':
            return self.session_store.create(str(self.get_parameter('map_label').value))
        if requested:
            return self.session_store.for_name(requested)
        usable = self.session_store.latest_usable()
        if usable is None:
            latest = self.session_store.latest()
            if latest is None:
                raise RuntimeError('No saved session found for resume mode')
            return latest
        return usable

    def publish_status(self, text: str) -> None:
        if text == self._last_status:
            return
        self._last_status = text
        msg = String(); msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def goal_cb(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        self.publish_status(f'cached_last_goal frame={msg.header.frame_id or self.map_frame}')

    def image_cb(self, msg: Image) -> None:
        jpeg = image_msg_to_jpeg_bytes(msg)
        if jpeg:
            self.latest_image_bytes = jpeg

    def compressed_cb(self, msg: CompressedImage) -> None:
        self.latest_image_bytes = bytes(msg.data)

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def odom_tf_ready(self) -> bool:
        try:
            self.buffer.lookup_transform('odom', self.base_frame, Time(), timeout=Duration(seconds=0.2))
            return True
        except TransformException:
            return False

    def restore_spawn_when_tf_ready(self) -> None:
        if self._restore_done:
            return
        if not self.odom_tf_ready():
            self.publish_status('waiting_for_odom_tf_before_spawn_restore')
            return
        meta = self.session_store.load_session_yaml(self.session)
        spawn = meta.get('spawn')
        if not spawn:
            self.publish_status('resume_no_spawn_found')
            self._restore_done = True
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = spawn.get('frame_id', self.map_frame)
        msg.header.stamp = BuiltinTime(sec=0, nanosec=0)
        msg.pose.pose.position.x = float(spawn.get('x', 0.0))
        msg.pose.pose.position.y = float(spawn.get('y', 0.0))
        qx, qy, qz, qw = quaternion_from_yaw(float(spawn.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.10
        msg.pose.covariance[7] = 0.10
        msg.pose.covariance[35] = 0.10
        self.initialpose_pub.publish(msg)
        self._restore_count += 1
        self.publish_status(
            f'restored_spawn_initialpose name=spawn x={msg.pose.pose.position.x:.2f} '
            f'y={msg.pose.pose.position.y:.2f} yaw={float(spawn.get("yaw", 0.0)):.2f} count={self._restore_count}'
        )
        if self._restore_count >= 3:
            self._restore_done = True

    def lookup_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.buffer.lookup_transform(self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.4))
        except TransformException as exc:
            self.publish_status(f'Could not lookup current pose in TF: {exc}')
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        return pose

    def build_pose(self, place: Place) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(place.x)
        pose.pose.position.y = float(place.y)
        qx, qy, qz, qw = quaternion_from_yaw(float(place.yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_markers(self) -> None:
        arr = MarkerArray()
        t = self.get_clock().now().to_msg()
        for idx, p in enumerate(sorted(self.place_store.places.values(), key=lambda x: x.name)):
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = t
            arrow.ns = 'places'
            arrow.id = idx * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.35
            arrow.scale.y = 0.07
            arrow.scale.z = 0.07
            arrow.pose = self.build_pose(p).pose
            arrow.color.a = 1.0
            arrow.color.r = 0.1
            arrow.color.g = 0.6
            arrow.color.b = 1.0
            arr.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = t
            label.ns = 'place_labels'
            label.id = idx * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.5
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = 0.55
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.text = p.name
            arr.markers.append(label)
        self.marker_pub.publish(arr)

    def parse_kv(self, parts):
        kwargs = {'room': '', 'category': '', 'tags': [], 'aliases': [], 'description': ''}
        for part in parts:
            if '=' not in part:
                continue
            k, v = part.split('=', 1)
            if k in {'tags', 'aliases'}:
                kwargs[k] = [slugify(x) for x in v.split(',') if slugify(x)]
            elif k in kwargs:
                kwargs[k] = slugify(v) if k in {'room', 'category'} else v
        return kwargs

    def persist_spawn_if_needed(self, place: Place) -> None:
        meta = self.session_store.load_session_yaml(self.session)
        meta.update({'session_name': self.session.session_name, 'map_yaml': self.session.map_prefix + '.yaml'})
        meta.setdefault('semantic_graph', self.memory.build_relationships(list(self.place_store.places.values())))
        if place.name.lower() == 'spawn':
            meta['spawn'] = {'x': place.x, 'y': place.y, 'yaw': place.yaw, 'frame_id': self.map_frame}
        self.session_store.save_session_yaml(self.session, meta)

    def save_place(self, place: Place, source_text: str = 'manual') -> None:
        self.place_store.upsert(place)
        self.place_store.save()
        self.persist_spawn_if_needed(place)
        self.publish_status(f'saved_place name={place.name} source={source_text} x={place.x:.2f} y={place.y:.2f} yaw={place.yaw:.2f}')
        self.publish_markers()

    def make_place(self, name: str, pose: PoseStamped, extra: dict, source: str = 'manual', confidence: float = 1.0) -> Place:
        return Place(
            name=name,
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=float(yaw_from_quat(pose.pose.orientation)),
            room=extra.get('room', ''),
            category=extra.get('category', ''),
            aliases=extra.get('aliases', []),
            tags=extra.get('tags', []),
            description=extra.get('description', ''),
            confidence=float(confidence),
            source=source,
        )

    def choose_place_name(self, meta: dict, pose: PoseStamped) -> str:
        # use previous labels as context to preserve naming stability
        label = slugify(str(meta.get('label', '')))
        room = slugify(str(meta.get('room', '')))
        category = slugify(str(meta.get('category', '')))
        candidates = [c for c in [label, f'{room}_{category}' if room and category else '', room, category] if c and c != 'unknown']
        base = candidates[0] if candidates else 'labeled_place'
        near = self.place_store.nearest_within(float(pose.pose.position.x), float(pose.pose.position.y), float(self.get_parameter('auto_save_merge_distance_m').value))
        if near:
            if base in near.name or near.name in base or (near.category and near.category == category):
                return near.name
        return self.place_store.unique_name(base)

    def extract_json(self, text: str) -> Optional[dict]:
        try:
            return json.loads(text)
        except Exception:
            pass
        m = re.search(r'\{.*\}', text, re.S)
        if not m:
            return None
        try:
            return json.loads(m.group(0))
        except Exception:
            return None

    def _call_openrouter(self, prompt: str) -> Optional[dict]:
        if not self.latest_image_bytes:
            self.publish_status('No camera frame received yet.')
            return None
        key = os.environ.get('OPENROUTER_API_KEY', '')
        if not key:
            self.publish_status('VLM is disabled or OPENROUTER_API_KEY is missing.')
            return None
        try:
            b64 = base64.b64encode(self.latest_image_bytes).decode('ascii')
            payload = {
                'model': self.get_parameter('openrouter_model').value,
                'messages': [{
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': prompt},
                        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
                    ],
                }],
                'temperature': 0.1,
            }
            headers = {'Authorization': f'Bearer {key}', 'Content-Type': 'application/json'}
            resp = requests.post(str(self.get_parameter('openrouter_base_url').value), headers=headers, json=payload, timeout=30)
            resp.raise_for_status()
            data = resp.json()
            text = data['choices'][0]['message']['content']
            if isinstance(text, list):
                text = ' '.join(str(x.get('text', '')) for x in text if isinstance(x, dict))
            return self.extract_json(str(text).strip())
        except Exception as exc:
            self.publish_status(f'VLM call failed: {exc}')
            return None

    def vlm_label_current_view(self) -> Optional[dict]:
        existing = [asdict(p) for p in list(self.place_store.places.values())[:12]]
        prompt = (
            'Look at this indoor robot camera view and return ONLY JSON with keys: '
            'label, room, category, aliases, tags, description, confidence. '
            'Use human-friendly snake_case labels and keep names stable with prior places when appropriate. '
            f'Existing places for naming context: {json.dumps(existing)}'
        )
        meta = self._call_openrouter(prompt)
        if not meta:
            return None
        meta['label'] = slugify(str(meta.get('label', '')))
        meta['room'] = slugify(str(meta.get('room', '')))
        meta['category'] = slugify(str(meta.get('category', '')))
        meta['aliases'] = [slugify(str(x)) for x in meta.get('aliases', []) if slugify(str(x))]
        meta['tags'] = [slugify(str(x)) for x in meta.get('tags', []) if slugify(str(x))]
        meta['description'] = str(meta.get('description', '')).strip()
        try:
            meta['confidence'] = float(meta.get('confidence', 0.0))
        except Exception:
            meta['confidence'] = 0.0
        if not meta['label']:
            meta['label'] = '_'.join([x for x in [meta['room'], meta['category']] if x]) or 'labeled_place'
        return meta

    def auto_save_tick(self) -> None:
        pose = self.lookup_current_pose()
        if pose is None:
            return
        self.publish_status('auto_save_tick: running')
        if self.last_auto_save_xy is not None:
            dx = pose.pose.position.x - self.last_auto_save_xy[0]
            dy = pose.pose.position.y - self.last_auto_save_xy[1]
            if math.hypot(dx, dy) < float(self.get_parameter('auto_save_min_distance_m').value):
                return
        if not bool(self.get_parameter('auto_save_use_vlm').value):
            return
        meta = self.vlm_label_current_view()
        if not meta:
            self.publish_status('auto_save_tick: vlm returned no label')
            return
        if float(meta.get('confidence', 0.0)) < float(self.get_parameter('auto_save_min_confidence').value):
            self.publish_status(f'vlm_low_confidence skipped confidence={float(meta.get("confidence", 0.0)):.2f}')
            return
        name = self.choose_place_name(meta, pose)
        place = self.make_place(name, pose, meta, source='vlm_auto', confidence=float(meta.get('confidence', 1.0)))
        near = self.place_store.nearest_within(place.x, place.y, float(self.get_parameter('auto_save_merge_distance_m').value))
        if near and near.name == name:
            place.name = near.name
        self.save_place(place, source_text='vlm_auto')
        self.last_auto_save_xy = (place.x, place.y)
        self.publish_status(
            f'vlm_labeled_place name={place.name} confidence={place.confidence:.2f} room={place.room or "-"} '
            f'category={place.category or "-"} tags={place.tags} desc={place.description or "-"}'
        )

    def describe(self) -> None:
        meta = self.vlm_label_current_view()
        if not meta:
            return
        self.publish_status(f'describe: {meta.get("description", "") or meta.get("label", "")}'.strip())

    def resolve_place(self, query: str) -> Optional[Place]:
        exact = self.place_store.get(query)
        if exact:
            return exact
        best, score, why = self.memory.resolve(query, list(self.place_store.places.values()))
        if best and score >= 20.0:
            self.publish_status(f'semantic_match query={query} -> {best.name} score={score:.1f} why={why}')
            return best
        return None

    def list_synonyms(self, token: str) -> None:
        expanded = self.memory.expand_tokens(token)
        self.publish_status(f'synonyms {token}: {expanded}')

    def list_related(self, name: str) -> None:
        p = self.resolve_place(name)
        if not p:
            self.publish_status(f'place not found: {name}')
            return
        graph = self.memory.build_relationships(list(self.place_store.places.values()))
        self.publish_status(f'related {p.name}: {graph.get(p.name, {})}')

    def send_goal(self, place: Place) -> None:
        self._goal_place = place
        self._last_goal_failed = False
        pose = self.build_pose(place)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.publish_status(f'sending_goal {place.name} -> ({place.x:.2f}, {place.y:.2f}, yaw={place.yaw:.2f})')
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('navigate_to_pose server not available')
            return
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', 0.0)
        self.publish_status(f'navigating target={self._goal_place.name if self._goal_place else "-"} distance_remaining={dist:.2f}')

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status('goal rejected')
            self._last_goal_failed = True
            self.try_fallback_recovery('rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future) -> None:
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('goal succeeded')
            self._fallback_attempts = 0
            self._last_goal_failed = False
        else:
            self.publish_status(f'goal failed status={status}')
            self._last_goal_failed = True
            self.try_fallback_recovery(f'status_{status}')

    def scan_clear(self, forward: bool = True) -> bool:
        if self.latest_scan is None:
            return True
        ranges = [r for r in self.latest_scan.ranges if math.isfinite(r)]
        if not ranges:
            return True
        threshold = 0.55 if forward else 0.35
        return min(ranges[:20] + ranges[-20:]) > threshold

    def classify_failure_with_vlm(self, reason: str) -> dict:
        if not self.latest_image_bytes:
            return {'cause': 'unknown', 'action': 'rotate_left', 'confidence': 0.2}
        goal = asdict(self._goal_place) if self._goal_place else {}
        existing = [asdict(p) for p in list(self.place_store.places.values())[:12]]
        prompt = (
            'A mobile robot failed to reach its goal. Return ONLY JSON with keys: '
            'cause, action, confidence, alternate_query, note. '
            'Allowed actions: rotate_left, rotate_right, backup, creep_forward, retry_nav, alternate_place, stop. '
            f'Failure reason={reason}. Goal={json.dumps(goal)}. Existing places={json.dumps(existing)}.'
        )
        meta = self._call_openrouter(prompt)
        if not meta:
            return {'cause': 'unknown', 'action': 'rotate_left', 'confidence': 0.2}
        meta['action'] = slugify(str(meta.get('action', 'rotate_left')))
        meta['cause'] = slugify(str(meta.get('cause', 'unknown')))
        meta['alternate_query'] = str(meta.get('alternate_query', '')).strip()
        try:
            meta['confidence'] = float(meta.get('confidence', 0.0))
        except Exception:
            meta['confidence'] = 0.0
        return meta

    def start_motion(self, linear_x: float, angular_z: float, duration_sec: float, retry_place: Optional[Place]) -> None:
        self._motion_active = True
        self._motion_twist = Twist()
        self._motion_twist.linear.x = float(linear_x)
        self._motion_twist.angular.z = float(angular_z)
        self._motion_end_ns = self.get_clock().now().nanoseconds + int(duration_sec * 1e9)
        self._motion_retry_place = retry_place

    def motion_tick(self) -> None:
        if not self._motion_active:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self._motion_end_ns:
            self.cmd_vel_pub.publish(Twist())
            self._motion_active = False
            if self._motion_retry_place is not None:
                place = self._motion_retry_place
                self._motion_retry_place = None
                self.send_goal(place)
            return
        self.cmd_vel_pub.publish(self._motion_twist)

    def try_fallback_recovery(self, reason: str) -> None:
        if not bool(self.get_parameter('fallback_enable').value):
            return
        if self._goal_place is None:
            return
        if self._fallback_attempts >= int(self.get_parameter('fallback_max_attempts').value):
            self.publish_status('fallback_exhausted')
            return
        self._fallback_attempts += 1
        meta = self.classify_failure_with_vlm(reason)
        action = meta.get('action', 'rotate_left')
        self.publish_status(f'fallback_decision attempt={self._fallback_attempts} cause={meta.get("cause", "unknown")} action={action} conf={meta.get("confidence", 0.0):.2f}')
        lin = float(self.get_parameter('fallback_linear_speed').value)
        ang = float(self.get_parameter('fallback_angular_speed').value)
        forward_d = float(self.get_parameter('fallback_forward_distance').value)
        backup_d = float(self.get_parameter('fallback_backup_distance').value)
        rotate_rad = math.radians(float(self.get_parameter('fallback_rotate_deg').value))
        if action == 'creep_forward':
            if not self.scan_clear(True):
                self.publish_status('fallback_creep_forward_blocked_by_scan')
                return
            self.start_motion(lin, 0.0, max(0.1, forward_d / max(lin, 1e-3)), self._goal_place)
        elif action == 'backup':
            self.start_motion(-lin, 0.0, max(0.1, backup_d / max(lin, 1e-3)), self._goal_place)
        elif action == 'rotate_right':
            self.start_motion(0.0, -ang, max(0.1, rotate_rad / max(ang, 1e-3)), self._goal_place)
        elif action == 'alternate_place':
            alt_query = meta.get('alternate_query', '')
            alt = self.resolve_place(alt_query) if alt_query else None
            if alt is None:
                graph = self.memory.build_relationships(list(self.place_store.places.values()))
                rel = graph.get(self._goal_place.name, {})
                for cand_name in rel.get('near', []) + rel.get('same_room', []) + rel.get('same_category', []):
                    alt = self.place_store.get(cand_name)
                    if alt:
                        break
            if alt is None:
                self.publish_status('fallback_alternate_place_not_found')
                return
            self.publish_status(f'fallback_alternate_place {self._goal_place.name} -> {alt.name}')
            self._goal_place = alt
            self.send_goal(alt)
        elif action == 'stop':
            self.cmd_vel_pub.publish(Twist())
            self.publish_status('fallback_stop')
        else:
            self.start_motion(0.0, ang, max(0.1, rotate_rad / max(ang, 1e-3)), self._goal_place)

    def cmd_cb(self, msg: String) -> None:
        line = msg.data.strip()
        if not line:
            return
        parts = line.split()
        cmd = parts[0].lower()
        if cmd == 'status':
            self.publish_status('status_ok')
            return
        if cmd == 'save' and len(parts) >= 2:
            pose = self.lookup_current_pose()
            if pose is None:
                return
            place = self.make_place(parts[1], pose, self.parse_kv(parts[2:]), source='manual')
            self.save_place(place, source_text='manual')
            return
        if cmd == 'goal-save' and len(parts) >= 2:
            if self.latest_goal is None:
                self.publish_status('No cached /goal_pose yet.')
                return
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = self.latest_goal.pose
            place = self.make_place(parts[1], pose, self.parse_kv(parts[2:]), source='goal_pose')
            self.save_place(place, source_text='goal_pose')
            return
        if cmd == 'places':
            lines = self.place_store.list_lines()
            if not lines:
                self.publish_status('places: none')
            for line in lines:
                self.publish_status(line)
            return
        if cmd in {'go', 'navigate'} and len(parts) >= 2:
            query = ' '.join(parts[1:]).replace('to ', '').strip()
            place = self.resolve_place(query)
            if not place:
                self.publish_status(f'place not found: {query}')
                return
            self.send_goal(place)
            return
        if cmd == 'near' and len(parts) >= 2:
            query = 'near ' + ' '.join(parts[1:])
            place = self.resolve_place(query)
            if not place:
                self.publish_status(f'place not found: {query}')
                return
            self.send_goal(place)
            return
        if cmd == 'describe':
            self.describe()
            return
        if cmd == 'synonyms' and len(parts) >= 2:
            self.list_synonyms(' '.join(parts[1:]))
            return
        if cmd == 'related' and len(parts) >= 2:
            self.list_related(' '.join(parts[1:]))
            return
        if cmd == 'cancel':
            self.publish_status('cancel not implemented in this build')
            return
        self.publish_status(f'unknown command: {line}')

    def save_map_snapshot(self) -> None:
        cmd = str(self.get_parameter('map_saver_cmd').value)
        if not cmd or not bool(self.get_parameter('save_map_on_shutdown').value):
            return
        prefix = self.session.map_prefix
        try:
            subprocess.run(shlex.split(cmd) + ['-f', prefix], check=True, timeout=90)
            meta = self.session_store.load_session_yaml(self.session)
            meta.update({'session_name': self.session.session_name, 'map_yaml': prefix + '.yaml'})
            meta.setdefault('semantic_graph', self.memory.build_relationships(list(self.place_store.places.values())))
            self.session_store.save_session_yaml(self.session, meta)
            self.publish_status(f'saved_map yaml={prefix}.yaml')
        except Exception as exc:
            self.publish_status(f'map_save_failed: {exc}')

    def on_shutdown(self) -> None:
        try:
            self.place_store.save()
            meta = self.session_store.load_session_yaml(self.session)
            meta['semantic_graph'] = self.memory.build_relationships(list(self.place_store.places.values()))
            self.session_store.save_session_yaml(self.session, meta)
            if self.mode == 'teach':
                self.save_map_snapshot()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
