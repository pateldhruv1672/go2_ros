from __future__ import annotations

import atexit
import base64
import io
import json
import math
import os
import re
import zlib
import shlex
import subprocess
from dataclasses import asdict
from typing import Optional

import numpy as np
from PIL import Image as PILImage
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
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

from go2_agentic_system.openrouter_client import OpenRouterClient
from go2_agentic_system.patrol_events import PatrolEvent
from go2_agentic_system.storage import MemoryStore as SharedMemoryStore

from .place_store import Place, PlaceStore
from .route_store import RoutePlan, RouteStop, RouteStore, slugify as route_slugify
from .session_store import SessionStore
from .semantic_memory import SemanticMemory, slugify


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def stable_marker_id(name: str, salt: int = 0) -> int:
    payload = f'{name}:{salt}'.encode('utf-8', errors='ignore')
    return int(zlib.crc32(payload) & 0x7fffffff)


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
        for _p in self.place_store.places.values():
            _p.frame_id = (getattr(_p, 'frame_id', '') or self.map_frame or 'map').strip() or 'map'
        self.memory = SemanticMemory()
        self.agent_memory = SharedMemoryStore(str(self.get_parameter('agent_memory_root').value))
        self.openrouter = OpenRouterClient(
            model=str(self.get_parameter('openrouter_model').value),
            base_url=str(self.get_parameter('openrouter_base_url').value),
        )
        self.route_store = RouteStore(self.session.session_dir)
        self.route = self._load_or_create_route()
        self._last_status = ''
        self._rehydrate_places_from_memory()
        self._normalize_route_stops()
        if self.mode == 'teach' and bool(self.get_parameter('clear_places_on_start').value):
            self.place_store.places = {}
            self.place_store.save()
            self.route = RoutePlan(name=route_slugify(self.session.session_name), mode='teach')
            self.route_store.save(self.route)

        self.latest_image_bytes: Optional[bytes] = None
        self.latest_scan: Optional[LaserScan] = None
        self._latest_scan_received_ns = 0
        self.latest_goal: Optional[PoseStamped] = None
        self.last_auto_save_xy: Optional[tuple[float, float]] = None
        self._goal_place: Optional[Place] = None
        self._fallback_attempts = 0
        self._last_goal_failed = False
        self._goal_send_retry_ns = 0
        self._goal_send_retry_count = 0
        self._restore_count = 0
        self._restore_done = False
        self._restore_spawn_last_publish_ns = 0
        self._restore_spawn_verified = False
        self._restore_spawn_last_target: Optional[dict] = None
        self._restore_next_attempt_ns = 0
        self._spawn_persisted = False
        self._amcl_active = False
        self._amcl_state_client = self.create_client(GetState, '/amcl/get_state')
        self._amcl_state_last_check_ns = 0
        self._amcl_state_check_pending = False
        self._motion_active = False
        self._motion_twist = Twist()
        self._motion_end_ns = 0
        self._motion_retry_place: Optional[Place] = None
        self._motion_retry_route_goal = False
        self._tour_pause_until_ns = 0
        self._stop_announced: set[str] = set()
        self._recovery_retries = 0
        self._route_goal_active = False
        self._active_goal_handle = None
        self._pending_recovery_goal: Optional[Place] = None
        self._teach_auto_save_count = sum(1 for p in self.place_store.places.values() if getattr(p, 'source', '') == 'vlm_auto')

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
        self.reply_pub = self.create_publisher(String, '/agent/reply', 20)
        self.event_pub = self.create_publisher(String, '/semantic_nav/event', status_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_nav/places_markers', marker_qos)
        self.preview_pub = self.create_publisher(MarkerArray, '/semantic_nav/route_preview', marker_qos)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, str(self.get_parameter('fallback_cmd_topic').value), cmd_qos)

        self.create_subscription(String, '/semantic_nav/command', self.cmd_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.create_subscription(Image, str(self.get_parameter('camera_image_topic').value), self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, str(self.get_parameter('camera_compressed_topic').value), self.compressed_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self.scan_cb, qos_profile_sensor_data)

        self.create_timer(1.0, self.publish_markers)
        self.create_timer(0.05, self.motion_tick)
        self.create_timer(0.5, self.route_tick)
        self.auto_save_timer = None
        if self.mode == 'teach' and bool(self.get_parameter('auto_save_places').value):
            self.auto_save_timer = self.create_timer(float(self.get_parameter('auto_save_interval_sec').value), self.auto_save_tick)
        if self.mode == 'teach' and bool(self.get_parameter('save_spawn_on_start').value):
            self.create_timer(0.5, self.persist_spawn_when_tf_ready)
        if self.mode == 'resume' and bool(self.get_parameter('restore_spawn_on_start').value):
            self.create_timer(0.5, self.restore_spawn_when_tf_ready)

        atexit.register(self.on_shutdown)
        self.publish_status(
            f'ready | mode={self.mode} | session={self.session.session_name} | '
            f'places_file={self.session.places_path} | places_loaded={len(self.place_store.places)} | '
            f'route={self.route.name} | route_mode={self.route.mode} | route_state={self.route.state} | '
            f'stops={len(self.route.stops)} | tour_mode={"on" if bool(self.get_parameter("tour_mode").value) else "off"} | '
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
        self.declare_parameter('agent_memory_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('save_map_on_shutdown', True)
        self.declare_parameter('map_saver_cmd', 'ros2 run nav2_map_server map_saver_cli')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_compressed_topic', '/camera/image_raw/compressed')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('openrouter_model', 'google/gemini-2.5-flash')
        self.declare_parameter('openrouter_base_url', 'https://openrouter.ai/api/v1/chat/completions')
        self.declare_parameter('tour_mode', True)
        self.declare_parameter('tour_default_pause_sec', 4.0)
        self.declare_parameter('route_name', '')
        self.declare_parameter('auto_save_places', True)
        self.declare_parameter('auto_save_interval_sec', 5.0)
        self.declare_parameter('auto_save_use_vlm', True)
        self.declare_parameter('auto_save_min_distance_m', 1.5)
        self.declare_parameter('auto_save_merge_distance_m', 1.5)
        self.declare_parameter('route_stop_merge_distance_m', 0.25)
        self.declare_parameter('auto_save_target_samples', 0)
        self.declare_parameter('auto_save_allow_repeat_samples', False)
        self.declare_parameter('auto_save_min_confidence', 0.55)
        self.declare_parameter('clear_places_on_start', False)
        self.declare_parameter('restore_spawn_on_start', True)
        self.declare_parameter('allow_manual_initialpose_override', True)
        self.declare_parameter('restore_spawn_retry_interval_sec', 5.0)
        self.declare_parameter('restore_spawn_position_tolerance_m', 0.75)
        self.declare_parameter('restore_spawn_yaw_tolerance_rad', 0.75)
        self.declare_parameter('restore_spawn_position_covariance', 0.5)
        self.declare_parameter('restore_spawn_yaw_covariance', 0.5)
        self.declare_parameter('save_spawn_on_start', True)
        self.declare_parameter('fallback_enable', True)
        self.declare_parameter('fallback_max_attempts', 3)
        self.declare_parameter('fallback_cmd_topic', '/cmd_vel')
        self.declare_parameter('fallback_linear_speed', 0.08)
        self.declare_parameter('fallback_angular_speed', 0.35)
        self.declare_parameter('fallback_forward_distance', 0.25)
        self.declare_parameter('fallback_backup_distance', 0.18)
        self.declare_parameter('fallback_rotate_deg', 18.0)
        self.declare_parameter('tf_max_age_sec', 5.0)
        self.declare_parameter('recovery_max_retries', 3)
        self.declare_parameter('restore_spawn_only_once', True)

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

    def _load_or_create_route(self) -> RoutePlan:
        route_name = str(self.get_parameter('route_name').value).strip() or self.session.session_name
        stops = []
        for p in sorted(self.place_store.places.values(), key=lambda x: x.name):
            stops.append({
                'stop_name': p.name,
                'place_name': p.name,
                'script': p.description or '',
                'fact': p.description or '',
                'pause_seconds': float(self.get_parameter('tour_default_pause_sec').value),
                'safe_anchor': 'spawn' if p.category == 'spawn' or p.name == 'spawn' else '',
                'kind': 'tour' if bool(self.get_parameter('tour_mode').value) else self.mode,
                'confidence': float(getattr(p, 'confidence', 1.0) or 1.0),
                'aliases': list(getattr(p, 'aliases', []) or []),
                'tags': list(getattr(p, 'tags', []) or []),
            })
        route = self.route_store.load_or_create(name=route_name, mode='tour' if bool(self.get_parameter('tour_mode').value) else self.mode, stop_candidates=stops)
        if not route.stops and self.place_store.places:
            for p in sorted(self.place_store.places.values(), key=lambda x: x.name):
                stop = RouteStop(
                    name=route_slugify(p.name),
                    place_name=p.name,
                    script=p.description or '',
                    fact=p.description or '',
                    pause_seconds=float(self.get_parameter('tour_default_pause_sec').value),
                    safe_anchor='spawn' if p.name == 'spawn' else '',
                    kind='tour',
                    confidence=float(getattr(p, 'confidence', 1.0) or 1.0),
                    aliases=list(getattr(p, 'aliases', []) or []),
                    tags=list(getattr(p, 'tags', []) or []),
                )
                self.route_store.upsert_stop(route, stop)
        if route.mode not in {'tour', 'patrol', 'resume', 'teach'}:
            route.mode = 'tour' if bool(self.get_parameter('tour_mode').value) else self.mode
        if self.mode == 'resume':
            route.mode = 'resume'
            if route.state in {'moving_to_stop', 'touring', 'recovering', 'blocked'}:
                route.state = 'paused'
        self.route_store.save(route)
        return route

    def _rehydrate_places_from_memory(self) -> None:
        if self.mode != 'resume' or self.place_store.places:
            return
        payload = self.agent_memory.list_places(self.route.name)
        places = dict((payload or {}).get('places') or {})
        if not places:
            return
        restored = 0
        for name, item in places.items():
            pose = item or {}
            try:
                place = Place(
                    name=str(name),
                    x=float(pose.get('x', 0.0)),
                    y=float(pose.get('y', 0.0)),
                    yaw=float(pose.get('yaw', 0.0)),
                    room=str(pose.get('room', '')),
                    category=str(pose.get('category', '')),
                    aliases=[str(x) for x in (pose.get('aliases') or [])],
                    tags=[str(x) for x in (pose.get('tags') or [])],
                    description=str(pose.get('description', '')),
                    summary=str(pose.get('summary', '')),
                    tour_fact=str(pose.get('tour_fact', '')),
                    navigation_hint=str(pose.get('navigation_hint', '')),
                    resume_hook=str(pose.get('resume_hook', '')),
                    safety_notes=str(pose.get('safety_notes', '')),
                    scene_context=str(pose.get('scene_context', '')),
                    capture_kind=str(pose.get('capture_kind', '')),
                    sample_index=int(pose.get('sample_index', 0) or 0),
                    sample_group=str(pose.get('sample_group', '')),
                    captured_at=str(pose.get('captured_at', '')),
                    confidence=float(pose.get('confidence', 1.0) or 1.0),
                    source=str(pose.get('source', 'event_memory')),
                    frame_id=str(pose.get('frame_id', self.map_frame) or self.map_frame),
                )
            except Exception:
                continue
            self.place_store.upsert(place)
            restored += 1
        if restored:
            self.place_store.save()
            self.publish_status(f'restored_places_from_memory count={restored}')

    def _normalize_route_stops(self) -> None:
        if not self.route.stops:
            return
        merge_radius = float(self.get_parameter('route_stop_merge_distance_m').value)
        if merge_radius <= 0.0:
            return
        unique_stops: List[RouteStop] = []
        unique_places: List[Optional[Place]] = []
        changed = False
        for stop in self.route.stops:
            place = self._resolve_stop_place(stop)
            duplicate = False
            if place is not None:
                for kept_place in unique_places:
                    if kept_place is None:
                        continue
                    if math.hypot(place.x - kept_place.x, place.y - kept_place.y) <= merge_radius:
                        duplicate = True
                        break
            if duplicate:
                changed = True
                continue
            unique_stops.append(stop)
            unique_places.append(place)
        if changed:
            self.route.stops = unique_stops
            if self.route.stops:
                self.route.current_stop_index = max(0, min(self.route.current_stop_index, len(self.route.stops) - 1))
            else:
                self.route.current_stop_index = 0
            self.route_store.save(self.route)
            self.publish_status(f'route_normalized merged_duplicates kept={len(self.route.stops)}')

    def publish_status(self, text: str) -> None:
        if text == self._last_status:
            return
        self._last_status = text
        msg = String(); msg.data = text
        if hasattr(self, 'status_pub') and self.status_pub is not None:
            self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_reply(self, text: str) -> None:
        if not text:
            return
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)
        self.get_logger().info(f'reply: {text}')

    def publish_event(self, event: PatrolEvent) -> None:
        payload = event.to_dict()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)
        self.route_store.append_event(event)
        self.agent_memory.log_event('patrol_event', payload)
        if event.speech:
            self.publish_reply(event.speech)
        if event.status:
            self.publish_status(event.status)

    def goal_cb(self, msg: PoseStamped) -> None:
        if not (msg.header.frame_id or '').strip():
            msg.header.frame_id = self.map_frame
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
        self._latest_scan_received_ns = self.get_clock().now().nanoseconds

    def scan_ready_for_initialpose(self) -> bool:
        if self.latest_scan is None:
            return False
        max_age_sec = max(0.5, float(self.get_parameter('tf_max_age_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self._latest_scan_received_ns) <= int(max_age_sec * 1e9)

    def scan_summary(self) -> dict:
        if self.latest_scan is None:
            return {'available': False}
        ranges = [float(r) for r in self.latest_scan.ranges if math.isfinite(r) and float(r) > 0.0]
        if not ranges:
            return {'available': True, 'range_count': 0}
        front = ranges[:20]
        rear = ranges[-20:]
        return {
            'available': True,
            'range_count': len(ranges),
            'min_range': min(ranges),
            'mean_range': float(sum(ranges) / len(ranges)),
            'front_clear': min(front) > 0.75 if front else True,
            'rear_clear': min(rear) > 0.45 if rear else True,
        }

    def pose_summary(self) -> dict:
        pose = self.lookup_current_pose()
        if pose is None:
            return {'available': False}
        return {
            'available': True,
            'x': float(pose.pose.position.x),
            'y': float(pose.pose.position.y),
            'frame_id': pose.header.frame_id or self.map_frame,
        }

    def odom_tf_ready(self) -> bool:
        try:
            self.buffer.lookup_transform('odom', self.base_frame, Time(), timeout=Duration(seconds=0.2))
            return True
        except TransformException:
            return False

    def amcl_ready_for_initialpose(self) -> bool:
        if self._amcl_active:
            return True
        now_ns = self.get_clock().now().nanoseconds
        if self._amcl_state_check_pending or now_ns < self._amcl_state_last_check_ns:
            return False
        if not self._amcl_state_client.service_is_ready():
            self.publish_status('waiting_for_amcl_lifecycle_service')
            self._amcl_state_last_check_ns = now_ns + int(1.0 * 1e9)
            return False
        future = self._amcl_state_client.call_async(GetState.Request())
        self._amcl_state_check_pending = True
        self._amcl_state_last_check_ns = now_ns + int(1.0 * 1e9)
        future.add_done_callback(self._amcl_state_cb)
        return self._amcl_active

    def _amcl_state_cb(self, future) -> None:
        self._amcl_state_check_pending = False
        try:
            response = future.result()
            state_id = int(getattr(getattr(response, 'current_state', None), 'id', 0))
            self._amcl_active = state_id == LifecycleState.PRIMARY_STATE_ACTIVE
        except Exception:
            self._amcl_active = False

    def tf_age_seconds(self, target_frame: str, source_frame: str) -> Optional[float]:
        try:
            transform = self.buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.4))
        except TransformException:
            return None
        stamp = getattr(transform, 'header', None)
        if stamp is None:
            return None
        try:
            stamp_ns = int(stamp.stamp.sec) * 1_000_000_000 + int(stamp.stamp.nanosec)
        except Exception:
            return None
        if stamp_ns <= 0:
            return None
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, float(now_ns - stamp_ns) / 1_000_000_000.0)

    def restore_spawn_when_tf_ready(self) -> None:
        """
        Resume-mode startup localization restore.

        We publish /initialpose once AMCL is active, then keep checking the
        resulting map->base_link pose until it settles near the saved spawn.
        If localization does not converge, we retry on a controlled cadence so
        resume mode can recover from a weak or ignored first initial pose.
        """
        if self._restore_done:
            return

        if not self.odom_tf_ready():
            self.publish_status('waiting_for_odom_tf_before_spawn_restore')
            return

        if not self.amcl_ready_for_initialpose():
            self.publish_status('waiting_for_amcl_active_before_spawn_restore')
            return

        meta = self.session_store.load_session_yaml(self.session)
        spawn = meta.get('spawn')

        if not spawn and self.route.safe_anchors:
            anchor_name = self.route.safe_anchors.get('spawn') or next(iter(self.route.safe_anchors.values()))
            anchor_place = self.resolve_place(anchor_name) or self.place_store.get(anchor_name)
            if anchor_place:
                spawn = {
                    'x': anchor_place.x,
                    'y': anchor_place.y,
                    'yaw': anchor_place.yaw,
                    'frame_id': anchor_place.frame_id or self.map_frame,
                }

        if not spawn:
            self.publish_status('resume_no_spawn_found')
            self._restore_done = True
            return

        if not self.scan_ready_for_initialpose():
            self.publish_status('scan_not_ready_before_spawn_restore_publishing_anyway')

        now_ns = self.get_clock().now().nanoseconds
        current_pose = self.lookup_current_pose()
        if current_pose is not None and self._restore_spawn_last_publish_ns > 0:
            matched, pose_dist, yaw_err = self.current_pose_matches_spawn(current_pose, spawn)
            if matched:
                self._restore_spawn_verified = True
                self._restore_done = True
                self.publish_status(
                    f'restored_spawn_verified name=spawn '
                    f'dist={pose_dist:.2f} yaw_err={yaw_err:.2f} '
                    f'count={self._restore_count}'
                )
                return
            self.publish_status(
                f'resume_pose_needs_refresh dist={pose_dist:.2f} yaw_err={yaw_err:.2f}'
            )

        retry_interval_ns = int(float(self.get_parameter('restore_spawn_retry_interval_sec').value) * 1e9)
        if self._restore_spawn_last_publish_ns and now_ns < self._restore_next_attempt_ns:
            return

        subscriber_count = 0
        try:
            subscriber_count = int(self.initialpose_pub.get_subscription_count())
        except Exception:
            subscriber_count = 0

        if subscriber_count <= 0:
            self.publish_status('waiting_for_initialpose_subscriber')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = (spawn.get('frame_id') or self.map_frame or 'map').strip() or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(spawn.get('x', 0.0))
        msg.pose.pose.position.y = float(spawn.get('y', 0.0))

        qx, qy, qz, qw = quaternion_from_yaw(float(spawn.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        pos_cov = max(0.05, float(self.get_parameter('restore_spawn_position_covariance').value))
        yaw_cov = max(0.05, float(self.get_parameter('restore_spawn_yaw_covariance').value))
        msg.pose.covariance[0] = pos_cov
        msg.pose.covariance[7] = pos_cov
        msg.pose.covariance[35] = yaw_cov

        self._restore_spawn_last_target = {
            'x': float(spawn.get('x', 0.0)),
            'y': float(spawn.get('y', 0.0)),
            'yaw': float(spawn.get('yaw', 0.0)),
            'frame_id': msg.header.frame_id,
        }
        self.initialpose_pub.publish(msg)
        self._restore_count += 1
        self._restore_spawn_last_publish_ns = now_ns
        self._restore_next_attempt_ns = now_ns + retry_interval_ns
        self._restore_spawn_verified = False

        self.publish_status(
            f'restored_spawn_initialpose name=spawn '
            f'x={msg.pose.pose.position.x:.2f} '
            f'y={msg.pose.pose.position.y:.2f} '
            f'yaw={float(spawn.get("yaw", 0.0)):.2f} '
            f'count={self._restore_count} subscribers={subscriber_count}'
        )

    def initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if self.mode != 'resume':
            return
        if not bool(self.get_parameter('allow_manual_initialpose_override').value):
            return

        spawn = self._restore_spawn_last_target
        if spawn is not None and self._restore_spawn_last_publish_ns > 0:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id or self.map_frame
            pose.pose = msg.pose.pose
            matched, _, _ = self.current_pose_matches_spawn(pose, spawn)
            if matched:
                return

        if self._restore_done:
            return

        pose_x = float(msg.pose.pose.position.x)
        pose_y = float(msg.pose.pose.position.y)
        pose_yaw = float(yaw_from_quat(msg.pose.pose.orientation))
        self._restore_done = True
        self._restore_spawn_verified = True
        self.publish_status(
            f'manual_initialpose_accepted from_rviz '
            f'x={pose_x:.2f} y={pose_y:.2f} yaw={pose_yaw:.2f}'
        )

    def current_pose_matches_spawn(self, current_pose: PoseStamped, spawn: dict) -> tuple[bool, float, float]:
        spawn_x = float(spawn.get('x', 0.0))
        spawn_y = float(spawn.get('y', 0.0))
        spawn_yaw = float(spawn.get('yaw', 0.0))
        dx = float(current_pose.pose.position.x) - spawn_x
        dy = float(current_pose.pose.position.y) - spawn_y
        pose_dist = math.hypot(dx, dy)
        pose_yaw = float(yaw_from_quat(current_pose.pose.orientation))
        yaw_err = abs(math.atan2(math.sin(pose_yaw - spawn_yaw), math.cos(pose_yaw - spawn_yaw)))
        pos_tol = max(0.05, float(self.get_parameter('restore_spawn_position_tolerance_m').value))
        yaw_tol = max(0.05, float(self.get_parameter('restore_spawn_yaw_tolerance_rad').value))
        return pose_dist <= pos_tol and yaw_err <= yaw_tol, pose_dist, yaw_err

    def persist_spawn_when_tf_ready(self) -> None:
        if self._spawn_persisted:
            return
        meta = self.session_store.load_session_yaml(self.session)
        if meta.get('spawn'):
            self._spawn_persisted = True
            return
        if not self.odom_tf_ready():
            self.publish_status('waiting_for_odom_tf_before_spawn_persist')
            return
        pose = self.lookup_current_pose()
        if pose is None:
            self.publish_status('waiting_for_current_pose_before_spawn_persist')
            return
        spawn = {
            'x': float(pose.pose.position.x),
            'y': float(pose.pose.position.y),
            'yaw': float(yaw_from_quat(pose.pose.orientation)),
            'frame_id': pose.header.frame_id or self.map_frame,
        }
        meta.update({'session_name': self.session.session_name, 'map_yaml': self.session.map_prefix + '.yaml'})
        meta['spawn'] = spawn
        meta.setdefault('semantic_graph', self.memory.build_relationships(list(self.place_store.places.values())))
        self.session_store.save_session_yaml(self.session, meta)
        self._spawn_persisted = True
        self.publish_status(
            f'saved_session_spawn name=spawn x={spawn["x"]:.2f} y={spawn["y"]:.2f} yaw={spawn["yaw"]:.2f} frame={spawn["frame_id"]}'
        )

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
        pose.header.frame_id = (getattr(place, 'frame_id', '') or self.map_frame or 'map').strip() or 'map'
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
            arrow.id = stable_marker_id(p.name, 0)
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
            label.id = stable_marker_id(p.name, 1)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.5
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = 0.60
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.text = p.name
            arr.markers.append(label)
        self.marker_pub.publish(arr)

        route_arr = MarkerArray()
        for idx, stop in enumerate(self.route.stops):
            place = self._resolve_stop_place(stop)
            if place is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = t
            marker.ns = 'route_stops'
            marker.id = stable_marker_id(stop.name, 10)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.22
            marker.scale.y = 0.22
            marker.scale.z = 0.22
            marker.pose = self.build_pose(place).pose
            marker.color.a = 0.9
            if idx == self.route.current_stop_index:
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.25, 0.2)
            elif stop.status == 'complete':
                marker.color.r, marker.color.g, marker.color.b = (0.2, 0.95, 0.3)
            elif stop.status in {'paused', 'blocked', 'recovering'}:
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.65, 0.1)
            else:
                marker.color.r, marker.color.g, marker.color.b = (0.35, 0.65, 1.0)
            route_arr.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = t
            label.ns = 'route_stop_labels'
            label.id = stable_marker_id(stop.name, 11)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.4
            label.pose.position.x = place.x
            label.pose.position.y = place.y
            label.pose.position.z = 0.86
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 0.9
            label.color.b = 0.3
            label.text = f'{idx + 1}. {stop.name} [{stop.status}]'
            route_arr.markers.append(label)
        self.preview_pub.publish(route_arr)

    def nearest_route_stop(self, *, include_complete: bool = False) -> Optional[tuple[int, RouteStop]]:
        pose = self.lookup_current_pose()
        if pose is None or not self.route.stops:
            return None
        best_idx: Optional[int] = None
        best_stop: Optional[RouteStop] = None
        best_dist = float('inf')
        for idx, stop in enumerate(self.route.stops):
            if not include_complete and stop.status == 'complete':
                continue
            place = self._resolve_stop_place(stop)
            if place is None:
                continue
            dist = math.hypot(pose.pose.position.x - place.x, pose.pose.position.y - place.y)
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
                best_stop = stop
        if best_idx is None or best_stop is None:
            return None
        self.route.current_stop_index = best_idx
        self.route_store.save(self.route)
        self.publish_status(f'resume_selected_stop index={best_idx} stop={best_stop.name} dist={best_dist:.2f}')
        return best_idx, best_stop

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
        if place.name.lower() == 'spawn' or place.category.lower() == 'spawn':
            meta['spawn'] = {'x': place.x, 'y': place.y, 'yaw': place.yaw, 'frame_id': self.map_frame}
            self._spawn_persisted = True
        self.session_store.save_session_yaml(self.session, meta)

    def _place_memory_payload(self, place: Place) -> dict:
        confidence = getattr(place, 'confidence', 1.0)
        try:
            confidence = float(confidence)
        except Exception:
            confidence = 1.0
        return {
            'x': float(place.x),
            'y': float(place.y),
            'yaw': float(place.yaw),
            'frame_id': place.frame_id or self.map_frame,
            'summary': place.summary or place.description or '',
            'description': place.description or '',
            'tour_fact': place.tour_fact or place.summary or place.description or '',
            'navigation_hint': place.navigation_hint or '',
            'resume_hook': place.resume_hook or '',
            'safety_notes': place.safety_notes or '',
            'scene_context': place.scene_context or '',
            'capture_kind': place.capture_kind or '',
            'sample_index': int(place.sample_index or 0),
            'sample_group': place.sample_group or '',
            'captured_at': place.captured_at or '',
            'aliases': list(place.aliases or []),
            'tags': list(place.tags or []),
            'confidence': confidence,
            'source': place.source,
        }

    def save_place(self, place: Place, source_text: str = 'manual') -> None:
        self.place_store.upsert(place)
        self.place_store.save()
        self.persist_spawn_if_needed(place)
        memory_payload = self._place_memory_payload(place)
        self.agent_memory.remember_place(self.route.name, place.name, memory_payload)
        self.agent_memory.add_observation({
            'id': f'{route_slugify(self.route.name)}_{route_slugify(place.name)}_{place.sample_index or len(self.place_store.places)}',
            'map_name': self.route.name,
            'label': place.name,
            'summary': place.summary or place.description or place.tour_fact or place.name,
            'tour_fact': place.tour_fact or place.summary or place.description or '',
            'navigation_hint': place.navigation_hint or '',
            'resume_hook': place.resume_hook or '',
            'safety_notes': place.safety_notes or '',
            'scene_context': place.scene_context or '',
            'pose': {'x': place.x, 'y': place.y, 'yaw': place.yaw, 'frame_id': place.frame_id or self.map_frame},
            'aliases': list(place.aliases or []),
            'objects': list(place.tags or []),
            'sample_index': int(place.sample_index or 0),
            'sample_group': place.sample_group or '',
            'source': place.source,
        })
        self.agent_memory.add_voxel_snapshot({
            'map_name': self.route.name,
            'place_name': place.name,
            'sample_index': int(place.sample_index or 0),
            'front_clear': self.scan_summary().get('front_clear'),
            'rear_clear': self.scan_summary().get('rear_clear'),
            'min_range': self.scan_summary().get('min_range'),
            'frame_id': place.frame_id or self.map_frame,
        })
        if place.name == 'spawn' or place.category == 'spawn':
            self.route_store.ensure_safe_anchor(self.route, 'spawn', place.name)
        if bool(self.get_parameter('tour_mode').value):
            confidence = getattr(place, 'confidence', 1.0)
            try:
                confidence = float(confidence)
            except Exception:
                confidence = 1.0
            stop = RouteStop(
                name=route_slugify(place.name),
                place_name=place.name,
                script=place.tour_fact or place.summary or place.description or '',
                fact=place.summary or place.description or place.tour_fact or '',
                navigation_hint=place.navigation_hint or '',
                resume_hook=place.resume_hook or '',
                safety_notes=place.safety_notes or '',
                scene_context=place.scene_context or '',
                capture_kind=place.capture_kind or '',
                sample_index=int(place.sample_index or 0),
                sample_group=place.sample_group or '',
                pause_seconds=float(self.get_parameter('tour_default_pause_sec').value),
                safe_anchor='spawn' if place.name == 'spawn' or place.category == 'spawn' else '',
                kind='tour',
                confidence=confidence,
                aliases=list(getattr(place, 'aliases', []) or []),
                tags=list(getattr(place, 'tags', []) or []),
            )
            self.route_store.upsert_stop(self.route, stop)
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
            summary=extra.get('summary', '') or extra.get('description', ''),
            tour_fact=extra.get('tour_fact', ''),
            navigation_hint=extra.get('navigation_hint', ''),
            resume_hook=extra.get('resume_hook', ''),
            safety_notes=extra.get('safety_notes', ''),
            scene_context=extra.get('scene_context', ''),
            capture_kind=extra.get('capture_kind', ''),
            sample_index=int(extra.get('sample_index', 0) or 0),
            sample_group=extra.get('sample_group', ''),
            captured_at=str(extra.get('captured_at', '')),
            confidence=float(confidence),
            source=source,
            frame_id=(pose.header.frame_id or self.map_frame or 'map').strip() or 'map',
        )

    def choose_place_name(self, meta: dict, pose: PoseStamped) -> str:
        # use previous labels as context to preserve naming stability
        label = slugify(str(meta.get('label', '')))
        room = slugify(str(meta.get('room', '')))
        category = slugify(str(meta.get('category', '')))
        capture_kind = slugify(str(meta.get('capture_kind', '') or meta.get('sample_kind', '') or ''))
        candidates = [c for c in [label, f'{label}_{capture_kind}' if label and capture_kind else '', f'{room}_{category}' if room and category else '', room, category, capture_kind] if c and c != 'unknown']
        base = candidates[0] if candidates else 'labeled_place'
        near = self.place_store.nearest_within(float(pose.pose.position.x), float(pose.pose.position.y), float(self.get_parameter('auto_save_merge_distance_m').value))
        if near:
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

    def _call_openrouter(self, prompt: str, *, max_tokens: int = 350) -> Optional[dict]:
        if not self.latest_image_bytes:
            self.publish_status('No camera frame received yet.')
            return None
        if not self.openrouter.available:
            self.publish_status('VLM is disabled or OPENROUTER_API_KEY is missing.')
            return None
        try:
            b64 = base64.b64encode(self.latest_image_bytes).decode('ascii')
            result = self.openrouter.complete_json(
                prompt,
                image_data_urls=[f'data:image/jpeg;base64,{b64}'],
                temperature=0.1,
                max_tokens=max_tokens,
                default={},
            )
            if not result.get('ok'):
                self.publish_status(f'VLM call failed: {result.get("error")}')
                return None
            parsed = result.get('parsed')
            if isinstance(parsed, dict):
                return parsed
        except Exception as exc:
            self.publish_status(f'VLM call failed: {exc}')
        return None

    def vlm_label_current_view(self) -> Optional[dict]:
        existing = [
            {
                'name': p.name,
                'room': p.room,
                'category': p.category,
                'capture_kind': getattr(p, 'capture_kind', ''),
            }
            for p in list(self.place_store.places.values())[:12]
        ]
        prompt = (
            'Look at this indoor robot camera view and return ONLY JSON with keys: '
            'label, room, category, aliases, tags, summary, description, tour_fact, navigation_hint, resume_hook, '
            'safety_notes, scene_context, capture_kind, confidence. '
            'Use human-friendly snake_case labels and keep names stable with prior places when appropriate. '
            'If this scene appears to be the same physical location as an existing place, reuse that place identity instead of inventing a new one. '
            'Keep every string short and concrete. Use at most 12 words per field and at most 3 aliases/tags. '
            'summary should explain why the place matters to navigation, resume, and tours. '
            'tour_fact should be a short visitor-facing fact grounded in the visible scene. '
            'navigation_hint should tell Sparky how to approach or re-find this spot. '
            'resume_hook should describe what to remember when resuming between nearby nodes. '
            'capture_kind should be one of place, landmark, transition, safe_anchor, obstacle, viewpoint. '
            'confidence must be a float between 0.0 and 1.0. '
            f'Existing places for naming context: {json.dumps(existing)}'
        )
        meta = self._call_openrouter(prompt, max_tokens=700)
        if not meta:
            return None
        meta['label'] = slugify(str(meta.get('label', '')))
        meta['room'] = slugify(str(meta.get('room', '')))
        meta['category'] = slugify(str(meta.get('category', '')))
        meta['aliases'] = [slugify(str(x)) for x in meta.get('aliases', []) if slugify(str(x))]
        meta['tags'] = [slugify(str(x)) for x in meta.get('tags', []) if slugify(str(x))]
        meta['summary'] = str(meta.get('summary', '') or meta.get('description', '')).strip()
        meta['description'] = str(meta.get('description', '')).strip()
        meta['tour_fact'] = str(meta.get('tour_fact', '')).strip()
        meta['navigation_hint'] = str(meta.get('navigation_hint', '')).strip()
        meta['resume_hook'] = str(meta.get('resume_hook', '')).strip()
        meta['safety_notes'] = str(meta.get('safety_notes', '')).strip()
        meta['scene_context'] = str(meta.get('scene_context', '')).strip()
        meta['capture_kind'] = slugify(str(meta.get('capture_kind', '')).strip())
        meta['sample_group'] = str(meta.get('sample_group', '')).strip()
        meta['sample_index'] = int(meta.get('sample_index', 0) or 0)
        meta['captured_at'] = str(meta.get('captured_at', '')).strip()
        try:
            meta['confidence'] = float(meta.get('confidence', 0.0))
        except Exception:
            meta['confidence'] = 0.0
        meta['confidence'] = max(0.0, min(1.0, meta['confidence']))
        if not meta['label']:
            meta['label'] = '_'.join([x for x in [meta['room'], meta['category']] if x]) or 'labeled_place'
        return meta

    def auto_save_tick(self) -> None:
        pose = self.lookup_current_pose()
        if pose is None:
            return
        self.publish_status('auto_save_tick: running')
        target_samples = max(0, int(self.get_parameter('auto_save_target_samples').value))
        allow_repeat_samples = bool(self.get_parameter('auto_save_allow_repeat_samples').value)
        if target_samples and self._teach_auto_save_count >= target_samples:
            self.publish_status(f'auto_save_tick: target_samples_reached count={self._teach_auto_save_count}')
            return
        if self.last_auto_save_xy is not None and not allow_repeat_samples:
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
        if int(meta.get('sample_index', 0) or 0) <= 0:
            meta['sample_index'] = self._teach_auto_save_count + 1
        if not str(meta.get('sample_group', '') or '').strip():
            meta['sample_group'] = self.route.name
        if not str(meta.get('captured_at', '') or '').strip():
            meta['captured_at'] = str(self.get_clock().now().nanoseconds)
        name = self.choose_place_name(meta, pose)
        merged_existing = name in self.place_store.places
        if allow_repeat_samples or (target_samples and self._teach_auto_save_count < target_samples):
            if not merged_existing:
                name = self.place_store.unique_name(name)
        place = self.make_place(name, pose, meta, source='vlm_auto', confidence=float(meta.get('confidence', 1.0)))
        near = self.place_store.nearest_within(place.x, place.y, float(self.get_parameter('auto_save_merge_distance_m').value))
        if near and near.name == name and not allow_repeat_samples:
            place.name = near.name
        self.save_place(place, source_text='vlm_auto')
        self.last_auto_save_xy = (place.x, place.y)
        if not merged_existing:
            self._teach_auto_save_count += 1
        else:
            self.publish_status(f'vlm_merged_duplicate_place name={place.name} sample_index={place.sample_index}')
        self.publish_status(
            f'vlm_labeled_place name={place.name} confidence={place.confidence:.2f} room={place.room or "-"} '
            f'category={place.category or "-"} tags={place.tags} summary={place.summary or place.description or "-"} '
            f'hint={place.navigation_hint or "-"} resume={place.resume_hook or "-"}'
        )

    def describe(self) -> None:
        meta = self.vlm_label_current_view()
        if not meta:
            return
        self.publish_status(f'describe: {meta.get("summary", "") or meta.get("description", "") or meta.get("label", "")}'.strip())

    def resolve_place(self, query: str) -> Optional[Place]:
        exact = self.place_store.get(query)
        if exact:
            return exact
        best, score, why = self.memory.resolve(query, list(self.place_store.places.values()))
        if best and score >= 20.0:
            self.publish_status(f'semantic_match query={query} -> {best.name} score={score:.1f} why={why}')
            return best
        map_name = self.route.name if self.route else None
        if map_name:
            resolved = self.agent_memory.resolve_destination(map_name, query)
            if resolved and resolved.get('pose'):
                pose = resolved['pose']
                return Place(
                    name=str(resolved.get('name') or resolved.get('label') or query),
                    x=float(pose.get('x', 0.0)),
                    y=float(pose.get('y', 0.0)),
                    yaw=float(pose.get('yaw', 0.0)),
                    room='',
                    category='',
                    aliases=list(pose.get('aliases', []) or []),
                    tags=list(pose.get('tags', []) or []),
                    description=str(resolved.get('summary', '')),
                    summary=str(resolved.get('summary', '')),
                    tour_fact=str(resolved.get('tour_fact', '')),
                    navigation_hint=str(resolved.get('navigation_hint', '')),
                    resume_hook=str(resolved.get('resume_hook', '')),
                    safety_notes=str(resolved.get('safety_notes', '')),
                    scene_context=str(resolved.get('scene_context', '')),
                    capture_kind=str(resolved.get('capture_kind', '')),
                    sample_index=int(resolved.get('sample_index', 0) or 0),
                    sample_group=str(resolved.get('sample_group', '')),
                    captured_at=str(resolved.get('captured_at', '')),
                    confidence=float(resolved.get('score', 0.0)) / 100.0 if resolved.get('score') else 0.5,
                    source='event_memory',
                    frame_id='map',
                )
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

    def route_summary(self) -> dict:
        return self.route_store.summary(self.route)

    def current_route_stop(self) -> Optional[RouteStop]:
        return self.route_store.current_stop(self.route)

    def _resolve_stop_place(self, stop: RouteStop) -> Optional[Place]:
        if not stop:
            return None
        place = self.place_store.get(stop.place_name)
        if place:
            return place
        place = self.resolve_place(stop.place_name)
        if place:
            return place
        if stop.fact:
            best, score, _why = self.memory.resolve(stop.fact, list(self.place_store.places.values()))
            if best and score >= 20.0:
                return best
        resolved = self.agent_memory.resolve_destination(self.route.name, stop.place_name or stop.fact or stop.name)
        if resolved and resolved.get('pose'):
            pose = resolved['pose']
            return Place(
                name=str(resolved.get('name') or resolved.get('label') or stop.name),
                x=float(pose.get('x', 0.0)),
                y=float(pose.get('y', 0.0)),
                yaw=float(pose.get('yaw', 0.0)),
                room='',
                category='',
                aliases=list(pose.get('aliases', []) or []),
                tags=list(pose.get('tags', []) or []),
                description=str(resolved.get('summary', '')),
                summary=str(resolved.get('summary', '')),
                tour_fact=str(resolved.get('tour_fact', '')),
                navigation_hint=str(resolved.get('navigation_hint', '')),
                resume_hook=str(resolved.get('resume_hook', '')),
                safety_notes=str(resolved.get('safety_notes', '')),
                scene_context=str(resolved.get('scene_context', '')),
                capture_kind=str(resolved.get('capture_kind', '')),
                sample_index=int(resolved.get('sample_index', 0) or 0),
                sample_group=str(resolved.get('sample_group', '')),
                captured_at=str(resolved.get('captured_at', '')),
                confidence=float(resolved.get('score', 0.0)) / 100.0 if resolved.get('score') else 0.5,
                source='event_memory',
                frame_id='map',
            )
        return None

    def _tour_fact_for_stop(self, stop: RouteStop) -> str:
        parts = [x.strip() for x in [stop.fact, stop.script, stop.navigation_hint, stop.resume_hook] if x and x.strip()]
        if parts:
            return ' '.join(parts)
        if stop.place_name:
            return f'This is {stop.place_name.replace("_", " ")}.'
        return 'This is one of Sparky’s tour stops.'

    def _publish_tour_explanation(self, stop: RouteStop, prefix: str = 'tour') -> None:
        fact = self._tour_fact_for_stop(stop)
        speech = f'{prefix}: {stop.name.replace("_", " ")}. {fact}'
        event = PatrolEvent(
            location={'place_name': stop.place_name or stop.name, 'route_name': self.route.name},
            confidence=float(stop.confidence or 1.0),
            request='tour_explain',
            status='tour_stop',
            completion=0.0,
            event_worthy=True,
            label=stop.name,
            replay_variations=[stop.fact] if stop.fact else [],
            update_strength=0.2 if stop.fact else 0.0,
            route_name=self.route.name,
            stop_name=stop.name,
            speech=speech,
            details={'fact': fact, 'script': stop.script, 'kind': stop.kind, 'pause_seconds': stop.pause_seconds},
        )
        self.publish_event(event)

    def start_tour(self, reset_index: bool = True) -> None:
        self.route.mode = 'tour'
        self.route.state = 'touring'
        self.route.current_stop_index = 0 if reset_index else self.route.current_stop_index
        if reset_index:
            for stop in self.route.stops:
                stop.status = 'pending'
            self._stop_announced.clear()
        self.route_store.save(self.route)
        stop = self.current_route_stop()
        self.publish_status(f'tour_started route={self.route.name} stop={stop.name if stop else "-"}')
        self.publish_event(PatrolEvent(
            location=self.pose_summary(),
            confidence=1.0,
            request='start_tour',
            status='tour_started',
            completion=0.0,
            event_worthy=True,
            label=self.route.name,
            route_name=self.route.name,
            stop_name=stop.name if stop else '',
            speech='tour: Starting the guest tour now.',
            details=self.route_summary(),
        ))
        if stop:
            self.send_stop(stop, reason='tour_start')

    def pause_tour(self, speech: str = 'tour: Pausing here for a moment.') -> None:
        self.cancel_active_goal('tour_pause')
        self.route.state = 'paused'
        self._tour_pause_until_ns = 0
        self.route_store.save(self.route)
        self.publish_event(PatrolEvent(
            location=self.pose_summary(),
            confidence=1.0,
            request='pause_tour',
            status='tour_paused',
            completion=float(self.route.current_stop_index) / max(1.0, float(len(self.route.stops))),
            event_worthy=True,
            label=self.route.name,
            route_name=self.route.name,
            stop_name=self.current_route_stop().name if self.current_route_stop() else '',
            speech=speech,
            details=self.route_summary(),
        ))

    def resume_tour(self, speech: str = 'tour: Resuming the tour.') -> None:
        self.route.state = 'touring'
        self.route_store.save(self.route)
        self.publish_event(PatrolEvent(
            location=self.pose_summary(),
            confidence=1.0,
            request='resume_tour',
            status='tour_resumed',
            completion=float(self.route.current_stop_index) / max(1.0, float(len(self.route.stops))),
            event_worthy=True,
            label=self.route.name,
            route_name=self.route.name,
            stop_name=self.current_route_stop().name if self.current_route_stop() else '',
            speech=speech,
            details=self.route_summary(),
        ))
        stop = self.current_route_stop()
        if stop and stop.status != 'complete':
            self.publish_status(f'resume_current_stop stop={stop.name}')
            self.send_stop(stop, reason='resume')
            return
        selected = self.nearest_route_stop(include_complete=False)
        if selected is not None:
            _, selected_stop = selected
            self.publish_status(f'resume_pose_selected_stop stop={selected_stop.name}')
            self.send_stop(selected_stop, reason='resume_pose_select')
            return
        if stop and stop.status == 'complete' and self.route.current_stop_index < len(self.route.stops) - 1:
            self.advance_tour()
        elif stop and stop.status == 'complete':
            self.on_route_complete()

    def advance_tour(self) -> None:
        next_stop = self.route_store.advance(self.route)
        if next_stop is None:
            self.on_route_complete()
            return
        self.publish_status(f'tour_advance next_stop={next_stop.name}')
        self.send_stop(next_stop, reason='advance')

    def send_stop(self, stop: RouteStop, reason: str = 'route') -> None:
        place = self._resolve_stop_place(stop)
        if place is None:
            self.publish_status(f'route_stop_unresolved stop={stop.name} reason={reason}')
            self.handle_goal_failure(f'unresolved_stop:{stop.name}')
            return
        self._route_goal_active = True
        self.route.state = 'moving_to_stop'
        self.route_store.save(self.route)
        self._goal_place = place
        self._pending_recovery_goal = None
        self.publish_event(PatrolEvent(
            location={'x': place.x, 'y': place.y, 'yaw': place.yaw, 'frame_id': place.frame_id or self.map_frame, 'place_name': place.name},
            confidence=float(getattr(place, 'confidence', 1.0) or 1.0),
            request=reason,
            status='sending_goal',
            completion=float(self.route.current_stop_index) / max(1.0, float(len(self.route.stops))),
            event_worthy=True,
            label=stop.name,
            route_name=self.route.name,
            stop_name=stop.name,
            speech=f'tour: Heading to {stop.name.replace("_", " ")}.',
            details={'place': asdict(place), 'stop': asdict(stop)},
        ))
        self.send_goal(place)

    def on_stop_reached(self) -> None:
        stop = self.current_route_stop()
        if stop:
            stop.status = 'complete'
            self.route_store.save(self.route)
            self._publish_tour_explanation(stop, prefix='tour')
            pause_until = self.get_clock().now().nanoseconds + int(max(0.1, float(stop.pause_seconds or self.get_parameter('tour_default_pause_sec').value)) * 1e9)
            self._tour_pause_until_ns = pause_until
            self.publish_status(f'tour_stop_complete stop={stop.name} pause_sec={stop.pause_seconds:.1f}')
            pose = self.pose_summary()
            self.agent_memory.add_observation({
                'id': f'tour_stop_{route_slugify(stop.name)}_{self.get_clock().now().nanoseconds}',
                'map_name': self.route.name,
                'label': stop.name,
                'summary': stop.fact or stop.script or stop.place_name,
                'pose': pose if pose.get('available') else {},
                'aliases': list(stop.aliases or []),
                'objects': list(stop.tags or []),
            })
        self._goal_place = None
        self._active_goal_handle = None
        if bool(self.get_parameter('tour_mode').value) and self.route.current_stop_index < len(self.route.stops) - 1:
            self.route.state = 'tour_pause'
            self.route_store.save(self.route)
        else:
            self.on_route_complete()

    def on_route_complete(self) -> None:
        self.route.state = 'complete'
        self.route_store.save(self.route)
        self._goal_place = None
        self._active_goal_handle = None
        self._pending_recovery_goal = None
        self._route_goal_active = False
        self._goal_send_retry_ns = 0
        self.publish_event(PatrolEvent(
            location=self.pose_summary(),
            confidence=1.0,
            request='route_complete',
            status='route_complete',
            completion=1.0,
            event_worthy=True,
            label=self.route.name,
            route_name=self.route.name,
            speech='tour: The route is complete. Thank you for joining me.',
            details=self.route_summary(),
        ))

    def handle_goal_failure(self, reason: str) -> None:
        if not bool(self.get_parameter('fallback_enable').value):
            self.publish_status(f'route_failure fallback_disabled reason={reason}')
            self.route.state = 'blocked'
            self.route_store.save(self.route)
            self.cancel_active_goal(reason)
            self._goal_send_retry_ns = 0
            return
        self._last_goal_failed = True
        self._route_goal_active = False
        self.cancel_active_goal(reason)
        self._goal_send_retry_ns = 0
        self.route.last_failure = {'reason': reason, 'attempts': self._recovery_retries}
        self.route_store.set_last_failure(self.route, self.route.last_failure)
        self.route.state = 'blocked'
        self.route_store.save(self.route)
        self.recover_from_failure(reason)

    def route_tick(self) -> None:
        if self._tour_pause_until_ns and self.get_clock().now().nanoseconds >= self._tour_pause_until_ns:
            self._tour_pause_until_ns = 0
            if self.route.state == 'tour_pause':
                if self.route.current_stop_index < len(self.route.stops) - 1:
                    self.advance_tour()
                else:
                    self.on_route_complete()
        if self._goal_place is not None and self._active_goal_handle is None and self._goal_send_retry_ns:
            if self.get_clock().now().nanoseconds >= self._goal_send_retry_ns:
                retry_goal = self._goal_place
                route_goal_active = self._route_goal_active
                self._goal_send_retry_ns = 0
                self.publish_status(f'retrying_goal_send target={retry_goal.name}')
                self.send_goal(retry_goal, route_goal_active=route_goal_active)
                return
        if self.route.state == 'touring' and self._goal_place is None and self.current_route_stop() is not None:
            self.send_stop(self.current_route_stop(), reason='resume_tick')

    def recover_from_failure(self, reason: str) -> None:
        if self._goal_place is None and self.current_route_stop() is None:
            self.publish_status('recovery_without_goal')
            self.pause_tour('tour: I am pausing because I do not have a target to recover.')
            return
        self._recovery_retries += 1
        scan = self.scan_summary()
        pose = self.pose_summary()
        goal = asdict(self._goal_place) if self._goal_place else {}
        meta = self.openrouter.analyze_navigation_failure(
            reason=reason,
            goal=goal,
            route_summary=self.route_summary(),
            scan_summary=scan,
            pose_summary=pose,
        )
        tf_age = self.tf_age_seconds(self.map_frame, self.base_frame)
        tf_max_age = float(self.get_parameter('tf_max_age_sec').value)
        if tf_age is None:
            meta['failure_type'] = 'missing_tf'
            meta['action'] = 'relocalize'
            meta['safe_anchor'] = meta.get('safe_anchor') or 'spawn'
            meta['alternate_query'] = meta.get('alternate_query') or 'spawn'
            meta['note'] = 'Localization transform is unavailable.'
        elif tf_age > tf_max_age:
            meta['failure_type'] = 'stale_tf'
            meta['action'] = 'wait'
            meta['note'] = f'Localization transform is stale by {tf_age:.1f}s.'
        failure_type = str(meta.get('failure_type', 'unknown')).strip()
        action = str(meta.get('action', 'retry_nav')).strip()
        safe_anchor_name = str(meta.get('safe_anchor', '')).strip()
        alternate_query = str(meta.get('alternate_query', '')).strip()
        speech = str(meta.get('speech', '')).strip() or f'recovery: {meta.get("note", "recovering from a navigation failure")}'
        self.publish_event(PatrolEvent(
            location=pose,
            confidence=float(meta.get('confidence', 0.2) or 0.2),
            request='route_recovery',
            status='recovery_started',
            completion=float(self.route.current_stop_index) / max(1.0, float(len(self.route.stops))),
            event_worthy=True,
            label=failure_type,
            replay_variations=[failure_type, action],
            update_strength=0.5,
            route_name=self.route.name,
            stop_name=self.current_route_stop().name if self.current_route_stop() else '',
            speech=speech,
            details={'failure_type': failure_type, 'action': action, 'safe_anchor': safe_anchor_name, 'alternate_query': alternate_query, 'reason': reason, 'scan': scan},
        ))
        self.publish_status(f'recovery_analysis failure_type={failure_type} action={action} retries={self._recovery_retries}')
        if self._recovery_retries > int(self.get_parameter('recovery_max_retries').value):
            self.publish_status('recovery_exhausted')
            self.publish_reply('tour: I need help recovering from this route failure.')
            self.pause_tour('tour: I’m pausing the route and asking for help.')
            return
        if failure_type in {'stale_tf', 'missing_tf'}:
            if self._recovery_retries <= 1:
                self.publish_status('recovery_waiting_for_fresh_tf')
                return
            self.pause_tour('tour: I need a fresh localization estimate before I can continue.')
            return
        if action in {'rotate_left', 'rotate_right', 'backup', 'creep_forward', 'wait', 'stop'}:
            if action == 'rotate_left':
                self.start_motion(0.0, float(self.get_parameter('fallback_angular_speed').value), 2.0, self._goal_place, route_goal_active=True)
            elif action == 'rotate_right':
                self.start_motion(0.0, -float(self.get_parameter('fallback_angular_speed').value), 2.0, self._goal_place, route_goal_active=True)
            elif action == 'backup':
                self.start_motion(-float(self.get_parameter('fallback_linear_speed').value), 0.0, 2.5, self._goal_place, route_goal_active=True)
            elif action == 'creep_forward':
                if scan.get('front_clear', True):
                    self.start_motion(float(self.get_parameter('fallback_linear_speed').value), 0.0, 2.0, self._goal_place, route_goal_active=True)
                else:
                    self.publish_status('recovery_creep_blocked')
            else:
                self.cmd_vel_pub.publish(Twist())
        elif action in {'safe_anchor', 'relocalize', 'retry_nav', 'alternate_goal'}:
            original_goal = self._goal_place or (self.current_route_stop() and self._resolve_stop_place(self.current_route_stop()))
            anchor = None
            if safe_anchor_name:
                anchor = self.resolve_place(safe_anchor_name) or self.place_store.get(safe_anchor_name)
            if anchor is None and alternate_query:
                anchor = self.resolve_place(alternate_query)
            if anchor is None and self.route.safe_anchors:
                for candidate in self.route.safe_anchors.values():
                    anchor = self.resolve_place(candidate) or self.place_store.get(candidate)
                    if anchor:
                        break
            if anchor:
                self.publish_status(f'recovery_anchor {anchor.name}')
                self.route.state = 'recovering'
                self.route_store.save(self.route)
                self._pending_recovery_goal = original_goal
                self.send_goal(anchor, route_goal_active=False)
                return
            if action == 'retry_nav' and self._goal_place is not None:
                self.publish_status('recovery_retry_goal')
                self.send_goal(self._goal_place, route_goal_active=True)
                return
            self.pause_tour('tour: I am re-checking the route before continuing.')
        else:
            waypoint = self.openrouter.propose_waypoint(
                context={'route': self.route_summary(), 'scan': scan, 'pose': pose, 'reason': reason},
                image_data_url=f'data:image/jpeg;base64,{base64.b64encode(self.latest_image_bytes).decode("ascii")}' if self.latest_image_bytes else None,
            )
            if waypoint.get('action') in {'forward', 'turn_left', 'turn_right', 'backup'}:
                action2 = waypoint.get('action')
                if action2 == 'forward':
                    self.start_motion(float(self.get_parameter('fallback_linear_speed').value), 0.0, 2.0, self._goal_place, route_goal_active=True)
                elif action2 == 'backup':
                    self.start_motion(-float(self.get_parameter('fallback_linear_speed').value), 0.0, 2.0, self._goal_place, route_goal_active=True)
                elif action2 == 'turn_left':
                    self.start_motion(0.0, float(self.get_parameter('fallback_angular_speed').value), 2.0, self._goal_place, route_goal_active=True)
                elif action2 == 'turn_right':
                    self.start_motion(0.0, -float(self.get_parameter('fallback_angular_speed').value), 2.0, self._goal_place, route_goal_active=True)
                self.publish_reply(str(waypoint.get('speech') or 'tour: I am making a small visual recovery move.'))
                return
            self.pause_tour('tour: I am pausing to recover localization.')

    def validate_goal_pose(self, pose: PoseStamped) -> bool:
        frame = (pose.header.frame_id or '').strip()
        if not frame:
            self.publish_status('refusing_goal empty_frame_id')
            return False
        return True

    def send_goal(self, place: Place, route_goal_active: bool = False) -> None:
        self._goal_place = place
        self._last_goal_failed = False
        self._route_goal_active = route_goal_active
        pose = self.build_pose(place)
        if not self.validate_goal_pose(pose):
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.publish_status(
            f'sending_goal {place.name} -> ({place.x:.2f}, {place.y:.2f}, yaw={place.yaw:.2f}, frame={pose.header.frame_id})'
        )
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._goal_send_retry_count += 1
            self._goal_send_retry_ns = self.get_clock().now().nanoseconds + int(2.0 * 1e9)
            self.publish_status(f'navigate_to_pose server not available retry={self._goal_send_retry_count}')
            return
        self._goal_send_retry_ns = 0
        self._goal_send_retry_count = 0
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def cancel_active_goal(self, reason: str = 'cancelled') -> None:
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            self._goal_send_retry_ns = 0
            return
        try:
            self.publish_status(f'cancelling_goal reason={reason}')
            goal_handle.cancel_goal_async()
        except Exception as exc:
            self.publish_status(f'cancel_goal_failed reason={reason} error={exc}')
        finally:
            self._active_goal_handle = None
            self._route_goal_active = False
            self._goal_send_retry_ns = 0

    def feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', 0.0)
        self.publish_status(f'navigating target={self._goal_place.name if self._goal_place else "-"} distance_remaining={dist:.2f}')

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status('goal rejected')
            self.handle_goal_failure('rejected')
            return
        self._goal_send_retry_ns = 0
        self._goal_send_retry_count = 0
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future) -> None:
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('goal succeeded')
            self._fallback_attempts = 0
            self._last_goal_failed = False
            self._recovery_retries = 0
            self._goal_send_retry_ns = 0
            self._goal_send_retry_count = 0
            self._active_goal_handle = None
            if self.route.state == 'recovering':
                if self._pending_recovery_goal is not None:
                    next_goal = self._pending_recovery_goal
                    self._pending_recovery_goal = None
                    self.publish_status(f'recovery_anchor_reached retrying_goal={next_goal.name}')
                    self.route.state = 'moving_to_stop'
                    self.route_store.save(self.route)
                    self.send_goal(next_goal, route_goal_active=True)
                    return
                self.route.state = 'paused'
                self.route_store.save(self.route)
                self.publish_status('recovery_anchor_reached_no_retry')
            if self._route_goal_active:
                self._route_goal_active = False
                self.on_stop_reached()
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status('goal canceled')
            self._active_goal_handle = None
            self._goal_send_retry_ns = 0
            self._goal_send_retry_count = 0
            if self.route.state in {'paused', 'tour_pause'}:
                return
        else:
            self.publish_status(f'goal failed status={status}')
            self.handle_goal_failure(f'status_{status}')

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

    def start_motion(self, linear_x: float, angular_z: float, duration_sec: float, retry_place: Optional[Place], route_goal_active: bool = False) -> None:
        self._motion_active = True
        self._motion_twist = Twist()
        self._motion_twist.linear.x = float(linear_x)
        self._motion_twist.angular.z = float(angular_z)
        self._motion_end_ns = self.get_clock().now().nanoseconds + int(duration_sec * 1e9)
        self._motion_retry_place = retry_place
        self._motion_retry_route_goal = route_goal_active

    def motion_tick(self) -> None:
        if not self._motion_active:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self._motion_end_ns:
            self.cmd_vel_pub.publish(Twist())
            self._motion_active = False
            if self._motion_retry_place is not None:
                place = self._motion_retry_place
                route_goal_active = self._motion_retry_route_goal
                self._motion_retry_place = None
                self._motion_retry_route_goal = False
                self.send_goal(place, route_goal_active=route_goal_active)
            return
        self.cmd_vel_pub.publish(self._motion_twist)

    def try_fallback_recovery(self, reason: str) -> None:
        if not bool(self.get_parameter('fallback_enable').value):
            return
        self.handle_goal_failure(reason)

    def cmd_cb(self, msg: String) -> None:
        line = msg.data.strip()
        if not line:
            return
        event = None
        if line.startswith('{'):
            try:
                event = json.loads(line)
            except Exception:
                event = None
        parts = line.split()
        cmd = str((event or {}).get('type') or parts[0]).lower()
        event_place = str((event or {}).get('place') or (event or {}).get('target') or '').strip()
        if cmd == 'status':
            self.publish_status('status_ok')
            return
        if cmd in {'start_tour', 'tour_start'}:
            if event and event.get('route_name'):
                self.route.name = route_slugify(str(event.get('route_name')))
            self.start_tour(reset_index=bool((event or {}).get('reset_index', True)))
            return
        if cmd in {'pause_tour', 'tour_pause'}:
            self.pause_tour(str((event or {}).get('speech') or 'tour: Pausing the route.'))
            return
        if cmd in {'resume_tour', 'tour_resume'}:
            self.resume_tour(str((event or {}).get('speech') or 'tour: Resuming the route.'))
            return
        if cmd in {'advance_tour', 'next_stop'}:
            self.advance_tour()
            return
        if cmd in {'tour_fact_request', 'explain_stop', 'tour_explain'}:
            stop = self.current_route_stop()
            if stop:
                self._publish_tour_explanation(stop, prefix='tour')
            else:
                self.publish_reply('tour: I do not have a current stop to explain yet.')
            return
        if cmd in {'recover_route', 'route_recover'}:
            self.handle_goal_failure(str((event or {}).get('reason') or 'route_recovery_requested'))
            return
        if cmd in {'handoff_tour', 'tour_handoff'}:
            self.publish_event(PatrolEvent(
                location=self.pose_summary(),
                confidence=1.0,
                request='handoff_tour',
                status='tour_handoff',
                completion=float(self.route.current_stop_index) / max(1.0, float(len(self.route.stops))),
                event_worthy=True,
                label=self.route.name,
                route_name=self.route.name,
                speech=str((event or {}).get('speech') or 'tour: I am handing off the tour to the next guide or operator.'),
                details=self.route_summary(),
            ))
            return
        if cmd == 'save' and len(parts) >= 2:
            pose = self.lookup_current_pose()
            if pose is None:
                return
            place = self.make_place(parts[1], pose, self.parse_kv(parts[2:]), source='manual')
            self.save_place(place, source_text='manual')
            return
        if cmd in {'save_spawn', 'spawn-save', 'save-spawn'}:
            pose = self.lookup_current_pose()
            if pose is None:
                self.publish_status('spawn_save_failed_no_pose')
                return
            place = self.make_place(
                'spawn',
                pose,
                {'room': 'spawn', 'category': 'spawn', 'aliases': ['origin', 'start'], 'tags': ['spawn', 'initial_pose']},
                source='manual',
            )
            self.save_place(place, source_text='spawn_save')
            self.publish_status('spawn_saved_from_current_pose')
            return
        if cmd in {'save_map', 'save-map', 'savemap'}:
            self.save_map_snapshot(force=True)
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
        if cmd in {'go', 'navigate'}:
            query = event_place or ' '.join(parts[1:]).replace('to ', '').strip()
            place = self.resolve_place(query)
            if not place:
                self.publish_status(f'place not found: {query}')
                return
            self.send_goal(place)
            return
        if cmd == 'near':
            query = event_place or ('near ' + ' '.join(parts[1:]) if len(parts) >= 2 else '')
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
            self.cmd_vel_pub.publish(Twist())
            self.pause_tour('tour: Route cancelled. I am stopping safely.')
            return
        self.publish_status(f'unknown command: {line}')

    def save_map_snapshot(self, force: bool = False) -> None:
        cmd = str(self.get_parameter('map_saver_cmd').value)
        if not cmd:
            return
        if not force and not bool(self.get_parameter('save_map_on_shutdown').value):
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
            self.route_store.save(self.route)
            self.place_store.save()
            meta = self.session_store.load_session_yaml(self.session)
            meta['semantic_graph'] = self.memory.build_relationships(list(self.place_store.places.values()))
            meta['route'] = self.route.to_dict()
            if self.mode == 'teach' and not meta.get('spawn'):
                pose = self.lookup_current_pose()
                if pose is not None:
                    meta['spawn'] = {
                        'x': float(pose.pose.position.x),
                        'y': float(pose.pose.position.y),
                        'yaw': float(yaw_from_quat(pose.pose.orientation)),
                        'frame_id': pose.header.frame_id or self.map_frame,
                    }
            self.session_store.save_session_yaml(self.session, meta)
            if self.mode == 'teach':
                self.save_map_snapshot(force=False)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.on_shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
