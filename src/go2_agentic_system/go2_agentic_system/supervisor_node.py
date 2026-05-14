from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.srv import SaveMap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .parser import parse_command
from .storage import MemoryStore, sanitize_name, utc_now


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('supervisor_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('input_topic', '/agent/input_text')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('mode_topic', '/agent/mode')
        self.declare_parameter('telemetry_topic', '/agent/telemetry')
        self.declare_parameter('speak_topic', '/agent/speak_text')
        self.declare_parameter('capture_request_topic', '/agent/capture_semantic')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('save_map_service', '/map_saver/save_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_poll_sec', 0.2)
        self.declare_parameter('telemetry_period_sec', 0.5)
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('nav_feedback_sec', 1.0)
        self.declare_parameter('manual_linear_speed_mps', 0.25)
        self.declare_parameter('manual_angular_speed_rps', 0.70)
        self.declare_parameter('default_manual_duration_sec', 0.9)
        self.declare_parameter('autostart_nav2_wait', False)

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        guard = self.store.read_guardrails().get('guardrails', {})
        self.min_front_clearance = float(guard.get('min_front_clearance_m', 0.75))
        self.min_rear_clearance = float(guard.get('min_rear_clearance_m', 0.45))
        self.front_arc_deg = float(guard.get('front_arc_deg', 40.0))
        self.rear_arc_deg = float(guard.get('rear_arc_deg', 50.0))
        self.localization_timeout_sec = float(guard.get('localization_timeout_sec', 2.0))
        self.require_localization_for_navigation = bool(guard.get('require_localization_for_navigation', True))
        self.allow_manual_without_localization = bool(guard.get('allow_manual_without_localization', True))

        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 50)
        self.mode_pub = self.create_publisher(String, self.get_parameter('mode_topic').value, 20)
        self.telemetry_pub = self.create_publisher(String, self.get_parameter('telemetry_topic').value, 20)
        self.speak_pub = self.create_publisher(String, self.get_parameter('speak_topic').value, 20)
        self.capture_pub = self.create_publisher(String, self.get_parameter('capture_request_topic').value, 20)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 20)

        self.create_subscription(String, self.get_parameter('input_topic').value, self.on_input_text, 50)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 20)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 50)
        self.create_subscription(PoseStamped, self.get_parameter('goal_pose_topic').value, self.on_goal_pose, 10)
        self.save_map_cli = self.create_client(SaveMap, self.get_parameter('save_map_service').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        if self.get_parameter('autostart_nav2_wait').value:
            self.get_logger().info('waiting for Nav2 to become active...')
            self.navigator.waitUntilNav2Active()

        self.latest_front_min: Optional[float] = None
        self.latest_rear_min: Optional[float] = None
        self.latest_map_pose: Optional[dict] = None
        self.latest_map_pose_time: Optional[Time] = None
        self.latest_odom_time: Optional[Time] = None
        self.manual_twist = Twist()
        self.manual_until_ns = 0
        self.nav_active = False
        self.pending_say_text: Optional[str] = None
        self.last_feedback_text = ''

        self.create_timer(float(self.get_parameter('tf_poll_sec').value), self.poll_tf)
        self.create_timer(float(self.get_parameter('telemetry_period_sec').value), self.publish_telemetry)
        self.create_timer(float(self.get_parameter('control_period_sec').value), self.control_tick)
        self.create_timer(float(self.get_parameter('nav_feedback_sec').value), self.nav_feedback_tick)

        self.set_mode(self.store.read_state().get('mode', 'idle'))
        self.status('supervisor ready; operator console available on /agent UI')

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)
        self.store.log_event('status', {'text': text})

    def set_mode(self, mode: str) -> None:
        self.store.update_state(mode=mode)
        self.mode_pub.publish(String(data=mode))

    def active_map(self) -> Optional[str]:
        return self.store.read_state().get('active_map')

    def localization_fresh(self) -> bool:
        if self.latest_map_pose_time is None:
            return False
        age = (self.get_clock().now() - self.latest_map_pose_time).nanoseconds / 1e9
        return age <= self.localization_timeout_sec

    def publish_telemetry(self) -> None:
        counts = self.store.count_memories(self.active_map())
        payload = {
            'mode': self.store.read_state().get('mode', 'idle'),
            'active_map': self.active_map(),
            'survey_active': bool(self.store.read_state().get('active_survey')),
            'pose': self.latest_map_pose,
            'localized': self.localization_fresh(),
            'front_clearance_m': self.latest_front_min,
            'rear_clearance_m': self.latest_rear_min,
            'nav_active': self.nav_active,
            'pending_say_text': self.pending_say_text,
            'memory': counts,
            'last_command': self.store.read_state().get('last_command'),
        }
        self.telemetry_pub.publish(String(data=json.dumps(payload)))

    def poll_tf(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('map_frame').value,
                self.get_parameter('base_frame').value,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return
        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z), 1.0 - 2.0 * (r.z * r.z))
        self.latest_map_pose = {
            'frame_id': self.get_parameter('map_frame').value,
            'x': float(t.x),
            'y': float(t.y),
            'z': float(t.z),
            'yaw': float(yaw),
            'stamp': utc_now(),
            'source': 'tf_map_to_base',
        }
        self.latest_map_pose_time = self.get_clock().now()
        self.store.update_state(last_map_pose=self.latest_map_pose)

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom_time = self.get_clock().now()

    def _sector_min(self, msg: LaserScan, front: bool) -> Optional[float]:
        arc = math.radians(self.front_arc_deg if front else self.rear_arc_deg)
        values = []
        for idx, rng in enumerate(msg.ranges):
            if not math.isfinite(rng) or rng <= 0.01:
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            if front and abs(angle) <= arc:
                values.append(rng)
            elif not front and abs(abs(angle) - math.pi) <= arc:
                values.append(rng)
        if not values:
            values = [r for r in msg.ranges if math.isfinite(r) and r > 0.01]
        return min(values) if values else None

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_front_min = self._sector_min(msg, True)
        self.latest_rear_min = self._sector_min(msg, False)
        if self.nav_active and self.latest_front_min is not None and self.latest_front_min < self.min_front_clearance:
            self.cancel_navigation(f'obstacle too close at {self.latest_front_min:.2f}m')

    def can_execute_twist(self, linear: float) -> tuple[bool, str]:
        if linear > 0.0 and self.latest_front_min is not None and self.latest_front_min < self.min_front_clearance:
            return False, f'front blocked at {self.latest_front_min:.2f}m'
        if linear < 0.0 and self.latest_rear_min is not None and self.latest_rear_min < self.min_rear_clearance:
            return False, f'rear blocked at {self.latest_rear_min:.2f}m'
        if not self.allow_manual_without_localization and not self.localization_fresh():
            return False, 'localization is stale'
        return True, 'ok'

    def on_input_text(self, msg: String) -> None:
        raw = msg.data.strip()
        cmd = parse_command(raw)
        ctype = cmd.get('type')
        self.store.update_state(last_command=raw)
        if ctype == 'noop':
            return
        if ctype == 'help':
            self.status('commands: start survey | stop survey | save map as NAME | remember this place as NAME | capture memory [as NAME] | what do you remember about NAME | go to NAME [and say ...] | say ... | forward/back/left/right [sec] | stop | status | list places | memory status')
            return
        if ctype == 'status':
            pose_txt = 'unknown'
            if self.latest_map_pose:
                pose_txt = f"x={self.latest_map_pose['x']:.2f} y={self.latest_map_pose['y']:.2f} yaw={self.latest_map_pose['yaw']:.2f}"
            counts = self.store.count_memories(self.active_map())
            self.status(f'mode={self.store.read_state().get("mode")} active_map={self.active_map()} pose={pose_txt} front={self.latest_front_min} rear={self.latest_rear_min} nav_active={self.nav_active} memories={counts["observations"]} obs/{counts["places"]} places')
            return
        if ctype == 'list_places':
            map_name = self.active_map()
            if not map_name:
                self.status('no active map')
                return
            names = sorted(self.store.list_places(map_name).get('places', {}).keys())
            self.status('places: ' + (', '.join(names) if names else '(none)'))
            return
        if ctype == 'memory_status':
            counts = self.store.count_memories(self.active_map())
            self.status(f'memory stats: active_map={self.active_map()} observations={counts["observations"]} places={counts["places"]}')
            return
        if ctype == 'memory_recall':
            self.recall_memory(str(cmd['query']))
            return
        if ctype in {'stop_all', 'stop_motion'}:
            self.stop_all('operator stop')
            return
        if ctype == 'set_active_map':
            map_name = sanitize_name(str(cmd['map_name']))
            self.store.update_state(active_map=map_name)
            self.status(f'active map set to {map_name}')
            return
        if ctype == 'start_survey':
            self.start_survey()
            return
        if ctype == 'stop_survey':
            self.stop_survey()
            return
        if ctype == 'save_map':
            self.save_map(str(cmd['map_name']))
            return
        if ctype == 'remember_current_pose':
            self.remember_current_pose(str(cmd['place']))
            return
        if ctype == 'capture_semantic':
            label = cmd.get('label')
            self.capture_pub.publish(String(data='' if label is None else str(label)))
            self.status('semantic capture requested')
            return
        if ctype == 'navigate_named_place':
            self.navigate_to_query(str(cmd['place']), say_text=cmd.get('say'))
            return
        if ctype == 'say':
            text = str(cmd['text'])
            self.speak_pub.publish(String(data=text))
            self.status(f'speech queued: {text}')
            return
        if ctype == 'manual_drive':
            duration = float(cmd.get('duration') or self.get_parameter('default_manual_duration_sec').value)
            self.handle_manual_drive(float(cmd.get('linear', 0.0)), float(cmd.get('angular', 0.0)), duration, str(cmd.get('label', 'manual_drive')))
            return
        self.status(f'unknown command: {cmd.get("text", raw)}')

    def start_survey(self) -> None:
        state = self.store.read_state()
        active_map = state.get('active_map')
        if not active_map:
            active_map = sanitize_name('survey_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
            self.store.update_state(active_map=active_map)
        survey_name = sanitize_name('survey_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.store.update_state(active_survey=survey_name)
        self.set_mode('survey')
        self.store.log_event('survey_started', {'survey_name': survey_name, 'active_map': active_map})
        self.status(f'survey mode active; current map label={active_map}')

    def stop_survey(self) -> None:
        self.store.log_event('survey_stopped', {'survey_name': self.store.read_state().get('active_survey')})
        self.store.update_state(active_survey=None)
        self.set_mode('idle')
        self.status('survey mode stopped; AI mode active')

    def stop_all(self, reason: str) -> None:
        if self.nav_active:
            self.navigator.cancelTask()
            self.nav_active = False
        self.manual_until_ns = 0
        self.pending_say_text = None
        self.cmd_pub.publish(Twist())
        self.set_mode('survey' if self.store.read_state().get('active_survey') else 'idle')
        self.status(f'stopped: {reason}')

    def handle_manual_drive(self, linear_scale: float, angular_scale: float, duration: float, label: str) -> None:
        if self.nav_active:
            self.navigator.cancelTask()
            self.nav_active = False
        allowed, reason = self.can_execute_twist(linear_scale)
        if not allowed:
            self.cmd_pub.publish(Twist())
            self.status(f'manual blocked: {reason}')
            return
        twist = Twist()
        twist.linear.x = linear_scale * float(self.get_parameter('manual_linear_speed_mps').value)
        twist.angular.z = angular_scale * float(self.get_parameter('manual_angular_speed_rps').value)
        self.manual_twist = twist
        self.manual_until_ns = self.get_clock().now().nanoseconds + int(max(duration, 0.05) * 1e9)
        self.set_mode('survey' if self.store.read_state().get('active_survey') else 'manual')
        self.store.log_event('manual_drive', {'label': label, 'linear': twist.linear.x, 'angular': twist.angular.z, 'duration_sec': duration})
        self.status(f'manual drive: {label} for {duration:.2f}s')

    def control_tick(self) -> None:
        if self.manual_until_ns <= 0:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self.manual_until_ns:
            self.manual_until_ns = 0
            self.cmd_pub.publish(Twist())
            self.set_mode('survey' if self.store.read_state().get('active_survey') else 'idle')
            return
        allowed, reason = self.can_execute_twist(self.manual_twist.linear.x)
        if not allowed:
            self.manual_until_ns = 0
            self.cmd_pub.publish(Twist())
            self.set_mode('idle')
            self.status(f'manual stopped: {reason}')
            return
        self.cmd_pub.publish(self.manual_twist)

    def recall_memory(self, query: str) -> None:
        map_name = self.active_map()
        matches = self.store.search_memories(query, map_name=map_name, limit=3)
        if not matches:
            self.status(f'no memory match for {query}')
            return
        parts = []
        for item in matches:
            pose = item.get('pose') or {}
            x = pose.get('x')
            y = pose.get('y')
            summary = str(item.get('summary') or '').strip()
            coords = ''
            if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                coords = f' @ ({x:.2f}, {y:.2f})'
            detail = summary[:120] if summary else 'no summary'
            parts.append(f'[{item.get("type")}] {item.get("name")}{coords}: {detail}')
        msg = 'memory recall: ' + ' | '.join(parts)
        self.store.log_event('memory_recall', {'query': query, 'matches': matches})
        self.status(msg)

    def remember_current_pose(self, place_name_raw: str) -> None:
        if self.latest_map_pose is None:
            self.status('cannot remember place: no map-frame pose')
            return
        map_name = self.active_map() or sanitize_name('unspecified_map')
        place_name = sanitize_name(place_name_raw)
        obs = self.store.list_observations(map_name)
        recent_summary = obs[-1]['summary'] if obs else ''
        self.store.remember_place(
            map_name,
            place_name,
            {
                'frame_id': self.get_parameter('map_frame').value,
                'x': self.latest_map_pose['x'],
                'y': self.latest_map_pose['y'],
                'yaw': self.latest_map_pose['yaw'],
                'saved_at': utc_now(),
                'source': self.latest_map_pose.get('source', 'tf'),
                'summary': recent_summary,
                'aliases': [place_name_raw.strip()],
            },
        )
        self.store.log_event('place_saved', {'map_name': map_name, 'place_name': place_name, 'pose': self.latest_map_pose})
        self.status(f'saved place {place_name} on map {map_name}')

    def make_pose(self, frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def navigate_to_query(self, query: str, say_text: Optional[object]) -> None:
        if self.require_localization_for_navigation and not self.localization_fresh():
            self.status('navigation blocked: localization is stale or unavailable')
            return
        map_name = self.active_map()
        if not map_name:
            self.status('navigation blocked: no active map')
            return
        resolved = self.store.resolve_destination(map_name, query)
        if not resolved:
            self.status(f'navigation blocked: no remembered destination for {query}')
            return
        pose = resolved['pose']
        goal = self.make_pose(pose.get('frame_id', self.get_parameter('map_frame').value), pose['x'], pose['y'], pose.get('yaw', 0.0))
        self.manual_until_ns = 0
        self.cmd_pub.publish(Twist())
        self.navigator.goToPose(goal)
        self.nav_active = True
        self.pending_say_text = str(say_text) if say_text else None
        self.set_mode('navigating')
        self.store.log_event('navigate_destination', {'query': query, 'resolved': resolved, 'pending_say': self.pending_say_text})
        self.status(f'navigation started to {resolved["name"]} (matched from "{query}")')

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if self.require_localization_for_navigation and not self.localization_fresh():
            self.status('rviz goal blocked: localization stale')
            return
        self.manual_until_ns = 0
        self.cmd_pub.publish(Twist())
        self.navigator.goToPose(msg)
        self.nav_active = True
        self.pending_say_text = None
        self.set_mode('navigating')
        self.store.log_event('navigate_goal_pose', {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'frame_id': msg.header.frame_id})
        self.status('rviz goal accepted')

    def cancel_navigation(self, reason: str) -> None:
        if not self.nav_active:
            return
        self.navigator.cancelTask()
        self.nav_active = False
        self.pending_say_text = None
        self.set_mode('idle')
        self.store.log_event('navigation_cancel', {'reason': reason})
        self.status(f'navigation canceled: {reason}')

    def nav_feedback_tick(self) -> None:
        if not self.nav_active:
            return
        if self.require_localization_for_navigation and not self.localization_fresh():
            self.cancel_navigation('localization became stale')
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                text = 'navigation succeeded'
            elif result == TaskResult.CANCELED:
                text = 'navigation canceled'
            else:
                text = 'navigation failed'
            self.nav_active = False
            self.set_mode('idle')
            self.store.log_event('navigation_result', {'result': text})
            self.status(text)
            if text == 'navigation succeeded' and self.pending_say_text:
                self.speak_pub.publish(String(data=self.pending_say_text))
                self.status(f'post-arrival speech queued: {self.pending_say_text}')
                self.pending_say_text = None
            return
        feedback = self.navigator.getFeedback()
        if feedback is None:
            return
        distance = getattr(feedback, 'distance_remaining', float('nan'))
        eta = getattr(feedback, 'estimated_time_remaining', None)
        eta_sec = getattr(eta, 'sec', None)
        text = f'nav progress: distance_remaining={distance:.2f}m eta={eta_sec}'
        if text != self.last_feedback_text:
            self.last_feedback_text = text
            self.status(text)

    def save_map(self, requested_name: str) -> None:
        new_name = sanitize_name(requested_name)
        old_name = self.active_map()
        if old_name and old_name.startswith('survey_') and new_name != old_name:
            self.store.rename_map(old_name, new_name)
        map_name = new_name
        map_dir = self.store.paths.maps / map_name
        map_dir.mkdir(parents=True, exist_ok=True)
        map_stem = map_dir / map_name
        if not self.save_map_cli.wait_for_service(timeout_sec=1.0):
            self.status('save_map service unavailable; use SlamToolbox RViz plugin if needed')
            return
        request = SaveMap.Request()
        request.map_topic = self.get_parameter('map_topic').value
        request.map_url = f'file://{map_stem}'
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        future = self.save_map_cli.call_async(request)
        future.add_done_callback(lambda fut: self._on_save_map_done(fut, map_name, map_stem))
        self.status(f'saving map to {map_stem}')

    def _on_save_map_done(self, future, map_name: str, map_stem: Path) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.status(f'save_map failed: {exc}')
            return
        if not getattr(result, 'result', False):
            self.status('save_map service returned failure')
            return
        map_yaml = str(Path(f'{map_stem}.yaml'))
        self.store.register_map_artifacts(map_name, map_yaml)
        self.store.update_state(active_map=map_name)
        self.store.log_event('map_saved', {'map_name': map_name, 'map_yaml': map_yaml})
        self.status(f'map saved as {map_name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
