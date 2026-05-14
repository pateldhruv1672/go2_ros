from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .image_utils import image_to_data_url, ros_image_to_pil
from .openrouter_client import OpenRouterClient
from .storage import MemoryStore, sanitize_name, utc_now


class SemanticMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_memory_node')
        self.declare_parameter('storage_root', '~/.ros/go2_agent_memory')
        self.declare_parameter('mode_topic', '/agent/mode')
        self.declare_parameter('status_topic', '/agent/status')
        self.declare_parameter('capture_request_topic', '/agent/capture_semantic')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('compressed_image_topic', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('auto_capture_enabled', True)
        self.declare_parameter('auto_capture_modes', ['survey', 'manual', 'idle'])
        self.declare_parameter('capture_timer_sec', 1.0)
        self.declare_parameter('max_image_side', 1024)

        self.store = MemoryStore(self.get_parameter('storage_root').value)
        guard = self.store.read_guardrails().get('guardrails', {})
        self.capture_distance_m = float(guard.get('capture_distance_m', 1.5))
        self.capture_interval_sec = float(guard.get('capture_interval_sec', 8.0))
        self.auto_capture_enabled = bool(self.get_parameter('auto_capture_enabled').value)
        raw_modes = self.get_parameter('auto_capture_modes').value
        self.auto_capture_modes = {str(item).strip() for item in raw_modes} if isinstance(raw_modes, (list, tuple)) else {'survey', 'manual', 'idle'}

        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 20)
        self.capture_sub = self.create_subscription(String, self.get_parameter('capture_request_topic').value, self.on_capture_request, 20)
        self.mode_sub = self.create_subscription(String, self.get_parameter('mode_topic').value, self.on_mode, 20)
        self.image_sub = self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, 10)
        compressed_topic = self.get_parameter('compressed_image_topic').value
        self.compressed_sub = None
        if compressed_topic:
            self.compressed_sub = self.create_subscription(CompressedImage, compressed_topic, self.on_image, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(float(self.get_parameter('capture_timer_sec').value), self.on_timer)
        self.mode = self.store.read_state().get('mode', 'idle')
        self.latest_image = None
        self.last_capture_pose = None
        self.last_capture_time = None
        self.client = OpenRouterClient()
        self.get_logger().info(f'semantic memory ready; openrouter={self.client.available}; auto_capture_modes={sorted(self.auto_capture_modes)}')

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_mode(self, msg: String) -> None:
        self.mode = msg.data.strip() or 'idle'

    def on_image(self, msg) -> None:
        self.latest_image = msg

    def current_pose(self) -> Optional[dict]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('map_frame').value,
                self.get_parameter('base_frame').value,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z), 1.0 - 2.0 * (r.z * r.z))
        return {
            'frame_id': self.get_parameter('map_frame').value,
            'x': float(t.x),
            'y': float(t.y),
            'z': float(t.z),
            'yaw': float(yaw),
            'stamp': utc_now(),
            'source': 'tf_map_to_base',
        }

    def should_auto_capture(self, pose: Optional[dict]) -> bool:
        if not self.auto_capture_enabled or self.mode not in self.auto_capture_modes or pose is None or self.latest_image is None:
            return False
        now = self.get_clock().now()
        if self.last_capture_time is None:
            return True
        dt = (now - self.last_capture_time).nanoseconds / 1e9
        if dt < self.capture_interval_sec:
            return False
        if self.last_capture_pose is None:
            return True
        dx = pose['x'] - self.last_capture_pose['x']
        dy = pose['y'] - self.last_capture_pose['y']
        return math.hypot(dx, dy) >= self.capture_distance_m

    def on_timer(self) -> None:
        pose = self.current_pose()
        if self.should_auto_capture(pose):
            self.capture_observation(label_hint=None, trigger=f'auto_{self.mode}', pose=pose)

    def on_capture_request(self, msg: String) -> None:
        label = msg.data.strip() or None
        self.capture_observation(label_hint=label, trigger='manual_request', pose=self.current_pose())

    def capture_observation(self, label_hint: Optional[str], trigger: str, pose: Optional[dict]) -> None:
        if pose is None:
            self.status('semantic capture skipped: no map-frame pose available')
            return
        if self.latest_image is None:
            self.status('semantic capture skipped: no camera image available')
            return
        pil = ros_image_to_pil(self.latest_image)
        if pil is None:
            self.status('semantic capture skipped: unsupported image encoding')
            return
        state = self.store.read_state()
        map_name = state.get('active_map') or sanitize_name(state.get('active_survey') or 'unspecified_map')
        img_dir = self.store.paths.images / map_name
        img_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime_safe_filename()
        image_path = img_dir / f'{stamp}.jpg'
        pil.save(image_path, format='JPEG', quality=85)

        label = label_hint or ''
        summary = ''
        aliases = []
        objects = []
        vlm_ok = False
        if self.client.available:
            result = self.client.describe_scene(image_to_data_url(pil, max_side=int(self.get_parameter('max_image_side').value)), hint=label_hint)
            if result.get('ok'):
                parsed = result.get('parsed', {})
                label = label or str(parsed.get('label') or '')
                summary = str(parsed.get('summary') or '')
                aliases = [str(x) for x in (parsed.get('aliases') or []) if str(x).strip()]
                objects = [str(x) for x in (parsed.get('objects') or []) if str(x).strip()]
                vlm_ok = True
            else:
                summary = f"vlm_error: {result.get('error')}"

        obs_id = stamp
        payload = {
            'id': obs_id,
            'ts': utc_now(),
            'trigger': trigger,
            'mode': self.mode,
            'map_name': map_name,
            'pose': pose,
            'label': label,
            'summary': summary,
            'aliases': aliases,
            'objects': objects,
            'image_path': str(image_path),
            'vlm_ok': vlm_ok,
            'label_hint': label_hint,
        }
        self.store.add_observation(payload)
        self.store.log_event('semantic_observation', {'id': obs_id, 'map_name': map_name, 'label': label, 'trigger': trigger, 'image_path': str(image_path), 'vlm_ok': vlm_ok})
        self.last_capture_time = self.get_clock().now()
        self.last_capture_pose = pose
        label_txt = label or '(unlabeled scene)'
        self.status(f'semantic memory saved: {label_txt} @ ({pose["x"]:.2f}, {pose["y"]:.2f})')


def datetime_safe_filename() -> str:
    return utc_now().replace(':', '-').replace('.', '_')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
