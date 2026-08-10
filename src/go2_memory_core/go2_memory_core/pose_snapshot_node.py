from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import String

from .memory_api import UnifiedMemoryAPI
from .session_resolution import resolve_semantic_session_name


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'on'}
    return bool(value)


class PoseSnapshotNode(Node):
    """Optional checkpoint writer for odom snapshots.

    It is disabled by default because teach/resume integrations should decide when
    a semantic checkpoint is meaningful. Enable `auto_write_checkpoints` only for
    debugging or bag-derived memory generation.
    """

    def __init__(self) -> None:
        super().__init__('go2_pose_snapshot_node')
        self.declare_parameter('session_root', '~/.ros/go2_semantic_nav_sessions')
        self.declare_parameter('session_name', 'default')
        self.declare_parameter('auto_write_checkpoints', False)
        self.declare_parameter('write_period_sec', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # SPARKY_TEACH_BACKGROUND_CHECKPOINTS_V2
        session_root = str(self.get_parameter('session_root').value)
        requested_session = str(self.get_parameter('session_name').value)
        self.session_name = resolve_semantic_session_name(session_root, requested_session)
        self.api = UnifiedMemoryAPI(session_root=session_root)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.latest_odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(String, '/go2_memory/write_snapshot_now', self._on_write_now, 10)
        self.create_timer(float(self.get_parameter('write_period_sec').value), self._timer)
        self.get_logger().info(f'Pose snapshot node ready | session={self.session_name}')

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _timer(self) -> None:
        if _as_bool(self.get_parameter('auto_write_checkpoints').value):
            self._write('timer')

    def _on_write_now(self, msg: String) -> None:
        self._write(msg.data or 'manual')

    def _write(self, source: str) -> None:
        if self.latest_odom is None:
            self.get_logger().warn('No odom received yet; not writing checkpoint')
            return
        odom = self.latest_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        t = odom.twist.twist
        record = {
            'label': f'pose_snapshot_{source}',
            'source': ['odom', 'tf', source],
            'layer': 'temporary',
            'odom_pose': {
                'frame_id': odom.header.frame_id or 'odom',
                'x': p.x, 'y': p.y, 'z': p.z,
                'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
            },
            'velocity_odom': {
                'linear_x': t.linear.x, 'linear_y': t.linear.y, 'linear_z': t.linear.z,
                'angular_x': t.angular.x, 'angular_y': t.angular.y, 'angular_z': t.angular.z,
            },
            'confidence': {'odom_confidence': 1.0, 'localization_confidence': 0.0},
        }
        try:
            map_frame = str(self.get_parameter('map_frame').value or 'map')
            base_frame = str(self.get_parameter('base_frame').value or 'base_link')
            tf = self.tf_buffer.lookup_transform(map_frame, base_frame, Time())
            tr = tf.transform.translation
            rot = tf.transform.rotation
            record['map_pose'] = {'frame_id': map_frame, 'x': tr.x, 'y': tr.y, 'z': tr.z, 'qx': rot.x, 'qy': rot.y, 'qz': rot.z, 'qw': rot.w}
            record['confidence']['localization_confidence'] = 1.0
        except TransformException:
            pass
        result = self.api.write_checkpoint(self.session_name, record)
        self.get_logger().info(json.dumps({'wrote_checkpoint': result.get('id')}, sort_keys=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseSnapshotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
