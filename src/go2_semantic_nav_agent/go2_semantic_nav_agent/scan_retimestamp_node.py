from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanRetimestampNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_retimestamp_node')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('stamp_offset_sec', 0.25)
        self.declare_parameter('use_latest_tf_stamp', True)
        # Keep measurement geometry tied to its real acquisition time.
        # /scan_nav remains a QoS bridge; it must not pretend the same ranges
        # were measured at a newer robot pose.
        self.declare_parameter('preserve_input_header', True)
        self.declare_parameter('tf_target_frame', 'odom')
        self.declare_parameter('tf_source_frame', 'base_link')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.stamp_offset_sec = float(self.get_parameter('stamp_offset_sec').value)
        self.use_latest_tf_stamp = bool(self.get_parameter('use_latest_tf_stamp').value)
        self.preserve_input_header = bool(self.get_parameter('preserve_input_header').value)
        self.tf_target_frame = str(self.get_parameter('tf_target_frame').value)
        self.tf_source_frame = str(self.get_parameter('tf_source_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(LaserScan, output_topic, output_qos)
        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_cb, input_qos)
        self.get_logger().info(
            f'Retimestamping {input_topic} -> {output_topic}; input_qos=BEST_EFFORT '
            f'output_qos=RELIABLE preserve_input_header={self.preserve_input_header} '
            f'use_latest_tf_stamp={self.use_latest_tf_stamp}'
        )

    def output_stamp(self):
        if self.use_latest_tf_stamp:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.tf_target_frame,
                    self.tf_source_frame,
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.02),
                )
                if int(tf.header.stamp.sec) > 0 or int(tf.header.stamp.nanosec) > 0:
                    return tf.header.stamp
            except TransformException:
                pass
        stamp = self.get_clock().now() + rclpy.duration.Duration(seconds=self.stamp_offset_sec)
        return stamp.to_msg()

    def scan_cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        if self.preserve_input_header:
            # LaserScan.header.stamp is the measurement acquisition time.
            # Copy it exactly so TF transforms the ranges at the pose where
            # they were actually observed. The input /scan is already in
            # base_link in the Go2 base stack.
            out.header.stamp = msg.header.stamp
            out.header.frame_id = msg.header.frame_id
        else:
            # Legacy behavior retained only as an explicit opt-out for
            # diagnostics/rollback comparisons.
            out.header.stamp = self.output_stamp()
            out.header.frame_id = self.frame_id or msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRetimestampNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
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
