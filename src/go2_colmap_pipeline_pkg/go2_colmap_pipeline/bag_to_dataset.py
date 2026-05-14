from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .common import configure_logging, detect_storage_id, ensure_dir, write_csv, write_yaml, LOGGER
from .transforms import Pose, nearest_pose, pose_to_matrix, transform_pose

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise ImportError('OpenCV is required. Install python3-opencv in your ROS environment.') from exc


IMAGE_EXTENSIONS = {
    'jpg': '.jpg',
    'jpeg': '.jpg',
    'png': '.png',
}


class BagExporter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.bridge = CvBridge()
        self.camera_info: Optional[Dict[str, Any]] = None
        self.poses: list[Pose] = []
        self.image_rows: list[dict[str, Any]] = []
        self.image_count = 0
        self.last_export_stamp_ns: Optional[int] = None
        self.base_t_camera = self._load_base_t_camera(args.camera_extrinsics)
        self.bag_path = Path(self.args.bag)
        self.storage_id = self.args.storage_id if self.args.storage_id != 'auto' else detect_storage_id(self.bag_path)
        self.topic_types: dict[str, str] = {}

    def _load_base_t_camera(self, path: Optional[str]) -> Optional[np.ndarray]:
        if not path:
            return None
        with Path(path).open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        translation = data.get('translation_xyz', [0.0, 0.0, 0.0])
        quaternion = data.get('quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        return pose_to_matrix(translation, quaternion)

    def _open_reader(self) -> SequentialReader:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(self.bag_path), storage_id=self.storage_id),
            ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
        )
        if not self.topic_types:
            self.topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        return reader

    def _deserialize(self, topic_name: str, data: bytes) -> Any:
        msg_type = get_message(self.topic_types[topic_name])
        return deserialize_message(data, msg_type)

    def _message_stamp_ns(self, msg: Any) -> int:
        stamp = getattr(msg, 'header', None)
        if stamp is not None:
            return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        raise ValueError('Message does not contain a header with a timestamp.')

    def _camera_info_dict(self, msg: CameraInfo) -> Dict[str, Any]:
        return {
            'width': int(msg.width),
            'height': int(msg.height),
            'distortion_model': msg.distortion_model,
            'k': [float(v) for v in msg.k],
            'd': [float(v) for v in msg.d],
            'r': [float(v) for v in msg.r],
            'p': [float(v) for v in msg.p],
        }

    def _pose_from_odom(self, msg: Odometry) -> Pose:
        pose = msg.pose.pose
        stamp_ns = self._message_stamp_ns(msg)
        return Pose(
            stamp_ns=stamp_ns,
            translation=np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ], dtype=float),
            quaternion_xyzw=np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ], dtype=float),
        )

    def _should_export(self, stamp_ns: int) -> bool:
        if self.last_export_stamp_ns is None:
            return True
        dt = stamp_ns - self.last_export_stamp_ns
        return dt >= int(self.args.min_dt_sec * 1_000_000_000)

    def _save_image(self, msg: Image | CompressedImage) -> Optional[Path]:
        stamp_ns = self._message_stamp_ns(msg)
        if not self._should_export(stamp_ns):
            return None
        if isinstance(msg, Image):
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        ext = IMAGE_EXTENSIONS[self.args.image_format]
        image_name = f'frame_{self.image_count:06d}{ext}'
        image_path = Path(self.args.output_dir) / 'images' / image_name
        ensure_dir(image_path.parent)
        if self.args.image_format in ('jpg', 'jpeg'):
            cv2.imwrite(str(image_path), image, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)])
        else:
            cv2.imwrite(str(image_path), image)

        pose = nearest_pose(self.poses, stamp_ns, max_dt_ns=int(self.args.max_pose_gap_sec * 1_000_000_000))
        if pose is not None:
            pose = transform_pose(pose, self.base_t_camera)
            xyz = pose.translation.tolist()
            quat = pose.quaternion_xyzw.tolist()
        else:
            xyz = [None, None, None]
            quat = [None, None, None, None]

        self.image_rows.append({
            'image_name': image_name,
            'stamp_ns': stamp_ns,
            'x': xyz[0],
            'y': xyz[1],
            'z': xyz[2],
            'qx': quat[0],
            'qy': quat[1],
            'qz': quat[2],
            'qw': quat[3],
        })
        self.image_count += 1
        self.last_export_stamp_ns = stamp_ns
        return image_path

    def _load_static_camera_info(self) -> None:
        if not self.args.camera_info_yaml:
            return
        with Path(self.args.camera_info_yaml).open('r', encoding='utf-8') as handle:
            camera_info_yaml = yaml.safe_load(handle)
        self.camera_info = {
            'width': camera_info_yaml['image_width'],
            'height': camera_info_yaml['image_height'],
            'distortion_model': camera_info_yaml['distortion_model'],
            'k': [float(v) for v in camera_info_yaml['camera_matrix']['data']],
            'd': [float(v) for v in camera_info_yaml['distortion_coefficients']['data']],
            'r': [float(v) for v in camera_info_yaml['rectification_matrix']['data']],
            'p': [float(v) for v in camera_info_yaml['projection_matrix']['data']],
        }

    def scan_bag(self) -> None:
        self._load_static_camera_info()
        reader = self._open_reader()

        if self.args.image_topic not in self.topic_types:
            raise RuntimeError(f'Image topic not found in bag: {self.args.image_topic}')
        if self.args.odom_topic and self.args.odom_topic not in self.topic_types:
            LOGGER.warning('Odometry topic %s not found. Position priors will be skipped.', self.args.odom_topic)

        while reader.has_next():
            topic_name, data, _ = reader.read_next()
            if topic_name == self.args.odom_topic:
                msg = self._deserialize(topic_name, data)
                if isinstance(msg, Odometry):
                    self.poses.append(self._pose_from_odom(msg))
            elif topic_name == self.args.camera_info_topic and self.camera_info is None:
                msg = self._deserialize(topic_name, data)
                if isinstance(msg, CameraInfo):
                    self.camera_info = self._camera_info_dict(msg)

    def export_images(self) -> None:
        output_dir = Path(self.args.output_dir)
        ensure_dir(output_dir)
        ensure_dir(output_dir / 'images')
        reader = self._open_reader()
        while reader.has_next():
            topic_name, data, _ = reader.read_next()
            if topic_name != self.args.image_topic:
                continue
            msg = self._deserialize(topic_name, data)
            if isinstance(msg, (Image, CompressedImage)):
                self._save_image(msg)

    def export(self) -> None:
        self.scan_bag()
        if self.camera_info is None:
            raise RuntimeError('No camera calibration found. Provide --camera-info-yaml or record /camera/camera_info.')
        self.export_images()

        output_dir = Path(self.args.output_dir)
        write_yaml(output_dir / 'camera_info.yaml', self.camera_info)
        write_csv(
            output_dir / 'image_metadata.csv',
            self.image_rows,
            ['image_name', 'stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'],
        )

        ref_images_path = output_dir / 'ref_images.txt'
        with ref_images_path.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.writer(handle, delimiter=' ')
            for row in self.image_rows:
                if row['x'] is None:
                    continue
                writer.writerow([row['image_name'], row['x'], row['y'], row['z']])

        summary = {
            'image_topic': self.args.image_topic,
            'camera_info_topic': self.args.camera_info_topic,
            'odom_topic': self.args.odom_topic,
            'image_count': self.image_count,
            'pose_count': len(self.poses),
            'camera_extrinsics': self.args.camera_extrinsics,
            'min_dt_sec': self.args.min_dt_sec,
            'max_pose_gap_sec': self.args.max_pose_gap_sec,
        }
        write_yaml(output_dir / 'export_summary.yaml', summary)
        LOGGER.info('Exported %d images into %s', self.image_count, output_dir)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Export a ROS 2 bag into a COLMAP-ready dataset.')
    parser.add_argument('--bag', required=True, help='Path to rosbag2 directory.')
    parser.add_argument('--output-dir', required=True, help='Directory to write the dataset into.')
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--camera-info-yaml', default='')
    parser.add_argument('--camera-extrinsics', default='')
    parser.add_argument('--storage-id', default='auto', choices=['auto', 'sqlite3', 'mcap'])
    parser.add_argument('--image-format', default='jpg', choices=['jpg', 'jpeg', 'png'])
    parser.add_argument('--jpeg-quality', default=95, type=int)
    parser.add_argument('--min-dt-sec', default=0.25, type=float, help='Minimum time between exported frames.')
    parser.add_argument('--max-pose-gap-sec', default=0.50, type=float, help='Maximum image-to-pose time delta.')
    parser.add_argument('--verbose', action='store_true')
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.verbose)
    exporter = BagExporter(args)
    exporter.export()


if __name__ == '__main__':
    main()
