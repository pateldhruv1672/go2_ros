
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import logging
import math

from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from go2_interfaces.msg import Go2State, IMU
from go2_interfaces.msg import VoxelMapCompressed
from sensor_msgs.msg import PointCloud2, PointField, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from ...domain.interfaces import IRobotDataPublisher
from ...domain.entities import RobotData, RobotConfig
from ..sensors.lidar_decoder import update_meshes_for_cloud2
from ..sensors.camera_config import load_camera_info

logger = logging.getLogger(__name__)


class ROS2Publisher(IRobotDataPublisher):
    """ROS2 adapter for publishing robot data"""

    def __init__(self, node: Node, config: RobotConfig, publishers: dict, broadcaster: TransformBroadcaster):
        self.node = node
        self.config = config
        self.publishers = publishers
        self.broadcaster = broadcaster
        self.bridge = CvBridge()
        self.camera_info = load_camera_info()
        # Per-robot history for deriving base-frame twist from successive odom poses.
        # The upstream Go2 odometry feed contains pose but the ROS Odometry message
        # previously left twist at zero, which breaks DWB velocity feedback.
        self._odom_twist_history = {}

    def publish_odometry(self, robot_data: RobotData) -> None:
        """Publish odometry data"""
        if not robot_data.odometry_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            
            # Publish transform
            self._publish_transform(robot_data, robot_idx)
            
            # Publish odometry topic
            self._publish_odometry_topic(robot_data, robot_idx)
            
        except Exception as e:
            logger.error(f"Error publishing odometry: {e}")

    def _publish_transform(self, robot_data: RobotData, robot_idx: int) -> None:
        """Publish TF transform"""
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.node.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'

        if self.config.conn_mode == 'single':
            odom_trans.child_frame_id = "base_link"
        else:
            odom_trans.child_frame_id = f"robot{robot_data.robot_id}/base_link"

        position = robot_data.odometry_data.position
        orientation = robot_data.odometry_data.orientation

        odom_trans.transform.translation.x = float(position['x'])
        odom_trans.transform.translation.y = float(position['y'])
        odom_trans.transform.translation.z = float(position['z']) + 0.07

        odom_trans.transform.rotation.x = float(orientation['x'])
        odom_trans.transform.rotation.y = float(orientation['y'])
        odom_trans.transform.rotation.z = float(orientation['z'])
        odom_trans.transform.rotation.w = float(orientation['w'])

        self.broadcaster.sendTransform(odom_trans)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        # Standard yaw extraction, robust to small roll/pitch from the quadruped body.
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _estimate_base_twist(self, robot_idx: int, x: float, y: float, yaw: float, stamp_ns: int):
        """Estimate child/base-frame vx, vy, wz from successive odom poses.

        The raw Go2 odometry stream is pose-rich but this ROS adapter historically
        published a zero twist. DWB uses odometry velocity feedback, so derive it
        here with dt guards, body-frame rotation, wrap-safe yaw difference, and
        light low-pass filtering.
        """
        previous = self._odom_twist_history.get(robot_idx)
        vx_body = vy_body = wz = 0.0
        if previous is not None:
            dt = (stamp_ns - previous['stamp_ns']) * 1e-9
            # The live stream can occasionally deliver duplicate/bursty timestamps.
            if 0.01 <= dt <= 0.75:
                vx_odom = (x - previous['x']) / dt
                vy_odom = (y - previous['y']) / dt
                dyaw = math.atan2(math.sin(yaw - previous['yaw']), math.cos(yaw - previous['yaw']))
                raw_wz = dyaw / dt

                # nav_msgs/Odometry twist is expressed in child_frame_id (base_link).
                c = math.cos(yaw)
                sn = math.sin(yaw)
                raw_vx_body = c * vx_odom + sn * vy_odom
                raw_vy_body = -sn * vx_odom + c * vy_odom

                # Reject impossible spikes caused by pose discontinuities/local resets.
                raw_vx_body = max(-2.0, min(2.0, raw_vx_body))
                raw_vy_body = max(-2.0, min(2.0, raw_vy_body))
                raw_wz = max(-4.0, min(4.0, raw_wz))

                alpha = 0.45
                vx_body = alpha * raw_vx_body + (1.0 - alpha) * previous.get('vx', 0.0)
                vy_body = alpha * raw_vy_body + (1.0 - alpha) * previous.get('vy', 0.0)
                wz = alpha * raw_wz + (1.0 - alpha) * previous.get('wz', 0.0)

        self._odom_twist_history[robot_idx] = {
            'x': x, 'y': y, 'yaw': yaw, 'stamp_ns': stamp_ns,
            'vx': vx_body, 'vy': vy_body, 'wz': wz,
        }
        return vx_body, vy_body, wz

    def _publish_odometry_topic(self, robot_data: RobotData, robot_idx: int) -> None:
        """Publish Odometry pose plus derived base-frame velocity feedback."""
        odom_msg = Odometry()
        now = self.node.get_clock().now()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'

        if self.config.conn_mode == 'single':
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = f"robot{robot_data.robot_id}/base_link"

        position = robot_data.odometry_data.position
        orientation = robot_data.odometry_data.orientation
        x = float(position['x'])
        y = float(position['y'])
        z = float(position['z'])
        qx = float(orientation['x'])
        qy = float(orientation['y'])
        qz = float(orientation['z'])
        qw = float(orientation['w'])

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z + 0.07
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        yaw = self._yaw_from_quaternion(qx, qy, qz, qw)
        vx, vy, wz = self._estimate_base_twist(robot_idx, x, y, yaw, now.nanoseconds)
        odom_msg.twist.twist.linear.x = float(vx)
        odom_msg.twist.twist.linear.y = float(vy)
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(wz)

        # Non-zero diagonal covariance tells consumers this is estimated feedback,
        # rather than falsely claiming perfect certainty.
        odom_msg.twist.covariance[0] = 0.02
        odom_msg.twist.covariance[7] = 0.02
        odom_msg.twist.covariance[35] = 0.03

        self.publishers['odometry'][robot_idx].publish(odom_msg)

    def publish_joint_state(self, robot_data: RobotData) -> None:
        """Publish joint state data"""
        if not robot_data.joint_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()

            # Define joint names
            if self.config.conn_mode == 'single':
                joint_state.name = [
                    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                ]
            else:
                joint_state.name = [
                    f'robot{robot_data.robot_id}/FL_hip_joint', f'robot{robot_data.robot_id}/FL_thigh_joint', f'robot{robot_data.robot_id}/FL_calf_joint',
                    f'robot{robot_data.robot_id}/FR_hip_joint', f'robot{robot_data.robot_id}/FR_thigh_joint', f'robot{robot_data.robot_id}/FR_calf_joint',
                    f'robot{robot_data.robot_id}/RL_hip_joint', f'robot{robot_data.robot_id}/RL_thigh_joint', f'robot{robot_data.robot_id}/RL_calf_joint',
                    f'robot{robot_data.robot_id}/RR_hip_joint', f'robot{robot_data.robot_id}/RR_thigh_joint', f'robot{robot_data.robot_id}/RR_calf_joint'
                ]

            motor_state = robot_data.joint_data.motor_state
            joint_state.position = [
                motor_state[3]['q'], motor_state[4]['q'], motor_state[5]['q'],  # FL leg
                motor_state[0]['q'], motor_state[1]['q'], motor_state[2]['q'],  # FR leg
                motor_state[9]['q'], motor_state[10]['q'], motor_state[11]['q'], # RL leg
                motor_state[6]['q'], motor_state[7]['q'], motor_state[8]['q'],  # RR leg
            ]

            self.publishers['joint_state'][robot_idx].publish(joint_state)

        except Exception as e:
            logger.error(f"Error publishing joint state: {e}")

    def publish_robot_state(self, robot_data: RobotData) -> None:
        """Publish robot state and IMU data"""
        if not robot_data.robot_state:
            return

        try:
            robot_idx = int(robot_data.robot_id)

            # Publish Go2State
            go2_state = Go2State()
            state = robot_data.robot_state
            go2_state.mode = state.mode
            go2_state.progress = state.progress
            go2_state.gait_type = state.gait_type
            go2_state.position = list(map(float, state.position))
            go2_state.body_height = float(state.body_height)
            go2_state.velocity = state.velocity
            go2_state.range_obstacle = list(map(float, state.range_obstacle))
            go2_state.foot_force = state.foot_force
            go2_state.foot_position_body = list(map(float, state.foot_position_body))
            go2_state.foot_speed_body = list(map(float, state.foot_speed_body))
            
            self.publishers['robot_state'][robot_idx].publish(go2_state)

            # Publish IMU
            if robot_data.imu_data:
                imu = IMU()
                imu_data = robot_data.imu_data
                imu.quaternion = list(map(float, imu_data.quaternion))
                imu.accelerometer = list(map(float, imu_data.accelerometer))
                imu.gyroscope = list(map(float, imu_data.gyroscope))
                imu.rpy = list(map(float, imu_data.rpy))
                imu.temperature = imu_data.temperature
                
                self.publishers['imu'][robot_idx].publish(imu)

        except Exception as e:
            logger.error(f"Error publishing robot state: {e}")

    def publish_lidar_data(self, robot_data: RobotData) -> None:
        """Publish lidar data"""
        if not robot_data.lidar_data or not self.config.decode_lidar:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            lidar = robot_data.lidar_data

            points = update_meshes_for_cloud2(
                lidar.positions,
                lidar.uvs,
                lidar.resolution,
                lidar.origin,
                0
            )

            point_cloud = PointCloud2()
            point_cloud.header = Header(frame_id="odom")
            point_cloud.header.stamp = self.node.get_clock().now().to_msg()
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
            self.publishers['lidar'][robot_idx].publish(point_cloud)

        except Exception as e:
            logger.error(f"Error publishing lidar data: {e}")

    def publish_camera_data(self, robot_data: RobotData) -> None:
        """Publish camera data"""
        if not robot_data.camera_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            camera = robot_data.camera_data

            # Convert to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(camera.image, encoding=camera.encoding)
            ros_image.header.stamp = self.node.get_clock().now().to_msg()

            # Camera info
            camera_info = self.camera_info[camera.height]
            camera_info.header.stamp = ros_image.header.stamp

            if self.config.conn_mode == 'single':
                camera_info.header.frame_id = 'front_camera'
                ros_image.header.frame_id = 'front_camera'
            else:
                camera_info.header.frame_id = f'robot{robot_data.robot_id}/front_camera'
                ros_image.header.frame_id = f'robot{robot_data.robot_id}/front_camera'

            # Publish
            self.publishers['camera'][robot_idx].publish(ros_image)
            self.publishers['camera_info'][robot_idx].publish(camera_info)

        except Exception as e:
            logger.error(f"Error publishing camera data: {e}")

    def publish_voxel_data(self, robot_data: RobotData) -> None:
        """Publish voxel data"""
        if not robot_data.lidar_data or not self.config.publish_raw_voxel:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            lidar = robot_data.lidar_data

            voxel_msg = VoxelMapCompressed()
            voxel_msg.stamp = float(lidar.stamp)
            voxel_msg.frame_id = 'odom'
            voxel_msg.resolution = lidar.resolution
            voxel_msg.origin = lidar.origin
            voxel_msg.width = lidar.width or []
            voxel_msg.src_size = lidar.src_size or 0
            voxel_msg.data = lidar.compressed_data or b''

            self.publishers['voxel'][robot_idx].publish(voxel_msg)

        except Exception as e:
            logger.error(f"Error publishing voxel data: {e}") 