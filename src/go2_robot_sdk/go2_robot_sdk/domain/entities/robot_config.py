# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass
from typing import List


@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    robot_ip_list: List[str]
    token: str
    conn_type: str
    enable_video: bool
    decode_lidar: bool
    publish_raw_voxel: bool
    obstacle_avoidance: bool
    cmd_vel_linear_gain: float
    cmd_vel_angular_gain: float
    cmd_vel_min_linear_x: float
    cmd_vel_min_angular_z: float
    cmd_vel_max_linear_x: float
    cmd_vel_max_angular_z: float
    cmd_vel_axis_mode: str
    cmd_vel_invert_linear_x: bool
    cmd_vel_invert_linear_y: bool
    cmd_vel_invert_angular_z: bool
    conn_mode: str  # 'single' or 'multi'

    @classmethod
    def from_params(cls, robot_ip: str, token: str, conn_type: str, 
                   enable_video: bool, decode_lidar: bool, 
                   publish_raw_voxel: bool, obstacle_avoidance: bool,
                   cmd_vel_linear_gain: float = 4.0,
                   cmd_vel_angular_gain: float = 0.8,
                   cmd_vel_min_linear_x: float = 0.22,
                   cmd_vel_min_angular_z: float = 0.12,
                   cmd_vel_max_linear_x: float = 0.40,
                   cmd_vel_max_angular_z: float = 0.80,
                   cmd_vel_axis_mode: str = 'standard',
                   cmd_vel_invert_linear_x: bool = False,
                   cmd_vel_invert_linear_y: bool = False,
                   cmd_vel_invert_angular_z: bool = False):
        """Создание конфигурации из параметров"""
        robot_ip_list = robot_ip.replace(" ", "").split(",")
        conn_mode = "single" if (
            len(robot_ip_list) == 1 and conn_type != "cyclonedds") else "multi"
        
        return cls(
            robot_ip_list=robot_ip_list,
            token=token,
            conn_type=conn_type,
            enable_video=enable_video,
            decode_lidar=decode_lidar,
            publish_raw_voxel=publish_raw_voxel,
            obstacle_avoidance=obstacle_avoidance,
            cmd_vel_linear_gain=cmd_vel_linear_gain,
            cmd_vel_angular_gain=cmd_vel_angular_gain,
            cmd_vel_min_linear_x=cmd_vel_min_linear_x,
            cmd_vel_min_angular_z=cmd_vel_min_angular_z,
            cmd_vel_max_linear_x=cmd_vel_max_linear_x,
            cmd_vel_max_angular_z=cmd_vel_max_angular_z,
            cmd_vel_axis_mode=cmd_vel_axis_mode,
            cmd_vel_invert_linear_x=cmd_vel_invert_linear_x,
            cmd_vel_invert_linear_y=cmd_vel_invert_linear_y,
            cmd_vel_invert_angular_z=cmd_vel_invert_angular_z,
            conn_mode=conn_mode
        )
