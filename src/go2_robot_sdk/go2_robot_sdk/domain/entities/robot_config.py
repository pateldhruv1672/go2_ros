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
    conn_mode: str  # 'single' or 'multi'

    @classmethod
    def from_params(cls, robot_ip: str, token: str, conn_type: str, 
                   enable_video: bool, decode_lidar: bool, 
                   publish_raw_voxel: bool, obstacle_avoidance: bool,
                   cmd_vel_linear_gain: float = 1.0,
                   cmd_vel_angular_gain: float = 1.0,
                   cmd_vel_min_linear_x: float = 0.0,
                   cmd_vel_min_angular_z: float = 0.0,
                   cmd_vel_max_linear_x: float = 0.5,
                   cmd_vel_max_angular_z: float = 1.0):
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
            conn_mode=conn_mode
        )
