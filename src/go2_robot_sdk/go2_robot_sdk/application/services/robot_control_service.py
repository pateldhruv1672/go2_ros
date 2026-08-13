# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import json
import os
import logging
import time


from ...domain.interfaces import IRobotController
from ..utils.command_generator import gen_mov_command
from ...domain.constants import RTC_TOPIC


logger = logging.getLogger(__name__)


class RobotControlService:
    """Service for robot control"""

    def __init__(self, controller: IRobotController):
        self.controller = controller
        # SPARKY_LOCOMOTION_REARM_V13_3
        self._last_nonzero_cmd_mono = {}
        self._last_locomotion_arm_mono = {}
        try:
            self._locomotion_rearm_idle_sec = max(0.5, float(os.environ.get('GO2_LOCOMOTION_REARM_IDLE_SEC', '2.0')))
        except Exception:
            self._locomotion_rearm_idle_sec = 2.0

    def handle_cmd_vel(self, x: float, y: float, z: float, robot_id: str, obstacle_avoidance: bool = False) -> None:
        """Process movement command"""
        try:
            _ = gen_mov_command(
                round(x, 2), 
                round(y, 2), 
                round(z, 2), 
                obstacle_avoidance
            )
            moving = abs(float(x)) > 1e-6 or abs(float(y)) > 1e-6 or abs(float(z)) > 1e-6
            now_mono = time.monotonic()
            if moving:
                last_nonzero = float(self._last_nonzero_cmd_mono.get(robot_id, -1e9))
                last_arm = float(self._last_locomotion_arm_mono.get(robot_id, -1e9))
                idle_for = now_mono - last_nonzero
                if idle_for >= self._locomotion_rearm_idle_sec and now_mono - last_arm >= self._locomotion_rearm_idle_sec:
                    # Existing controller path sends Unitree StandUp then BalanceStand.
                    self.controller.send_stand_up_command(robot_id)
                    self._last_locomotion_arm_mono[robot_id] = now_mono
                    logger.info('Re-armed Go2 locomotion posture before non-zero cmd_vel after %.2fs idle', idle_for)
                self._last_nonzero_cmd_mono[robot_id] = now_mono
            self.controller.send_movement_command(robot_id, x, y, z)
        except Exception as e:
            logger.error(f"Error handling cmd_vel: {e}")

    def handle_webrtc_request(self, api_id: int, parameter_str: str, topic: str, msg_id: str, robot_id: str) -> None:
        """Process WebRTC request"""
        try:
            parameter = "" if parameter_str == "" else json.loads(parameter_str)
            self.controller.send_webrtc_request(robot_id, api_id, parameter, topic)
            logger.info(f"WebRTC request sent to robot {robot_id}")
        except ValueError as e:
            logger.error(f"Invalid JSON in WebRTC request: {e}")
        except Exception as e:
            logger.error(f"Error handling WebRTC request: {e}")

    def handle_joy_command(self, joy_buttons: list, robot_id: str) -> None:
        """Process joystick commands"""
        try:
            if joy_buttons and len(joy_buttons) > 1:
                if joy_buttons[1]:  # Stand down
                    self.controller.send_stand_down_command(robot_id)
                    logger.info(f"Stand down command sent to robot {robot_id}")
                
                elif joy_buttons[0]:  # Stand up
                    self.controller.send_stand_up_command(robot_id)
                    logger.info(f"Stand up command sent to robot {robot_id}")

        except Exception as e:
            logger.error(f"Error handling joy command: {e}")

    def set_obstacle_avoidance(self, enabled: bool, robot_id: str) -> None:
        """Set obstacle avoidance mode"""
        try:
            self.controller.send_webrtc_request(
                robot_id, 
                1004, 
                {"is_remote_commands_from_api": enabled},
                RTC_TOPIC['OBSTACLES_AVOID']
            )
            logger.info(f"Obstacle avoidance set to {enabled} for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error setting obstacle avoidance: {e}") 