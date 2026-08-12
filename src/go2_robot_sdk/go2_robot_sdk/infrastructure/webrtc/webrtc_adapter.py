# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
from typing import Callable, Dict, Any, Optional

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

logger = logging.getLogger(__name__)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(
        self,
        config: RobotConfig,
        on_validated_callback: Callable,
        on_video_frame_callback: Callable = None,
        event_loop=None,
        on_movement_command_callback: Optional[Callable[[str, float, float, float], None]] = None,
    ):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.data_callback: Callable[[RobotData], None] = None
        self.webrtc_msgs = asyncio.Queue()
        self.on_validated_callback = on_validated_callback
        self.on_video_frame_callback = on_video_frame_callback
        self.on_movement_command_callback = on_movement_command_callback
        # Store the event loop (passed from main thread or detect current)
        if event_loop:
            self.main_loop = event_loop
        else:
            try:
                self.main_loop = asyncio.get_running_loop()
            except RuntimeError:
                self.main_loop = None

    async def connect(self, robot_id: str) -> None:
        """Connect to robot via WebRTC"""
        try:
            robot_idx = int(robot_id)
            robot_ip = self.config.robot_ip_list[robot_idx]
            
            conn = Go2Connection(
                robot_ip=robot_ip,
                robot_num=robot_id,
                token=self.config.token,
                on_validated=self._on_validated,
                on_message=self._on_data_channel_message,
                on_video_frame=self.on_video_frame_callback if self.config.enable_video else None,
                decode_lidar=self.config.decode_lidar,
            )
            
            self.connections[robot_id] = conn
            await conn.connect()
            await conn.disableTrafficSaving(True)
            
            logger.info(f"Connected to robot {robot_id} at {robot_ip}")
            
        except Exception as e:
            logger.error(f"Failed to connect to robot {robot_id}: {e}")
            raise

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        if robot_id in self.connections:
            try:
                # Используем правильный метод для закрытия WebRTC соединения
                connection = self.connections[robot_id]
                if hasattr(connection, 'disconnect'):
                    await connection.disconnect()
                elif hasattr(connection, 'pc') and connection.pc:
                    await connection.pc.close()
                del self.connections[robot_id]
                logger.info(f"Disconnected from robot {robot_id}")
            except Exception as e:
                logger.error(f"Error disconnecting from robot {robot_id}: {e}")

    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for data reception"""
        self.data_callback = callback

    def send_command(self, robot_id: str, command: str) -> None:
        """Send command to robot"""
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    # Use asyncio.run_coroutine_threadsafe to handle cross-thread calls
                    loop = self._get_or_create_event_loop()
                    if loop and loop.is_running():
                        # Schedule the coroutine in the existing loop
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        # Fallback to synchronous send
                        connection.data_channel.send(command)
                    logger.debug(f"Command sent to robot {robot_id}: {command[:50]}")
                else:
                    logger.warning(f"No data channel available for robot {robot_id}")
            except Exception as e:
                logger.error(f"Error sending command to robot {robot_id}: {e}")

    def _get_or_create_event_loop(self):
        """Get existing event loop or return the main loop"""
        # First try to get the current loop
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            # If no current loop, return the main loop stored during init
            return self.main_loop

    async def _async_send_command(self, connection, command: str):
        """Async wrapper for sending commands"""
        try:
            if hasattr(connection, 'data_channel') and connection.data_channel:
                connection.data_channel.send(command)
        except Exception as e:
            logger.error(f"Error in async send command: {e}")

    @staticmethod
    def _apply_axis_gain(value: float, gain: float, minimum: float, maximum: float) -> float:
        """Convert ROS Twist-scale values into bounded Go2 sport command values."""
        if value == 0.0:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        scaled = abs(value) * gain
        # IMPORTANT: `minimum` is an actuator deadband, not a command floor.
        # Never turn a tiny DWB correction into a larger physical kick. DWB is
        # configured to sample at/above the same physical minimum when moving.
        if minimum > 0.0 and scaled < minimum:
            return 0.0
        if maximum > 0.0:
            scaled = min(scaled, maximum)
        return sign * scaled

    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        try:
            if self.config.cmd_vel_axis_mode == 'swap_xy':
                sdk_x = y
                sdk_y = x
            else:
                sdk_x = x
                sdk_y = y

            adapted_x = -sdk_x if self.config.cmd_vel_invert_linear_x else sdk_x
            adapted_y = -sdk_y if self.config.cmd_vel_invert_linear_y else sdk_y
            adapted_z = -z if self.config.cmd_vel_invert_angular_z else z
            y_min = self.config.cmd_vel_min_linear_x if self.config.cmd_vel_axis_mode == 'swap_xy' else 0.0
            cmd_x = self._apply_axis_gain(
                adapted_x,
                self.config.cmd_vel_linear_gain,
                self.config.cmd_vel_min_linear_x,
                self.config.cmd_vel_max_linear_x,
            )
            cmd_y = self._apply_axis_gain(
                adapted_y,
                self.config.cmd_vel_linear_gain,
                y_min,
                self.config.cmd_vel_max_linear_x,
            )
            cmd_z = self._apply_axis_gain(
                adapted_z,
                self.config.cmd_vel_angular_gain,
                self.config.cmd_vel_min_angular_z,
                self.config.cmd_vel_max_angular_z,
            )
            command = gen_mov_command(
                round(cmd_x, 2),
                round(cmd_y, 2),
                round(cmd_z, 2),
                self.config.obstacle_avoidance
            )
            if self.on_movement_command_callback:
                self.on_movement_command_callback(
                    robot_id,
                    round(cmd_x, 2),
                    round(cmd_y, 2),
                    round(cmd_z, 2),
                )
            self.send_command(robot_id, command)
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        try:
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.send_command(robot_id, stand_up_cmd)
            
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.send_command(robot_id, move_cmd)
        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        try:
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.send_command(robot_id, stand_down_cmd)
        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Send WebRTC request"""
        try:
            payload = gen_command(api_id, parameter, topic)
            self.webrtc_msgs.put_nowait(payload)
            logger.debug(f"WebRTC request queued for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error sending WebRTC request: {e}")

    def process_webrtc_commands(self, robot_id: str) -> None:
        """Process WebRTC commands from queue"""
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.send_command(robot_id, message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def _on_validated(self, robot_id: str) -> None:
        """Callback after connection validation"""
        try:
            if robot_id in self.connections:
                for topic in RTC_TOPIC.values():
                    self.connections[robot_id].data_channel.send(
                        json.dumps({"type": "subscribe", "topic": topic}))
            
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)
                
        except Exception as e:
            logger.error(f"Error in validated callback: {e}")

    def _on_data_channel_message(self, _, msg: Dict[str, Any], robot_id: str) -> None:
        """Handle incoming data channel messages"""
        try:
            if self.data_callback:
                # Создаем объект RobotData для передачи в callback
                # Фактическая обработка будет в RobotDataService
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)
                self.data_callback(msg, robot_id)  # Передаем сырые данные для обработки
                
        except Exception as e:
            logger.error(f"Error processing data channel message: {e}")
