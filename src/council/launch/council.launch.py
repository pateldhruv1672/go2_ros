#!/usr/bin/env python3
"""
Launch file for Council Multi-Agent Navigation System
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/point_cloud2',
        description='LiDAR point cloud topic'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU data topic'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel_joy',
        description='Velocity command topic'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug output'
    )
    
    # Council node
    council_node = Node(
        package='council',
        executable='council_node',
        name='council_agent',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'debug': LaunchConfiguration('debug'),
        }],
    )
    
    return LaunchDescription([
        camera_topic_arg,
        lidar_topic_arg,
        imu_topic_arg,
        cmd_vel_topic_arg,
        debug_arg,
        council_node,
    ])
