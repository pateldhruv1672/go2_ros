from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_label', default_value='session'),
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('auto_save_places', default_value='true'),
        DeclareLaunchArgument('auto_save_interval_sec', default_value='5.0'),
        DeclareLaunchArgument('auto_save_use_vlm', default_value='true'),
        DeclareLaunchArgument('clear_places_on_start', default_value='true'),
        Node(
            package='go2_semantic_nav_agent',
            executable='scan_retimestamp_node',
            name='scan_retimestamp_node',
            output='screen',
        ),
        Node(
            package='go2_semantic_nav_agent',
            executable='semantic_nav_node',
            name='semantic_nav_node',
            output='screen',
            parameters=[{
                'mode': 'teach',
                'map_label': LaunchConfiguration('map_label'),
                'session_root': LaunchConfiguration('session_root'),
                'auto_save_places': LaunchConfiguration('auto_save_places'),
                'auto_save_interval_sec': LaunchConfiguration('auto_save_interval_sec'),
                'auto_save_use_vlm': LaunchConfiguration('auto_save_use_vlm'),
                'clear_places_on_start': LaunchConfiguration('clear_places_on_start'),
                'restore_spawn_on_start': False,
            }],
        ),
    ])
