from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_label', default_value='session'),
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('auto_save_places', default_value='true'),
        DeclareLaunchArgument('auto_save_interval_sec', default_value='5.0'),
        DeclareLaunchArgument('auto_save_use_vlm', default_value='true'),
        DeclareLaunchArgument('auto_save_target_samples', default_value='10'),
        DeclareLaunchArgument('auto_save_allow_repeat_samples', default_value='true'),
        DeclareLaunchArgument('auto_save_min_distance_m', default_value='0.0'),
        DeclareLaunchArgument('clear_places_on_start', default_value='true'),
        DeclareLaunchArgument('save_map_on_shutdown', default_value='true'),
        DeclareLaunchArgument('semantic_rviz', default_value='true'),
        DeclareLaunchArgument('semantic_rviz_config',
                              default_value=PathJoinSubstitution([
                                  FindPackageShare('go2_semantic_nav_agent'),
                                  'config',
                                  'semantic_nav.rviz',
                              ])),
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
                'auto_save_target_samples': LaunchConfiguration('auto_save_target_samples'),
                'auto_save_allow_repeat_samples': LaunchConfiguration('auto_save_allow_repeat_samples'),
                'auto_save_min_distance_m': LaunchConfiguration('auto_save_min_distance_m'),
                'clear_places_on_start': LaunchConfiguration('clear_places_on_start'),
                'save_map_on_shutdown': LaunchConfiguration('save_map_on_shutdown'),
                'restore_spawn_on_start': False,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='semantic_nav_rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('semantic_rviz_config')],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            condition=IfCondition(LaunchConfiguration('semantic_rviz')),
        ),
    ])
