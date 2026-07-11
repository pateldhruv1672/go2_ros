from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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
        DeclareLaunchArgument('vlm_provider', default_value='ollama'),
        DeclareLaunchArgument('vlm_model', default_value='gemma4:12b'),
        DeclareLaunchArgument('vlm_base_url', default_value='http://127.0.0.1:11434/api/chat'),
        DeclareLaunchArgument('vlm_timeout_sec', default_value='90.0'),
        DeclareLaunchArgument('vlm_fail_fast_on_start', default_value='true'),
        DeclareLaunchArgument('vlm_shutdown_wait_sec', default_value='300.0'),
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
            on_exit=Shutdown(reason='semantic_nav_node exited'),
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

                'vlm_provider': LaunchConfiguration('vlm_provider'),
                'vlm_model': LaunchConfiguration('vlm_model'),
                'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                'vlm_timeout_sec': LaunchConfiguration('vlm_timeout_sec'),
                'vlm_fail_fast_on_start': LaunchConfiguration('vlm_fail_fast_on_start'),
                'vlm_shutdown_wait_sec': LaunchConfiguration('vlm_shutdown_wait_sec'),
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
