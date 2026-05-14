from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('go2_agentic_system')
    params_file = LaunchConfiguration('params_file')
    storage_root = LaunchConfiguration('storage_root')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution([package_share, 'config', 'agent_params.yaml']),
            ),
            DeclareLaunchArgument('storage_root', default_value='~/.ros/go2_agent_memory'),
            Node(
                package='go2_agentic_system',
                executable='memory_manager_node',
                name='memory_manager_node',
                output='screen',
                parameters=[params_file, {'storage_root': storage_root}],
            ),
            Node(
                package='go2_agentic_system',
                executable='survey_mode_node',
                name='survey_mode_node',
                output='screen',
                parameters=[params_file, {'storage_root': storage_root}],
            ),
            Node(
                package='go2_agentic_system',
                executable='semantic_map_visualizer_node',
                name='semantic_map_visualizer_node',
                output='screen',
                parameters=[params_file, {'storage_root': storage_root}],
            ),
        ]
    )
