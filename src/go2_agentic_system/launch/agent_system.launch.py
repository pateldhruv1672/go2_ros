from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('go2_agentic_system')
    params_file = LaunchConfiguration('params_file')
    storage_root = LaunchConfiguration('storage_root')
    open_rviz = LaunchConfiguration('open_rviz')
    start_ui = LaunchConfiguration('start_ui')
    start_keyboard = LaunchConfiguration('start_keyboard')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([package_share, 'config', 'agent_params.yaml'])),
        DeclareLaunchArgument('storage_root', default_value='~/.ros/go2_agent_memory'),
        DeclareLaunchArgument('open_rviz', default_value='false'),
        DeclareLaunchArgument('start_ui', default_value='true'),
        DeclareLaunchArgument('start_keyboard', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([package_share, 'config', 'go2_agent_system.rviz'])),
        Node(
            package='go2_agentic_system', executable='operator_console_node', name='operator_console_node', output='screen',
            condition=IfCondition(start_ui), parameters=[params_file], emulate_tty=True,
        ),
        Node(
            package='go2_agentic_system', executable='keyboard_teleop_node', name='keyboard_teleop_node', output='log',
            condition=IfCondition(start_keyboard), parameters=[params_file],
        ),
        Node(
            package='go2_agentic_system', executable='supervisor_node', name='supervisor_node', output='log',
            parameters=[params_file, {'storage_root': storage_root, 'cmd_vel_topic': cmd_vel_topic}],
        ),
        Node(
            package='go2_agentic_system', executable='semantic_memory_node', name='semantic_memory_node', output='log',
            parameters=[params_file, {'storage_root': storage_root}],
        ),
        Node(
            package='go2_agentic_system', executable='semantic_map_visualizer_node', name='semantic_map_visualizer_node', output='log',
            parameters=[params_file, {'storage_root': storage_root}],
        ),
        Node(
            package='go2_agentic_system', executable='speech_output_node', name='speech_output_node', output='log',
            parameters=[params_file],
        ),
        Node(
            package='rviz2', executable='rviz2', name='go2_agent_rviz', output='log', condition=IfCondition(open_rviz),
            arguments=['--ros-args', '--log-level', 'warn', '-d', rviz_config],
        ),
    ])
