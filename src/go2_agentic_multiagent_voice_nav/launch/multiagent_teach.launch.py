from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('go2_agentic_multiagent_voice_nav')
    cfg = PathJoinSubstitution([package_share, 'config', 'agent_params.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('session_root', default_value='~/.ros/go2_semantic_nav_sessions'),
        DeclareLaunchArgument('session_name', default_value=''),
        DeclareLaunchArgument('input_backend', default_value='auto'),
        DeclareLaunchArgument('speaker_tts_enabled', default_value='true'),
        Node(package='go2_agentic_multiagent_voice_nav', executable='local_omi_stt_node',
             name='local_omi_stt_node', output='screen', parameters=[cfg, {
                 'session_root': LaunchConfiguration('session_root'),
                 'session_name': LaunchConfiguration('session_name'),
                 'input_backend': LaunchConfiguration('input_backend'),
             }]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='dialogue_orchestrator_node',
             name='dialogue_orchestrator_node', output='screen', parameters=[cfg, {
                 'session_root': LaunchConfiguration('session_root'),
                 'session_name': LaunchConfiguration('session_name'),
             }]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='camera_agent_node',
             name='camera_agent_node', output='screen', parameters=[cfg]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='speaker_tts_node',
             name='speaker_tts_node', output='screen', parameters=[cfg, {
                 'enabled': LaunchConfiguration('speaker_tts_enabled'),
             }]),
    ])
