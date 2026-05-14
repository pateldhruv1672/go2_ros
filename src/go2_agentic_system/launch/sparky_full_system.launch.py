from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    net_if = LaunchConfiguration('network_interface')
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value=''),
        Node(package='go2_agentic_multiagent_voice_nav', executable='local_omi_stt_node',
             name='local_omi_stt_node', output='screen'),
        Node(package='go2_agentic_multiagent_voice_nav', executable='dialogue_orchestrator_node',
             name='dialogue_orchestrator_node', output='screen'),
        Node(package='go2_agentic_multiagent_voice_nav', executable='camera_agent_node',
             name='camera_agent_node', output='screen'),
        Node(package='go2_agentic_multiagent_voice_nav', executable='speaker_tts_node',
             name='speaker_tts_node', output='screen'),
        Node(package='go2_agentic_motion_skills', executable='motion_skill_agent_node',
             name='motion_skill_agent_node', output='screen',
             parameters=[{'network_interface': net_if}]),
    ])
