from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'install',
                             'go2_agentic_multiagent_voice_nav', 'share',
                             'go2_agentic_multiagent_voice_nav')
    cfg = os.path.join(pkg_share, 'config', 'agent_params.yaml')
    return LaunchDescription([
        Node(package='go2_agentic_multiagent_voice_nav', executable='local_omi_stt_node',
             name='local_omi_stt_node', output='screen', parameters=[cfg]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='dialogue_orchestrator_node',
             name='dialogue_orchestrator_node', output='screen', parameters=[cfg]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='camera_agent_node',
             name='camera_agent_node', output='screen', parameters=[cfg]),
        Node(package='go2_agentic_multiagent_voice_nav', executable='speaker_tts_node',
             name='speaker_tts_node', output='screen', parameters=[cfg]),
    ])
