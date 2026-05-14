from setuptools import setup

package_name = 'go2_agentic_multiagent_voice_nav'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/multiagent_resume.launch.py',
            'launch/multiagent_teach.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/agent_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Omi + local STT + orchestrator + camera agent + TTS overlay for Sparky.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'local_omi_stt_node = go2_agentic_multiagent_voice_nav.local_omi_stt_node:main',
            'dialogue_orchestrator_node = go2_agentic_multiagent_voice_nav.dialogue_orchestrator_node:main',
            'camera_agent_node = go2_agentic_multiagent_voice_nav.camera_agent_node:main',
            'speaker_tts_node = go2_agentic_multiagent_voice_nav.speaker_tts_node:main',
        ],
    },
)
