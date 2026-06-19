from setuptools import setup
from glob import glob
import os

package_name = 'go2_agentic_system'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='user@example.com',
    description='Top-level launch wrapper for Sparky agentic system.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'command_console_node = go2_agentic_system.command_console_node:main',
            'keyboard_teleop_node = go2_agentic_system.keyboard_teleop_node:main',
            'memory_manager_node = go2_agentic_system.memory_manager_node:main',
            'navigation_agent_node = go2_agentic_system.navigation_agent_node:main',
            'operator_console_node = go2_agentic_system.operator_console_node:main',
            'safety_supervisor_node = go2_agentic_system.safety_supervisor_node:main',
            'semantic_map_visualizer_node = go2_agentic_system.semantic_map_visualizer_node:main',
            'semantic_memory_node = go2_agentic_system.semantic_memory_node:main',
            'speech_output_node = go2_agentic_system.speech_output_node:main',
            'supervisor_node = go2_agentic_system.supervisor_node:main',
            'survey_mode_node = go2_agentic_system.survey_mode_node:main',
            'task_planner_node = go2_agentic_system.task_planner_node:main',
            'voice_command_node = go2_agentic_system.voice_command_node:main',
        ],
    },
)
