from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_semantic_nav_agent'

setup(
    name=package_name,
    version='1.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'requests', 'PyYAML', 'Pillow', 'numpy'],
    zip_safe=True,
    maintainer='Dhruv',
    maintainer_email='pateldhruv1672@gmail.com',
    description='Semantic navigation overlay for Go2 with teach/resume sessions and VLM-supervised fallback recovery.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omi_demo_bridge = go2_semantic_nav_agent.omi_demo_bridge:main',
            'semantic_place_markers = go2_semantic_nav_agent.semantic_place_markers:main',
            'semantic_nav_node = go2_semantic_nav_agent.semantic_nav_node:main',
            'semantic_nav_console = go2_semantic_nav_agent.semantic_nav_console:main',
            'scan_retimestamp_node = go2_semantic_nav_agent.scan_retimestamp_node:main',
            'patch_go2_sdk_scan_topics = go2_semantic_nav_agent.patch_go2_sdk_scan_topics:main',
            'semantic_nav_fresh_start = go2_semantic_nav_agent.semantic_nav_fresh_start:main', 'semantic_omi_command_router = go2_semantic_nav_agent.semantic_omi_command_router:main', 'semantic_goal_bridge = go2_semantic_nav_agent.semantic_goal_bridge:main', 'semantic_demo_marker_publisher = go2_semantic_nav_agent.semantic_demo_marker_publisher:main',
        ],
    },
)
