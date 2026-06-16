from setuptools import find_packages, setup

package_name = 'go2_langgraph_agent'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhruv Patel',
    maintainer_email='pateldhruv1672@gmail.com',
    description='Persistent LangGraph-style agent and debate council for Go2.',
    license='MIT',
    entry_points={'console_scripts': [
        'main_supervisor = go2_langgraph_agent.graphs.main_supervisor:main',
        'voice_input_node = go2_langgraph_agent.voice_input_node:main',
        'tts_output_node = go2_langgraph_agent.tts_output_node:main',
    ]},
)
