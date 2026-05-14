from setuptools import setup

package_name = 'go2_agentic_motion_skills'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motion_skills.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Grounded Unitree SDK2 motion skills for Sparky.',
    license='MIT',
    entry_points={'console_scripts': [
        'motion_skill_agent_node = go2_agentic_motion_skills.motion_skill_agent_node:main',
    ]},
)
