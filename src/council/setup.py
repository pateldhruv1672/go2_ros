from setuptools import setup
import os
from glob import glob

package_name = 'council'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        package_name + '.agents',
        package_name + '.ros_interfaces',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Council Team',
    maintainer_email='council@example.com',
    description='Multi-Agent Council Navigation System for Unitree Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'council_node = council.main:main',
            'voice_controller = council.voice_controller:main',
        ],
    },
)
