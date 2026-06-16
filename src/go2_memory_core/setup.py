from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_memory_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Dhruv Patel',
    maintainer_email='pateldhruv1672@gmail.com',
    description='Unified file/graph/vector/voxel memory API for Go2 agentic navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'memory_server_node = go2_memory_core.memory_server_node:main',
            'pose_snapshot_node = go2_memory_core.pose_snapshot_node:main',
            'vlm_checkpoint_node = go2_memory_core.vlm_checkpoint_node:main',
        ],
    },
)
