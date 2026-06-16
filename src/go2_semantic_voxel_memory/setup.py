from setuptools import find_packages, setup

package_name = 'go2_semantic_voxel_memory'
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
    description='Semantic voxel memory store and RViz markers for Go2.',
    license='MIT',
    entry_points={'console_scripts': [
        'semantic_voxel_node = go2_semantic_voxel_memory.semantic_voxel_node:main',
    ]},
)
