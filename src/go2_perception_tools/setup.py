from setuptools import find_packages, setup

package_name = 'go2_perception_tools'
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
    description='Geometry perception summaries for Go2 LiDAR and point clouds.',
    license='MIT',
    entry_points={'console_scripts': [
        'lidar_geometry_node = go2_perception_tools.lidar_geometry_node:main',
        'pointcloud_analyzer_node = go2_perception_tools.pointcloud_analyzer_node:main',
        'traversability_node = go2_perception_tools.traversability_node:main',
        'open_vocab_detector_node = go2_perception_tools.open_vocab_detector_node:main',
        'dynamic_obstacle_tracker = go2_perception_tools.dynamic_obstacle_tracker:main',
    ]},
)
