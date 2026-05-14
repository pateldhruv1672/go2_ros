from setuptools import setup
from glob import glob

package_name = 'go2_colmap_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Export Go2 ROS 2 recordings into COLMAP datasets and prepare environments for Isaac Sim.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_dataset = go2_colmap_pipeline.bag_to_dataset:main',
            'run_colmap = go2_colmap_pipeline.run_colmap:main',
            'mesh_to_obj = go2_colmap_pipeline.mesh_to_obj:main',
            'full_pipeline = go2_colmap_pipeline.full_pipeline:main',
        ],
    },
)
