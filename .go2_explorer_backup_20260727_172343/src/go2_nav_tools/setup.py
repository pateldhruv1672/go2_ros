from setuptools import find_packages, setup

package_name = 'go2_nav_tools'
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
    description='Safe dry-run-first navigation tools for Go2.',
    license='MIT',
    entry_points={'console_scripts': [
            'motion_arbiter = go2_nav_tools.motion_arbiter:main',
        'nav2_tool_server = go2_nav_tools.nav2_tool_server:main',
        'frontier_explorer = go2_nav_tools.frontier_explorer:main',
        'coverage_explorer = go2_nav_tools.coverage_explorer:main',
        'recovery_manager = go2_nav_tools.recovery_manager:main',
    ]},
)
