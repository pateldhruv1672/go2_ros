from glob import glob
import os

from setuptools import find_packages, setup

package_name = "go2_object_explorer"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),

        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dhruv Patel",
    maintainer_email="pateldhruv1672@gmail.com",
    description="Frontier-based Omi ObjectNav explorer for Go2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nav2_diagnostic_logger_node = go2_object_explorer.nav2_diagnostic_logger_node:main",
            "nav2_readiness_activator_node = go2_object_explorer.nav2_readiness_activator_node:main",
            "annotated_image_qos_relay_node = go2_object_explorer.annotated_image_qos_relay_node:main",
            "scan_retimestamp_node = go2_object_explorer.scan_retimestamp_node:main",
            "safe_frontier_filter_node = go2_object_explorer.safe_frontier_filter_node:main",
            "topic_web_dashboard_node = go2_object_explorer.topic_web_dashboard_node:main",
            "mrkl_explorer_agent_node = go2_object_explorer.mrkl_explorer_agent_node:main",
            "fast_sam2_tracker_overlay_node = go2_object_explorer.fast_sam2_tracker_overlay_node:main",
            "sam2_tracker_overlay_node = go2_object_explorer.sam2_tracker_overlay_node:main",
            "object_rviz_overlay_node = go2_object_explorer.object_rviz_overlay_node:main",
            "frontier_object_explorer_node = go2_object_explorer.frontier_object_explorer_node:main",
            "padded_map_node = go2_object_explorer.padded_map_node:main",
        ],
    },
)
