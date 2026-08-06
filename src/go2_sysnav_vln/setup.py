from glob import glob
from setuptools import find_packages, setup

package_name = "go2_sysnav_vln"

setup(
    name=package_name,
    version="0.4.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dhruv Patel",
    maintainer_email="pateldhruv1672@gmail.com",
    description="Hierarchical SysNav-aligned VLN for Unitree Go2 using structured Ollama reasoning and Nav2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "target_conditioned_detector = go2_sysnav_vln.target_conditioned_detector:main",
            "registered_cloud_object_projector = go2_sysnav_vln.registered_cloud_object_projector:main",
            "pose_aware_object_mapper = go2_sysnav_vln.pose_aware_object_mapper:main",
            "hierarchical_frontier_planner = go2_sysnav_vln.hierarchical_frontier_planner:main",
            "vln_supervisor = go2_sysnav_vln.vln_supervisor:main",
            "autonomous_explore_supervisor = go2_sysnav_vln.autonomous_explore_supervisor:main",
            "go2_sysnav_selftest = go2_sysnav_vln.selftest:main",
            "ollama_probe = go2_sysnav_vln.ollama_probe:main",
            "frontier_vision_probe = go2_sysnav_vln.frontier_vision_probe:main",
        ]
    },
)
