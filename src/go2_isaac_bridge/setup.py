from setuptools import setup

package_name = "go2_isaac_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/isaac_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dhruv Patel",
    maintainer_email="dhruvkumarkamleshbhai.patel@sjsu.edu",
    description="ROS-side bridge between go2_ros and IsaacLab Go2 backend.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "go2_isaac_bridge_node = go2_isaac_bridge.udp_bridge_node:main",
        ],
    },
)
