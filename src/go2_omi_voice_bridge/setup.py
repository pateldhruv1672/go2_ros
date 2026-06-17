from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_omi_voice_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dhruv Patel",
    maintainer_email="pateldhruv1672@gmail.com",
    description="Omi-style voice bridge and safety intent gate for Go2 agentic tour mode.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "omi_ble_bridge_node = go2_omi_voice_bridge.omi_ble_bridge_node:main",
            "stt_node = go2_omi_voice_bridge.stt_node:main",
            "tts_node = go2_omi_voice_bridge.tts_node:main",
            "voice_intent_gate_node = go2_omi_voice_bridge.voice_intent_gate_node:main",
            "tour_voice_command_router = go2_omi_voice_bridge.tour_voice_command_router:main",
        ],
    },
)
