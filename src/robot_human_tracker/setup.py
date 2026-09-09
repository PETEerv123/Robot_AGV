from setuptools import setup, find_packages
import os
from glob import glob

package_name = "robot_human_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_human_tracker"]),
        ("share/robot_human_tracker", ["package.xml"]),
        ("share/robot_human_tracker/models", glob("models/*")),
        ("share/robot_human_tracker/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="tathuan9d@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "depth_tracker = robot_human_tracker.depth_tracker:main",
            "go2_follower  = robot_human_tracker.go2_follower_nav2:main",
        ],
    },
)
