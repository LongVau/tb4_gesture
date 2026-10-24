from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tb4_gesture"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.task")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sumika",
    maintainer_email="sumika@todo.todo",
    description="Assignment 6: TurtleBot 4 Hand Gesture Recognition and Control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gesture_recognition_node = tb4_gesture.gesture_recognition_node:main",
            "robot_behavior_node = tb4_gesture.robot_behavior_node:main",
        ],
    },
)
