from setuptools import setup
from glob import glob
import os

package_name = "so_arm_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    # pupil-apriltags = official AprilRobotics apriltag3 detector (Python binding),
    # used by apriltag_detector.py. Not a rosdep; install with:
    #   /usr/bin/python3 -m pip install --user pupil-apriltags
    install_requires=["setuptools", "pupil-apriltags"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@email.com",
    description="SO-ARM 101 perception",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = so_arm_perception.perception_node:main",
            "tracking_node = so_arm_perception.tracking_node:main",
            "apriltag_node = so_arm_perception.apriltag_node:main",
            "handle_detector = so_arm_perception.handle_detector:main",
        ],
    },
)
