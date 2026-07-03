"""Installation script for the 'docking_rl' python package."""

from setuptools import setup

# Minimum dependencies required prior to installation.
# Isaac Lab itself (isaaclab, isaaclab_assets, isaaclab_tasks, isaaclab_rl, ...) is expected to
# already be installed/importable in the active Python environment (e.g. via IsaacLab's own
# conda env or `isaaclab.sh -p`), so it is intentionally not listed here.
INSTALL_REQUIRES = [
    "psutil",
]

setup(
    name="docking_rl",
    packages=["docking_rl"],
    version="0.1.0",
    description="Isaac Lab RL task: drive a JetRacer from an arbitrary start pose to an AprilTag docking-bay staging pose.",
    install_requires=INSTALL_REQUIRES,
    license="Apache-2.0",
    include_package_data=True,
    python_requires=">=3.10",
    zip_safe=False,
)
