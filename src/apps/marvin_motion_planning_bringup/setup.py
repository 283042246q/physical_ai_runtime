from glob import glob
import os

from setuptools import find_packages, setup

package_name = "marvin_motion_planning_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description="Config-driven Marvin motion-planning bringup.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motion_planning_target_markers = "
            "marvin_motion_planning_bringup.rviz_targets:main",
        ],
    },
)
