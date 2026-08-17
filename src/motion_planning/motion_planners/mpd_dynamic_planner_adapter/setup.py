from glob import glob
import os

from setuptools import find_packages, setup


package_name = "mpd_dynamic_planner_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description="Fixed-timing dynamic-world replanning extension for MPD.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dynamic_replan_node = mpd_dynamic_planner_adapter.replan_node:main",
            "dynamic_world_demo = mpd_dynamic_planner_adapter.world_demo_node:main",
        ],
    },
)
