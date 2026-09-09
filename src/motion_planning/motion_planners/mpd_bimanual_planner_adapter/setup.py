from glob import glob
import os
from setuptools import find_packages, setup

package_name = "mpd_bimanual_planner_adapter"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "bimanual_planner_node = mpd_bimanual_planner_adapter.node:main",
            "marvin_bimanual_one_shot = mpd_bimanual_planner_adapter.one_shot_node:main",
        ]
    },
)
