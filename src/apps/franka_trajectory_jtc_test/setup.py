from glob import glob
import os

from setuptools import find_packages, setup


package_name = "franka_trajectory_jtc_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    entry_points={
        "console_scripts": [
            "send_smooth_trajectory = franka_trajectory_jtc_test.send_smooth_trajectory:main",
            "send_mpd_trajectory = franka_trajectory_jtc_test.send_mpd_trajectory:main",
            "move_to_start = franka_trajectory_jtc_test.move_to_start:main",
            "joint_target_gui = franka_trajectory_jtc_test.joint_target_gui:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description="Franka FR3 EM-to-effort-JTC trajectory and joint-GUI validation app.",
    license="Apache-2.0",
)
