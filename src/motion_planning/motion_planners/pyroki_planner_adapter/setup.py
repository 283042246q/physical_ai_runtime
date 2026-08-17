from setuptools import find_packages, setup

package_name = "pyroki_planner_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description=(
        "PyRoki planner adapter: J-PARSE setpoint IK, trajopt trajectory, "
        "and online horizon MPC."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            (
                "pyroki_global_setpoint_planner = "
                "pyroki_planner_adapter.setpoint_node:main"
            ),
            "pyroki_online_mpc_planner = pyroki_planner_adapter.mpc_node:main",
        ],
    },
)
