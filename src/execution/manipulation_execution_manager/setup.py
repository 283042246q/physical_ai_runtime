from glob import glob

from setuptools import find_packages, setup


package_name = "manipulation_execution_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "LICENSE", "CHANGELOG.md"],
        ),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.12",
    zip_safe=False,
    maintainer="Gabriel-Ning",
    maintainer_email="guomning@gmail.com",
    description=(
        "Generic ROS 2 execution manager for manipulation source arbitration, "
        "normalization, reference routing, and diagnostics."
    ),
    license="Apache-2.0",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering",
    ],
    entry_points={
        "console_scripts": [
            "execution_manager = manipulation_execution_manager.execution_manager_node:main",
        ],
    },
)
