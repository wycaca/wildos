from glob import glob
import os

from setuptools import find_packages, setup


package_name = "wildos_navigation"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WildOS Maintainers",
    maintainer_email="todo@example.com",
    description="GO2 local path following and point cloud collision avoidance",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = wildos_navigation.controller_node:main",
            "velocity_sender = wildos_navigation.velocity_sender:main",
            "velocity_receiver = wildos_navigation.velocity_receiver:main",
        ],
    },
)
