from glob import glob
import os

from setuptools import find_packages, setup


package_name = "graph_construction"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "configs"), glob("configs/*.yaml")),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WildOS Graph Construction Maintainers",
    maintainer_email="todo@example.com",
    description="Sparse navigation graph construction for WildOS.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "graph_construction = graph_construction.node:main",
        ],
    },
)
