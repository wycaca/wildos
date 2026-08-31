from glob import glob
import os

from setuptools import find_packages, setup


package_name = "wildos_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "configs"), glob("configs/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WildOS Maintainers",
    maintainer_email="todo@example.com",
    description="Independent RViz visualization for WildOS",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "graph_visualizer = wildos_visualization.graph_visualizer:main",
        ],
    },
)
