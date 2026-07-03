from glob import glob
import os

from setuptools import find_packages, setup


package_name = "graph_construction"
launch_files = [
    "launch/wildos_2d_sim.launch.py",
    "launch/elevation_visual_navigation_sim.launch.py",
]

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
            launch_files,
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
            "livox_grid_builder = graph_construction.livox_grid_builder:main",
            "grid_map_to_occupancy = graph_construction.grid_map_to_occupancy:main",
            "pointcloud_axis_adapter = graph_construction.pointcloud_axis_adapter:main",
        ],
    },
)
