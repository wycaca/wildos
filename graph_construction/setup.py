from glob import glob
import os

from setuptools import find_packages, setup


package_name = "graph_construction"
launch_files = [
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
            os.path.join("share", package_name, "configs", "dlio"),
            glob("configs/dlio/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            launch_files,
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
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
            "camera_stamp_adapter = graph_construction.camera_stamp_adapter:main",
            "dlio_output_guard = graph_construction.dlio_output_guard:main",
            "dlio_tf_adapter = graph_construction.dlio_tf_adapter:main",
            "graph_construction = graph_construction.node:main",
            "pointcloud_axis_adapter = graph_construction.pointcloud_axis_adapter:main",
        ],
    },
)
