#!/usr/bin/env python3

from pathlib import Path
import sys
import xml.etree.ElementTree as ET


REQUIRED_DEPENDENCIES = {
    "graph_construction": {
        "ament_index_python",
        "diagnostic_msgs",
        "geometry_msgs",
        "graphnav_msgs",
        "grid_map_msgs",
        "nav_msgs",
        "object_search_msgs",
        "python3-numpy",
        "python3-scipy",
        "python3-skimage",
        "python3-yaml",
        "rclpy",
        "sensor_msgs",
        "std_msgs",
        "tf2_msgs",
        "tf2_ros",
        "wildos_visualization",
    },
    "visual_navigation": {
        "ament_index_python",
        "builtin_interfaces",
        "cv_bridge",
        "geometry_msgs",
        "graphnav_msgs",
        "launch",
        "launch_ros",
        "message_filters",
        "nav_msgs",
        "object_search_msgs",
        "rclpy",
        "sensor_msgs",
        "sensor_msgs_py",
        "std_msgs",
        "tf2_msgs",
        "tf2_ros",
        "triangulation3d",
    },
    "graphnav_planner": {
        "geometry_msgs",
        "graaf_vendor",
        "graphnav_msgs",
        "nav_msgs",
        "object_search_msgs",
        "rclcpp",
        "rclcpp_components",
        "std_msgs",
        "tf2",
        "tf2_geometry_msgs",
        "tf2_ros",
    },
    "wildos_navigation": {
        "ament_index_python",
        "geometry_msgs",
        "launch",
        "launch_ros",
        "motion",
        "nav_msgs",
        "python3-numpy",
        "python3-opencv",
        "python3-scipy",
        "rclpy",
        "ros2launch",
        "sensor_msgs",
        "sensor_msgs_py",
        "tf2_geometry_msgs",
        "tf2_ros",
    },
    "wildos_visualization": {
        "geometry_msgs",
        "graphnav_msgs",
        "nav_msgs",
        "object_search_msgs",
        "rclpy",
        "std_msgs",
        "visualization_msgs",
    },
}


def declared_dependencies(package_xml: Path) -> set[str]:
    root = ET.parse(package_xml).getroot()
    dependency_tags = {
        "depend",
        "build_depend",
        "buildtool_depend",
        "exec_depend",
        "test_depend",
    }
    return {
        element.text.strip()
        for element in root
        if element.tag in dependency_tags and element.text
    }


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    failed = False
    for package, required in REQUIRED_DEPENDENCIES.items():
        declared = declared_dependencies(repo_root / package / "package.xml")
        missing = sorted(required - declared)
        if missing:
            failed = True
            print(f"{package}: missing {', '.join(missing)}", file=sys.stderr)
        else:
            print(f"{package}: manifest dependency contract passed")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
