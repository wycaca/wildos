from __future__ import annotations

import os
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


def main() -> None:
    """Wait for one CameraInfo message from every configured camera"""
    camera_namespace = os.environ.get(
        "CAMERA_NAMESPACE",
        "spot1/realsense",
    ).strip("/")
    camera_names = ("front", "left", "right")
    timeout_seconds = float(os.environ.get("CAMERA_HEALTH_TIMEOUT", "6"))
    received: set[str] = set()

    rclpy.init()
    node = rclpy.create_node("wildos_camera_healthcheck")
    subscriptions = []

    # Keep subscriptions alive until all camera callbacks have executed
    for camera_name in camera_names:
        topic = f"/{camera_namespace}/{camera_name}/color/camera_info"
        subscriptions.append(
            node.create_subscription(
                CameraInfo,
                topic,
                lambda _, name=camera_name: received.add(name),
                qos_profile_sensor_data,
            )
        )

    deadline = time.monotonic() + timeout_seconds
    try:
        while received != set(camera_names) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = sorted(set(camera_names) - received)
    if missing:
        raise RuntimeError(f"Missing CameraInfo topics: {missing}")


if __name__ == "__main__":
    main()
