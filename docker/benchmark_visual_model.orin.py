from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
import torch

from explorfm import ExploRFMInference


CAMERA_NAMES = ("front", "left", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark one real three-camera WildOS batch",
    )
    parser.add_argument("--camera-namespace", default="spot1/realsense")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--warmup-runs", type=int, default=2)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--object-query", default="blue bucket")
    return parser.parse_args()


def collect_images(
    camera_namespace: str,
    timeout_seconds: float,
) -> tuple[list[np.ndarray], dict[str, int]]:
    """Collect one current compressed RGB frame from every camera"""
    received: dict[str, CompressedImage] = {}
    namespace = camera_namespace.strip("/")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    rclpy.init()
    node = rclpy.create_node("wildos_visual_benchmark_input")
    subscriptions = []
    for camera_name in CAMERA_NAMES:
        topic = f"/{namespace}/{camera_name}/color/image_raw/compressed"
        subscriptions.append(
            node.create_subscription(
                CompressedImage,
                topic,
                lambda msg, name=camera_name: received.setdefault(name, msg),
                qos,
            )
        )

    deadline = time.monotonic() + timeout_seconds
    try:
        while len(received) != len(CAMERA_NAMES) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = [name for name in CAMERA_NAMES if name not in received]
    if missing:
        raise RuntimeError(f"Missing camera frames: {missing}")

    images = []
    compressed_sizes = {}
    for camera_name in CAMERA_NAMES:
        message = received[camera_name]
        compressed_sizes[camera_name] = len(message.data)
        bgr_image = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if bgr_image is None:
            raise RuntimeError(f"Failed to decode camera frame: {camera_name}")
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        images.append(np.rot90(rgb_image, k=2).copy())

    return images, compressed_sizes


def create_model(repo_root: Path) -> ExploRFMInference:
    """Create the same FP16 model used by the Orin WildOS deployment"""
    checkpoint_root = repo_root / "ckpts"
    return ExploRFMInference(
        frontier_ckpt=checkpoint_root / "frontier_ckpt_new.ckpt",
        traversability_ckpt=checkpoint_root / "traversability_ckpt.ckpt",
        model_version=checkpoint_root / "c-radio_v3-b_half.pth.tar",
        adaptor_version="siglip2",
        adaptor_ckpt_path=checkpoint_root / "siglip2",
        use_naclip=True,
        use_summary_for_spatial=True,
        radio_dim=768,
        static_scale_factor=0.75,
        model_precision="FP16",
    )


def main() -> None:
    args = parse_args()
    repo_root = Path(
        os.environ.get(
            "WILDOS_REPO_ROOT",
            "/opt/wildos_ws/src/nebula2-wildos",
        )
    )
    images, compressed_sizes = collect_images(
        args.camera_namespace,
        args.timeout,
    )
    batch = torch.stack(
        [
            torch.from_numpy(image).permute(2, 0, 1).float().div_(255.0)
            for image in images
        ]
    )

    model_load_started = time.perf_counter()
    model = create_model(repo_root)
    torch.cuda.synchronize()
    model_load_seconds = time.perf_counter() - model_load_started

    text_started = time.perf_counter()
    with torch.inference_mode():
        text_features = model.forward_on_text([args.object_query])
    torch.cuda.synchronize()
    text_seconds = time.perf_counter() - text_started

    # Warm up CUDA kernels before collecting stable batch latency
    with torch.inference_mode():
        for _ in range(args.warmup_runs):
            outputs = model.forward(batch)
            torch.cuda.synchronize()

        timings_ms = []
        torch.cuda.reset_peak_memory_stats()
        for _ in range(args.runs):
            started = time.perf_counter()
            outputs = model.forward(batch)
            torch.cuda.synchronize()
            timings_ms.append((time.perf_counter() - started) * 1000.0)

    result = {
        "camera_names": CAMERA_NAMES,
        "image_shape": list(images[0].shape),
        "compressed_bytes": compressed_sizes,
        "batch_shape": list(batch.shape),
        "output_shapes": [list(output.shape) for output in outputs],
        "text_feature_shape": list(text_features.shape),
        "model_load_seconds": round(model_load_seconds, 3),
        "text_query_ms": round(text_seconds * 1000.0, 3),
        "runs": args.runs,
        "batch_latency_average_ms": round(float(np.mean(timings_ms)), 3),
        "batch_latency_p50_ms": round(float(np.percentile(timings_ms, 50)), 3),
        "batch_latency_p95_ms": round(float(np.percentile(timings_ms, 95)), 3),
        "batch_latency_max_ms": round(float(np.max(timings_ms)), 3),
        "batch_throughput_fps": round(
            1000.0 * len(CAMERA_NAMES) / float(np.mean(timings_ms)),
            3,
        ),
        "cuda_peak_allocated_gib": round(
            torch.cuda.max_memory_allocated() / (1024**3),
            3,
        ),
        "cuda_reserved_gib": round(
            torch.cuda.memory_reserved() / (1024**3),
            3,
        ),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
