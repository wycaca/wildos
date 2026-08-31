#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import statistics
import time

import yaml


DEFAULT_CONFIG = Path(__file__).with_name("performance_baseline.yaml")


def _percentile(samples: list[float], fraction: float) -> float:
    ordered = sorted(samples)
    index = min(math.ceil(len(ordered) * fraction) - 1, len(ordered) - 1)
    return ordered[index]


def summarize_samples(
    samples: list[float],
    budget_ms: float,
) -> dict[str, object]:
    """汇总固定样本并执行周期预算门槛"""
    if not samples:
        raise ValueError("performance samples must not be empty")
    p95_ms = _percentile(samples, 0.95)
    return {
        "sample_count": len(samples),
        "median_ms": statistics.median(samples),
        "p95_ms": p95_ms,
        "maximum_ms": max(samples),
        "budget_ms": budget_ms,
        "budget_met": p95_ms < budget_ms,
        "budget_utilization": p95_ms / budget_ms,
    }


def migration_candidates(
    scenarios: dict[str, dict[str, object]],
    hotspot_share_threshold: float = 0.30,
) -> list[str]:
    """仅选择超预算且存在主要热点的迁移候选"""
    candidates = []
    for name, result in scenarios.items():
        if result["budget_met"]:
            continue
        stages = result.get("stages", {})
        detailed_p95 = [
            stage["p95_ms"]
            for stage_name, stage in stages.items()
            if stage_name not in {"total", "scan_total", "control_total"}
        ]
        unattributed_p95 = max(0.0, result["p95_ms"] - sum(detailed_p95))
        largest_stage_p95 = max([unattributed_p95, *detailed_p95])
        if largest_stage_p95 / result["p95_ms"] >= hotspot_share_threshold:
            candidates.append(name)
    return candidates


def _measure(callback, warmup: int, repeats: int):
    for _ in range(warmup):
        callback()
    samples = []
    stage_samples = {}
    last_output = None
    for _ in range(repeats):
        started = time.perf_counter()
        last_output = callback()
        samples.append((time.perf_counter() - started) * 1000.0)
        for name, elapsed_ms in last_output.pop("_stages_ms", {}).items():
            stage_samples.setdefault(name, []).append(elapsed_ms)
    return samples, last_output, stage_samples


def _summarize_stages(
    stage_samples: dict[str, list[float]],
) -> dict[str, dict]:
    return {
        name: {
            "sample_count": len(samples),
            "median_ms": statistics.median(samples),
            "p95_ms": _percentile(samples, 0.95),
            "maximum_ms": max(samples),
        }
        for name, samples in stage_samples.items()
    }


def _quick_config(config: dict) -> dict:
    updated = json.loads(json.dumps(config))
    for scenario in updated["scenarios"].values():
        scenario["warmup"] = min(int(scenario["warmup"]), 1)
        scenario["repeats"] = min(int(scenario["repeats"]), 2)
        if "points" in scenario:
            scenario["points"] = min(int(scenario["points"]), 2_000)
        if "width" in scenario:
            scenario["width"] = min(int(scenario["width"]), 40)
            scenario["height"] = min(int(scenario["height"]), 40)
    return updated


class _Publisher:
    def __init__(self) -> None:
        self.last_message = None

    def publish(self, message) -> None:
        self.last_message = message


def _navigation_scenarios(config: dict, rng):
    import numpy as np
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    from wildos_navigation.controller_node import (
        AdvancedGeometricFollower,
        SimpleKalmanFilter,
    )
    from wildos_navigation.map_pub import LocalObstacleGridNode
    from wildos_navigation.performance_stats import EventRate, TimingWindow

    map_config = config["scenarios"]["map_pub"]
    map_xyz = np.empty((int(map_config["points"]), 3), dtype=np.float32)
    map_xyz[:, 0] = rng.uniform(0.0, 10.1, len(map_xyz))
    map_xyz[:, 1] = rng.uniform(0.0, 10.1, len(map_xyz))
    map_xyz[:, 2] = rng.uniform(0.0, 1.4, len(map_xyz))
    map_cloud = point_cloud2.create_cloud_xyz32(
        header=Header(),
        points=map_xyz,
    )
    map_node = LocalObstacleGridNode.__new__(LocalObstacleGridNode)
    map_node.width_cells = map_node.height_cells = 101
    map_node.center_u = map_node.center_v = 50
    map_node.resolution = 0.1
    map_node.min_height = 0.1
    map_node.max_height = 1.0
    map_node.obstacle_radius = 0.2
    map_node.obstacle_height_threshold = 0.25
    map_node.odom_frame_id = "odom"
    map_node.odom_data = Odometry()
    map_node.odom_data.pose.pose.position.x = 5.05
    map_node.odom_data.pose.pose.position.y = 5.05
    map_node.grid_combined = OccupancyGrid()
    map_node.grid_combined.info.width = 101
    map_node.grid_combined.info.height = 101
    map_node.grid_combined.info.resolution = 0.1
    map_node.grid_combined_pub = _Publisher()
    map_node._timings = {
        stage: TimingWindow(512)
        for stage in (
            "total",
            "decode",
            "filter",
            "project",
            "grid",
            "publish",
        )
    }
    map_node._cloud_rate = EventRate()
    map_node._diagnostic_counters = Counter()
    map_node._last_cloud_stamp_ns = None

    scan_config = config["scenarios"]["controller_scan"]
    scan_xyz = np.empty((int(scan_config["points"]), 3), dtype=np.float32)
    scan_xyz[:, 0] = rng.uniform(-4.5, 4.5, len(scan_xyz))
    scan_xyz[:, 1] = rng.uniform(-4.5, 4.5, len(scan_xyz))
    scan_xyz[:, 2] = rng.uniform(0.0, 1.0, len(scan_xyz))
    scan_cloud = point_cloud2.create_cloud_xyz32(map_cloud.header, scan_xyz)
    controller = AdvancedGeometricFollower.__new__(AdvancedGeometricFollower)
    controller.latest_pose = [0.0, 0.0, 0.0]
    controller.real_obstacles = []
    controller.predicted_dynamic_obs = []
    controller.kf_tracker = SimpleKalmanFilter(0.1)
    controller._timings = {
        stage: TimingWindow(512)
        for stage in (
            "scan_total",
            "scan_decode",
            "scan_filter",
            "scan_limit",
            "scan_cluster",
            "control_total",
            "control_publish",
        )
    }
    controller._cloud_rate = EventRate()
    controller._diagnostic_counters = Counter()
    controller._last_cloud_stamp_ns = None

    follower = AdvancedGeometricFollower.__new__(AdvancedGeometricFollower)
    follower.path_received = True
    follower.path_points = np.array([[0.0, 0.0], [1.0, 0.0]])
    follower.latest_pose = [0.0, 0.0, 0.0]
    follower.real_obstacles = []
    follower.predicted_dynamic_obs = []
    follower.v_target = 0.6
    follower.w_max = 1.0
    follower.lookahead_dist = 0.55
    follower.safe_radius = 0.22
    follower.dynamic_safe_zone = 0.45
    follower.cmd_vel_publisher = _Publisher()
    follower._timings = {
        stage: TimingWindow(512)
        for stage in (
            "scan_total",
            "scan_decode",
            "scan_filter",
            "scan_limit",
            "scan_cluster",
            "control_total",
            "control_publish",
        )
    }
    follower._diagnostic_counters = Counter()

    return {
        "map_pub": (
            lambda: (
                map_node.pointcloud_callback(map_cloud),
                {
                    "grid_cells": len(map_node.grid_combined.data),
                    "published": (
                        map_node.grid_combined_pub.last_message is not None
                    ),
                    "_stages_ms": {
                        stage: timing.latest_ms()
                        for stage, timing in map_node._timings.items()
                        if timing.latest_ms() is not None
                    },
                },
            )[1]
        ),
        "controller_scan": (
            lambda: (
                controller.scan_callback(scan_cloud),
                {
                    "obstacles": len(controller.real_obstacles),
                    "predicted": len(controller.predicted_dynamic_obs),
                    "_stages_ms": {
                        stage: timing.latest_ms()
                        for stage, timing in controller._timings.items()
                        if timing.latest_ms() is not None
                    },
                },
            )[1]
        ),
        "controller_control": (
            lambda: (
                follower.control_loop(),
                {
                    "linear_x": (
                        follower.cmd_vel_publisher.last_message.linear.x
                    ),
                    "angular_z": (
                        follower.cmd_vel_publisher.last_message.angular.z
                    ),
                    "_stages_ms": {
                        stage: timing.latest_ms()
                        for stage, timing in follower._timings.items()
                        if timing.latest_ms() is not None
                    },
                },
            )[1]
        ),
    }


def _graph_grid(width: int, height: int, resolution: float):
    import numpy as np
    from graph_construction.grid_types import ClassifiedGrid

    free = np.ones((height, width), dtype=bool)
    obstacle = np.zeros_like(free)
    unknown = np.zeros_like(free)
    unknown[[0, -1], :] = True
    unknown[:, [0, -1]] = True
    unknown[1:-1, 3 * width // 4:] = True
    free[unknown] = False
    obstacle[height // 4:3 * height // 4, width // 2] = True
    obstacle[height // 2, width // 2] = False
    free[obstacle] = False
    return ClassifiedGrid(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=np.zeros((height, width), dtype=np.float32),
    )


def _graph_scenarios(config: dict):
    from graph_construction.graph_builder import (
        GraphBuilderConfig,
        SparseGraphBuilder,
    )

    def full_update(settings):
        grid = _graph_grid(
            int(settings["width"]),
            int(settings["height"]),
            float(settings["resolution"]),
        )
        builder = SparseGraphBuilder(GraphBuilderConfig())
        result = builder.update(
            grid,
            robot_position=(
                settings["width"] * settings["resolution"] * 0.25,
                settings["height"] * settings["resolution"] * 0.5,
                0.0,
            ),
            stamp_seconds=1.0,
        )
        return {
            "nodes": len(result.graph.nodes),
            "edges": len(result.graph.edges),
            "frontier_candidates": result.stats.frontier_candidate_count,
            "_stages_ms": {
                name: elapsed * 1000.0
                for name, elapsed in result.stats.stage_seconds.items()
            },
        }

    incremental_settings = config["scenarios"]["graph_incremental"]
    incremental_grid = _graph_grid(
        int(incremental_settings["width"]),
        int(incremental_settings["height"]),
        float(incremental_settings["resolution"]),
    )
    incremental_position = (
        incremental_settings["width"]
        * incremental_settings["resolution"]
        * 0.25,
        incremental_settings["height"]
        * incremental_settings["resolution"]
        * 0.5,
        0.0,
    )
    incremental_builder = SparseGraphBuilder(GraphBuilderConfig())
    incremental_builder.update(incremental_grid, incremental_position, 1.0)
    incremental_stamp = 1.0

    def incremental_update():
        nonlocal incremental_stamp
        incremental_stamp += 1.0
        ix = int(incremental_settings["width"]) // 3
        iy = int(incremental_settings["height"]) // 3
        blocked = int(incremental_stamp) % 2 == 0
        incremental_grid.obstacle[iy, ix] = blocked
        incremental_grid.free[iy, ix] = not blocked
        incremental_grid.origin_x += float(incremental_settings["resolution"])
        result = incremental_builder.update(
            incremental_grid,
            incremental_position,
            incremental_stamp,
        )
        return {
            "nodes": len(result.graph.nodes),
            "edges": len(result.graph.edges),
            "dirty_cells": result.stats.dirty_cell_count,
            "_stages_ms": {
                name: elapsed * 1000.0
                for name, elapsed in result.stats.stage_seconds.items()
            },
        }

    return {
        "graph_full": lambda: full_update(config["scenarios"]["graph_full"]),
        "graph_incremental": incremental_update,
    }


def run_benchmarks(config: dict) -> dict[str, dict[str, object]]:
    """运行固定合成输入并返回统一场景结果"""
    import numpy as np

    random.seed(int(config["seed"]))
    rng = np.random.default_rng(int(config["seed"]))
    callbacks = _navigation_scenarios(config, rng)
    callbacks.update(_graph_scenarios(config))
    results = {}
    for name, callback in callbacks.items():
        settings = config["scenarios"][name]
        samples, output, stage_samples = _measure(
            callback,
            int(settings["warmup"]),
            int(settings["repeats"]),
        )
        results[name] = {
            **summarize_samples(samples, float(settings["budget_ms"])),
            "stages": _summarize_stages(stage_samples),
            "output": output,
        }
    return results


def _load_config(path: Path) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported performance config schema")
    return config


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run reproducible WildOS performance gates"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--fail-on-budget", action="store_true")
    args = parser.parse_args()

    config = _load_config(args.config)
    if args.quick:
        config = _quick_config(config)
    output_dir = args.output_dir or Path(config["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    scenarios = run_benchmarks(config)
    candidates = migration_candidates(
        scenarios,
        float(config["migration_hotspot_share"]),
    )
    report = {
        "schema_version": 1,
        "generated_at_unix": started,
        "duration_sec": time.time() - started,
        "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
        "environment": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "cpu_count": os.cpu_count(),
        },
        "scenarios": scenarios,
        "migration_gate": {
            "hotspot_share_threshold": config["migration_hotspot_share"],
            "budget_candidates": candidates,
            "cpp_migration_required": bool(candidates),
            "reason": (
                "review stage hotspot share before migration"
                if candidates
                else "all measured Python cores meet their cycle budgets"
            ),
        },
    }
    output_path = output_dir / "report.json"
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(output_path)
    for name, result in scenarios.items():
        print(
            f"{name}: median={result['median_ms']:.3f}ms, "
            f"p95={result['p95_ms']:.3f}ms, "
            f"budget={result['budget_ms']:.1f}ms, "
            f"met={result['budget_met']}"
        )
    return 2 if args.fail_on_budget and candidates else 0


if __name__ == "__main__":
    raise SystemExit(main())
