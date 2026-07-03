from __future__ import annotations

import argparse
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from graph_construction.grid_adapter import (
    decode_grid_map_layer,
    normalize_grid_map_layer,
    orient_grid_map_array,
    postprocess_classification,
)


@dataclass
class GridMapToOccupancyConfig:
    input_topic: str = "/elevation_mapping_node/elevation_map_raw"
    output_topic: str = "/spot1/elevation_traversability_grid"
    traversability_layer: str = "traversability"
    elevation_layer: str = "elevation"
    free_threshold: float = 0.7
    occupied_threshold: float = 0.4
    unknown_value: int = -1
    free_value: int = 0
    occupied_value: int = 100
    publish_rate_hz: float = 5.0
    use_timer: bool = False
    normalize_traversability: bool = True
    normalize_low_quantile: float = 0.05
    normalize_high_quantile: float = 0.95
    enable_postprocess: bool = True
    min_free_component_cells: int = 25
    fill_hole_max_cells: int = 90
    fill_hole_min_free_neighbor_ratio: float = 0.65
    majority_fill_iterations: int = 1
    majority_fill_min_neighbors: int = 6
    transpose: bool = False
    flip_x: bool = False
    flip_y: bool = False


class GridMapToOccupancyNode(Node):
    """将 GridMap 转成 OccupancyGrid, 仅用于 debug 和兼容旧工具"""

    def __init__(self, config: Dict[str, Any]):
        super().__init__("grid_map_to_occupancy")
        self.config = _adapter_config(config)
        self.latest_grid_map: Optional[GridMap] = None
        self._logged_first_input = False
        self._logged_first_publish = False
        self._last_normalization_stats: Optional[Dict[str, float]] = None

        self.grid_sub = self.create_subscription(
            GridMap,
            self.config.input_topic,
            self._on_grid_map,
            10,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, self.config.output_topic, 10)

        if self.config.use_timer:
            period = 1.0 / max(self.config.publish_rate_hz, 0.1)
            self.timer = self.create_timer(period, self._on_timer)
        else:
            self.timer = None

        self.get_logger().info(
            f"GridMap adapter started, input={self.config.input_topic}, output={self.config.output_topic}"
        )

    def _on_grid_map(self, msg: GridMap) -> None:
        """除非启用 timer 模式, 否则每次 GridMap 更新后立即转换"""
        self.latest_grid_map = msg
        if not self._logged_first_input:
            self.get_logger().info(
                f"Received first GridMap, frame={msg.header.frame_id}, layers={list(msg.layers)}"
            )
            self._logged_first_input = True
        if not self.config.use_timer:
            self._publish_occupancy(msg)

    def _on_timer(self) -> None:
        """timer 模式下按固定频率发布最近一帧 GridMap 投影"""
        if self.latest_grid_map is None:
            return
        self._publish_occupancy(self.latest_grid_map)

    def _publish_occupancy(self, msg: GridMap) -> None:
        try:
            occupancy = self._convert(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        self.grid_pub.publish(occupancy)
        if not self._logged_first_publish:
            self.get_logger().info(
                f"Published first OccupancyGrid, frame={occupancy.header.frame_id}, "
                f"size={occupancy.info.width}x{occupancy.info.height}"
            )
            self._logged_first_publish = True

    def _convert(self, msg: GridMap) -> OccupancyGrid:
        """复用主 GridMap 解码链路, 只在最后转换为 OccupancyGrid"""
        layers = {name: data for name, data in zip(msg.layers, msg.data)}
        if self.config.traversability_layer not in layers:
            raise ValueError(
                f"GridMap layer '{self.config.traversability_layer}' not found, available={list(msg.layers)}"
            )

        trav = decode_grid_map_layer(self.config.traversability_layer, layers[self.config.traversability_layer], msg)
        if self.config.elevation_layer in layers:
            elevation = decode_grid_map_layer(self.config.elevation_layer, layers[self.config.elevation_layer], msg)
            valid = np.isfinite(trav) & np.isfinite(elevation)
        else:
            valid = np.isfinite(trav)

        self._last_normalization_stats = _normalization_stats(trav, valid, self.config)
        trav = normalize_grid_map_layer(
            trav,
            valid,
            enabled=self.config.normalize_traversability,
            low_quantile=self.config.normalize_low_quantile,
            high_quantile=self.config.normalize_high_quantile,
        )
        trav = orient_grid_map_array(
            trav,
            transpose=self.config.transpose,
            flip_x=self.config.flip_x,
            flip_y=self.config.flip_y,
        )
        valid = orient_grid_map_array(
            valid,
            transpose=self.config.transpose,
            flip_x=self.config.flip_x,
            flip_y=self.config.flip_y,
        )
        rows, cols = trav.shape

        free = valid & (trav >= self.config.free_threshold)
        occupied = valid & (trav <= self.config.occupied_threshold)
        unknown = ~valid | (valid & ~(free | occupied))
        raw_grid = _masks_to_occupancy_values(free, occupied, unknown, self.config)
        free, occupied, unknown = postprocess_classification(
            free,
            occupied,
            unknown,
            enabled=self.config.enable_postprocess,
            min_free_component_cells=self.config.min_free_component_cells,
            fill_hole_max_cells=self.config.fill_hole_max_cells,
            fill_hole_min_free_neighbor_ratio=self.config.fill_hole_min_free_neighbor_ratio,
            majority_fill_iterations=self.config.majority_fill_iterations,
            majority_fill_min_neighbors=self.config.majority_fill_min_neighbors,
        )
        grid = _masks_to_occupancy_values(free, occupied, unknown, self.config)

        if not self._logged_first_publish:
            self._log_first_grid_stats(raw_grid, grid, valid)

        out = OccupancyGrid()
        out.header = msg.header
        out.info.resolution = float(msg.info.resolution)
        out.info.width = int(cols)
        out.info.height = int(rows)
        out.info.origin.position.x = float(msg.info.pose.position.x - msg.info.length_x * 0.5)
        out.info.origin.position.y = float(msg.info.pose.position.y - msg.info.length_y * 0.5)
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = [int(v) for v in grid.reshape(-1)]
        return out

    def _log_first_grid_stats(self, raw_grid: np.ndarray, grid: np.ndarray, valid: np.ndarray) -> None:
        """输出首帧转换统计, 用于发现阈值或 publisher 混用问题"""
        unknown = int(np.count_nonzero(grid == self.config.unknown_value))
        free = int(np.count_nonzero(grid == self.config.free_value))
        occupied = int(np.count_nonzero(grid == self.config.occupied_value))
        raw_unknown = int(np.count_nonzero(raw_grid == self.config.unknown_value))
        raw_free = int(np.count_nonzero(raw_grid == self.config.free_value))
        raw_occupied = int(np.count_nonzero(raw_grid == self.config.occupied_value))
        valid_count = int(np.count_nonzero(valid))

        stats = self._last_normalization_stats
        if stats:
            self.get_logger().info(
                "First occupancy stats, "
                f"valid={valid_count}, raw_free={raw_free}, raw_occupied={raw_occupied}, "
                f"raw_unknown={raw_unknown}, free={free}, occupied={occupied}, unknown={unknown}, "
                f"raw_min={stats['raw_min']:.3f}, raw_median={stats['raw_median']:.3f}, "
                f"raw_max={stats['raw_max']:.3f}, norm_low={stats['low']:.3f}, norm_high={stats['high']:.3f}"
            )
        else:
            self.get_logger().info(
                f"First occupancy stats, valid={valid_count}, raw_free={raw_free}, "
                f"raw_occupied={raw_occupied}, raw_unknown={raw_unknown}, free={free}, "
                f"occupied={occupied}, unknown={unknown}, normalization=off"
            )


def _masks_to_occupancy_values(
    free: np.ndarray,
    occupied: np.ndarray,
    unknown: np.ndarray,
    config: GridMapToOccupancyConfig,
) -> np.ndarray:
    """把分类 mask 转成 OccupancyGrid 数值"""
    grid = np.full(free.shape, config.unknown_value, dtype=np.int16)
    grid[unknown] = config.unknown_value
    grid[free] = config.free_value
    grid[occupied] = config.occupied_value
    return grid


def _normalization_stats(
    trav: np.ndarray,
    valid: np.ndarray,
    config: GridMapToOccupancyConfig,
) -> Optional[Dict[str, float]]:
    """记录归一化前的分布, 只用于首帧诊断日志"""
    if not config.normalize_traversability:
        return None

    finite_values = trav[valid & np.isfinite(trav)]
    if finite_values.size < 2:
        return None

    low_q = min(max(float(config.normalize_low_quantile), 0.0), 1.0)
    high_q = min(max(float(config.normalize_high_quantile), 0.0), 1.0)
    if high_q <= low_q:
        return None

    low = float(np.quantile(finite_values, low_q))
    high = float(np.quantile(finite_values, high_q))
    if not np.isfinite(low) or not np.isfinite(high) or high <= low:
        return None

    return {
        "valid_count": float(finite_values.size),
        "raw_min": float(np.min(finite_values)),
        "raw_median": float(np.median(finite_values)),
        "raw_max": float(np.max(finite_values)),
        "low": low,
        "high": high,
    }


def _adapter_config(config: Dict[str, Any]) -> GridMapToOccupancyConfig:
    """过滤 YAML 中属于 debug adapter 的配置项"""
    allowed = {field.name for field in fields(GridMapToOccupancyConfig)}
    values = {key: value for key, value in config.items() if key in allowed}
    return GridMapToOccupancyConfig(**values)


def _load_config(config_name: str) -> Dict[str, Any]:
    """加载安装目录或绝对路径中的 debug adapter 配置"""
    config_path = Path(config_name)
    if not config_path.is_absolute():
        config_path = Path(get_package_share_directory("graph_construction")) / "configs" / config_name
    if not config_path.exists():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    return loaded


def main(args=None) -> None:
    """ROS2 控制台脚本入口"""
    rclpy.init(args=args)
    custom_args = rclpy.utilities.remove_ros_args(args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="grid_map_to_occupancy.yaml")
    parsed = parser.parse_args(custom_args[1:] if custom_args else [])

    node = GridMapToOccupancyNode(_load_config(parsed.config))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
