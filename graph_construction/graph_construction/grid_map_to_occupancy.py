from __future__ import annotations

import argparse
from dataclasses import dataclass, fields
from math import ceil, isfinite
from pathlib import Path
from collections import deque
from typing import Any, Dict, List, Optional, Set, Tuple

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


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
        """Convert each GridMap update immediately unless timer mode is enabled"""
        self.latest_grid_map = msg
        if not self._logged_first_input:
            self.get_logger().info(
                f"Received first GridMap, frame={msg.header.frame_id}, layers={list(msg.layers)}"
            )
            self._logged_first_input = True
        if not self.config.use_timer:
            self._publish_occupancy(msg)

    def _on_timer(self) -> None:
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
        layers = {name: data for name, data in zip(msg.layers, msg.data)}
        if self.config.traversability_layer not in layers:
            raise ValueError(
                f"GridMap layer '{self.config.traversability_layer}' not found, available={list(msg.layers)}"
            )

        trav = _decode_multiarray(self.config.traversability_layer, layers[self.config.traversability_layer])
        if self.config.elevation_layer in layers:
            elevation = _decode_multiarray(self.config.elevation_layer, layers[self.config.elevation_layer])
            valid = np.isfinite(trav) & np.isfinite(elevation)
        else:
            valid = np.isfinite(trav)

        trav = self._normalize_traversability(trav, valid)
        trav = self._orient_array(trav)
        valid = self._orient_array(valid)
        rows, cols = trav.shape

        grid = np.full((rows, cols), self.config.unknown_value, dtype=np.int16)
        grid[valid & (trav >= self.config.free_threshold)] = self.config.free_value
        grid[valid & (trav <= self.config.occupied_threshold)] = self.config.occupied_value
        raw_grid = grid.copy()
        grid = self._postprocess_grid(grid)

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

    def _normalize_traversability(self, trav: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """Stretch compressed traversability scores while preserving invalid cells"""
        if not self.config.normalize_traversability:
            return trav

        finite_values = trav[valid & np.isfinite(trav)]
        if finite_values.size < 2:
            return trav

        low_q = min(max(self.config.normalize_low_quantile, 0.0), 1.0)
        high_q = min(max(self.config.normalize_high_quantile, 0.0), 1.0)
        if high_q <= low_q:
            self._last_normalization_stats = None
            return trav

        low = float(np.quantile(finite_values, low_q))
        high = float(np.quantile(finite_values, high_q))
        if not isfinite(low) or not isfinite(high) or high <= low:
            self._last_normalization_stats = None
            return trav

        normalized = (trav - low) / (high - low)
        self._last_normalization_stats = {
            "valid_count": float(finite_values.size),
            "raw_min": float(np.min(finite_values)),
            "raw_median": float(np.median(finite_values)),
            "raw_max": float(np.max(finite_values)),
            "low": low,
            "high": high,
        }
        return np.clip(normalized, 0.0, 1.0).astype(np.float32)

    def _orient_array(self, array: np.ndarray) -> np.ndarray:
        oriented = array
        if self.config.transpose:
            oriented = oriented.T
        if self.config.flip_x:
            oriented = np.flip(oriented, axis=1)
        if self.config.flip_y:
            oriented = np.flip(oriented, axis=0)
        return oriented

    def _postprocess_grid(self, grid: np.ndarray) -> np.ndarray:
        """Apply conservative topology cleanup for graph sampling"""
        if not self.config.enable_postprocess:
            return grid

        processed = grid.copy()
        processed = _majority_fill_free(
            processed,
            free_value=self.config.free_value,
            iterations=max(0, int(self.config.majority_fill_iterations)),
            min_neighbors=max(1, int(self.config.majority_fill_min_neighbors)),
        )
        processed = _fill_enclosed_regions(
            processed,
            target_values={self.config.unknown_value, self.config.occupied_value},
            free_value=self.config.free_value,
            max_cells=max(0, int(self.config.fill_hole_max_cells)),
            min_free_neighbor_ratio=min(max(self.config.fill_hole_min_free_neighbor_ratio, 0.0), 1.0),
        )
        processed = _remove_small_free_components(
            processed,
            free_value=self.config.free_value,
            replacement_value=self.config.unknown_value,
            min_cells=max(1, int(self.config.min_free_component_cells)),
        )
        return processed

    def _log_first_grid_stats(self, raw_grid: np.ndarray, grid: np.ndarray, valid: np.ndarray) -> None:
        """Report first conversion statistics to catch threshold and publisher mixups"""
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


GridIndex = Tuple[int, int]


def _majority_fill_free(
    grid: np.ndarray,
    free_value: int,
    iterations: int,
    min_neighbors: int,
) -> np.ndarray:
    """Fill one-cell cracks when most neighbors are already free"""
    processed = grid.copy()
    for _ in range(iterations):
        free = processed == free_value
        neighbor_count = np.zeros(processed.shape, dtype=np.uint8)
        for dx, dy in _NEIGHBOR_OFFSETS_8:
            shifted = np.zeros(processed.shape, dtype=bool)
            src_y, dst_y = _shift_slices(free.shape[0], dy)
            src_x, dst_x = _shift_slices(free.shape[1], dx)
            shifted[dst_y, dst_x] = free[src_y, src_x]
            neighbor_count += shifted.astype(np.uint8)
        promote = (processed != free_value) & (neighbor_count >= min_neighbors)
        if not np.any(promote):
            break
        processed[promote] = free_value
    return processed


def _fill_enclosed_regions(
    grid: np.ndarray,
    target_values: Set[int],
    free_value: int,
    max_cells: int,
    min_free_neighbor_ratio: float,
) -> np.ndarray:
    """Fill small non-free components surrounded by free cells"""
    if max_cells <= 0:
        return grid

    processed = grid.copy()
    target = np.isin(processed, list(target_values))
    visited = np.zeros(processed.shape, dtype=bool)
    height, width = processed.shape

    for iy in range(height):
        for ix in range(width):
            if visited[iy, ix] or not target[iy, ix]:
                continue

            component, touches_border = _collect_component(target, visited, ix, iy)
            if touches_border or len(component) > max_cells:
                continue
            if _free_neighbor_ratio(processed, component, free_value) < min_free_neighbor_ratio:
                continue
            for cx, cy in component:
                processed[cy, cx] = free_value

    return processed


def _remove_small_free_components(
    grid: np.ndarray,
    free_value: int,
    replacement_value: int,
    min_cells: int,
) -> np.ndarray:
    """Remove isolated free islands that cannot support graph nodes"""
    processed = grid.copy()
    free = processed == free_value
    visited = np.zeros(processed.shape, dtype=bool)
    height, width = processed.shape

    for iy in range(height):
        for ix in range(width):
            if visited[iy, ix] or not free[iy, ix]:
                continue
            component, _ = _collect_component(free, visited, ix, iy)
            if len(component) >= min_cells:
                continue
            for cx, cy in component:
                processed[cy, cx] = replacement_value

    return processed


def _collect_component(
    mask: np.ndarray,
    visited: np.ndarray,
    start_x: int,
    start_y: int,
) -> Tuple[List[GridIndex], bool]:
    """Collect one 8-connected component from a boolean mask"""
    height, width = mask.shape
    queue: deque[GridIndex] = deque([(start_x, start_y)])
    visited[start_y, start_x] = True
    component: List[GridIndex] = []
    touches_border = False

    while queue:
        ix, iy = queue.popleft()
        component.append((ix, iy))
        if ix == 0 or iy == 0 or ix == width - 1 or iy == height - 1:
            touches_border = True

        for dx, dy in _NEIGHBOR_OFFSETS_8:
            nx = ix + dx
            ny = iy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if visited[ny, nx] or not mask[ny, nx]:
                continue
            visited[ny, nx] = True
            queue.append((nx, ny))

    return component, touches_border


def _free_neighbor_ratio(grid: np.ndarray, component: List[GridIndex], free_value: int) -> float:
    free_neighbors = 0
    non_component_neighbors = 0
    component_set = set(component)
    height, width = grid.shape

    for ix, iy in component:
        for dx, dy in _NEIGHBOR_OFFSETS_8:
            nx = ix + dx
            ny = iy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height or (nx, ny) in component_set:
                continue
            non_component_neighbors += 1
            if grid[ny, nx] == free_value:
                free_neighbors += 1

    if non_component_neighbors == 0:
        return 0.0
    return float(free_neighbors) / float(non_component_neighbors)


def _shift_slices(size: int, offset: int) -> Tuple[slice, slice]:
    if offset < 0:
        return slice(-offset, size), slice(0, size + offset)
    if offset > 0:
        return slice(0, size - offset), slice(offset, size)
    return slice(0, size), slice(0, size)


_NEIGHBOR_OFFSETS_8: Tuple[GridIndex, ...] = (
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
)


def _decode_multiarray(name: str, array_msg: Float32MultiArray) -> np.ndarray:
    data = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0 = dims[0].label
        label1 = dims[1].label
        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size
            cols = dims[1].size
            return _reshape_checked(name, data, rows, cols, "C")
        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size
            rows = dims[1].size
            return _reshape_checked(name, data, rows, cols, "F")

    if len(dims) >= 2:
        rows = dims[1].size
        cols = dims[0].size
        return _reshape_checked(name, data, rows, cols, "C")

    side = int(ceil(np.sqrt(data.size))) if data.size else 0
    return _reshape_checked(name, data, side, side, "C")


def _reshape_checked(name: str, data: np.ndarray, rows: int, cols: int, order: str) -> np.ndarray:
    if rows * cols != data.size:
        raise ValueError(f"Layer '{name}' layout size does not match data length")
    return data.reshape((rows, cols), order=order)


def _adapter_config(config: Dict[str, Any]) -> GridMapToOccupancyConfig:
    allowed = {field.name for field in fields(GridMapToOccupancyConfig)}
    values = {key: value for key, value in config.items() if key in allowed}
    return GridMapToOccupancyConfig(**values)


def _load_config(config_name: str) -> Dict[str, Any]:
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
