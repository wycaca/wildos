from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from math import floor, hypot
from typing import Dict, Iterable, List, Optional, Set, Tuple

from grid_map_msgs.msg import GridMap
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray


GridIndex = Tuple[int, int]


@dataclass
class ClassifiedGrid:
    """Source-agnostic free, obstacle, unknown grid used by graph construction

    OccupancyGrid and GridMap inputs are both converted into this structure
    Elevation is optional and only used for visualization and 3D graph positions
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    free: np.ndarray
    obstacle: np.ndarray
    unknown: np.ndarray
    elevation: Optional[np.ndarray] = None
    z_offset: float = 0.0
    stats: Optional[Dict[str, int]] = None
    grid_map_center_x: Optional[float] = None
    grid_map_center_y: Optional[float] = None
    grid_map_length_x: Optional[float] = None
    grid_map_length_y: Optional[float] = None
    grid_map_convention: bool = False

    def in_bounds(self, ix: int, iy: int) -> bool:
        """判断栅格索引是否落在地图范围内"""
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        """将世界坐标转换为栅格索引, 超出地图时返回 None"""
        if self.grid_map_convention:
            iy = int(floor((self.origin_x - x) / self.resolution))
            ix = int(floor((self.origin_y - y) / self.resolution))
        else:
            ix = int(floor((x - self.origin_x) / self.resolution))
            iy = int(floor((y - self.origin_y) / self.resolution))
        if not self.in_bounds(ix, iy):
            return None
        return ix, iy

    def grid_to_world(self, ix: int, iy: int, z: Optional[float] = None) -> Tuple[float, float, float]:
        """将栅格索引转换为 cell 中心的世界坐标"""
        if self.grid_map_convention:
            x = self.origin_x - (iy + 0.5) * self.resolution
            y = self.origin_y - (ix + 0.5) * self.resolution
        else:
            x = self.origin_x + (ix + 0.5) * self.resolution
            y = self.origin_y + (iy + 0.5) * self.resolution
        if z is None:
            z = self.elevation_at_index(ix, iy)
        return x, y, z

    def elevation_at_index(self, ix: int, iy: int) -> float:
        """Return elevation for a cell when GridMap data is available"""
        if self.elevation is None or not self.in_bounds(ix, iy):
            return self.z_offset
        value = float(self.elevation[iy, ix])
        if not np.isfinite(value):
            return self.z_offset
        return value + self.z_offset

    def elevation_at_world(self, x: float, y: float) -> Optional[float]:
        """返回世界坐标处的 GridMap elevation, 无有效 cell 时返回 None"""
        if self.elevation is None:
            return None
        grid_index = self.world_to_grid(x, y)
        if grid_index is None:
            return None
        ix, iy = grid_index
        value = float(self.elevation[iy, ix])
        if not np.isfinite(value):
            value = self._nearest_finite_elevation(ix, iy)
        if value is None:
            return None
        return value + self.z_offset

    def _nearest_finite_elevation(self, ix: int, iy: int, radius_cells: int = 3) -> Optional[float]:
        """在局部邻域查找最近有效 elevation, 用于填补机器人脚下小 NaN 空洞"""
        if self.elevation is None:
            return None
        best_value = None
        best_distance = float("inf")
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = ix + dx
                ny = iy + dy
                if not self.in_bounds(nx, ny):
                    continue
                value = float(self.elevation[ny, nx])
                if not np.isfinite(value):
                    continue
                distance = hypot(float(dx), float(dy))
                if distance < best_distance:
                    best_distance = distance
                    best_value = value
        return best_value

    def project_to_elevation(self, position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """将 XY 位置投影到 GridMap elevation 表面, 无有效 elevation 时保留原 z"""
        elevation = self.elevation_at_world(position[0], position[1])
        if elevation is None:
            return position
        return position[0], position[1], elevation

    def is_free_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是已知可通行区域"""
        return self.in_bounds(ix, iy) and bool(self.free[iy, ix])

    def is_obstacle_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是障碍, 地图外默认按障碍处理"""
        return (not self.in_bounds(ix, iy)) or bool(self.obstacle[iy, ix])

    def is_unknown_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是未知区域"""
        return self.in_bounds(ix, iy) and bool(self.unknown[iy, ix])

    def is_world_collision_free(self, start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> bool:
        """检查世界坐标下两点之间的直线是否穿过 obstacle 或 unknown

        第一版将 unknown 也视为不可穿越, 这样生成的图会更保守
        后续如果需要更激进探索, 可以允许边接近 unknown, 但不能穿过 obstacle
        """
        start = self.world_to_grid(start_xy[0], start_xy[1])
        end = self.world_to_grid(end_xy[0], end_xy[1])
        if start is None or end is None:
            return False
        for ix, iy in bresenham_line(start[0], start[1], end[0], end[1]):
            if self.is_obstacle_index(ix, iy) or self.is_unknown_index(ix, iy):
                return False
        return True


def classify_occupancy_grid(
    msg: OccupancyGrid,
    free_threshold: int,
    obstacle_threshold: int,
) -> ClassifiedGrid:
    """将 OccupancyGrid 数值转换为 free, obstacle, unknown 掩码

    OccupancyGrid 中 -1 表示 unknown
    小于 free_threshold 的非负值视为 free
    大于 obstacle_threshold 的值视为 obstacle
    中间灰区暂时不作为可通行区域使用, 避免第一版生成过于激进的边
    """
    data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    unknown = data < 0
    free = np.logical_and(data >= 0, data <= free_threshold)
    obstacle = data >= obstacle_threshold
    return ClassifiedGrid(
        width=msg.info.width,
        height=msg.info.height,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
        frame_id=msg.header.frame_id,
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )


def classify_grid_map(
    msg: GridMap,
    traversability_layer: str,
    elevation_layer: str,
    free_threshold: float,
    obstacle_threshold: float,
    normalize_traversability: bool,
    normalize_low_quantile: float,
    normalize_high_quantile: float,
    z_offset: float,
    enable_postprocess: bool,
    min_free_component_cells: int,
    fill_hole_max_cells: int,
    fill_hole_min_free_neighbor_ratio: float,
    majority_fill_iterations: int,
    majority_fill_min_neighbors: int,
    transpose: bool,
    flip_x: bool,
    flip_y: bool,
) -> ClassifiedGrid:
    """Convert elevation GridMap layers into the graph classification format"""
    layers = {name: data for name, data in zip(msg.layers, msg.data)}
    if traversability_layer not in layers:
        raise ValueError(f"GridMap layer '{traversability_layer}' not found, available={list(msg.layers)}")

    trav = decode_multiarray(traversability_layer, layers[traversability_layer])
    if elevation_layer in layers:
        elevation = decode_multiarray(elevation_layer, layers[elevation_layer])
        valid = np.isfinite(trav) & np.isfinite(elevation)
    else:
        elevation = None
        valid = np.isfinite(trav)

    trav = normalize_grid_map_layer(
        trav,
        valid,
        enabled=normalize_traversability,
        low_quantile=normalize_low_quantile,
        high_quantile=normalize_high_quantile,
    )
    trav = orient_grid_map_array(trav, transpose=transpose, flip_x=flip_x, flip_y=flip_y)
    valid = orient_grid_map_array(valid, transpose=transpose, flip_x=flip_x, flip_y=flip_y)
    if elevation is not None:
        elevation = orient_grid_map_array(elevation, transpose=transpose, flip_x=flip_x, flip_y=flip_y)

    free = valid & (trav >= free_threshold)
    obstacle = valid & (trav <= obstacle_threshold)
    unknown = ~valid | (valid & ~(free | obstacle))
    raw_free_count = int(np.count_nonzero(free))
    raw_obstacle_count = int(np.count_nonzero(obstacle))
    raw_unknown_count = int(np.count_nonzero(unknown))
    free, obstacle, unknown = postprocess_classification(
        free,
        obstacle,
        unknown,
        enabled=enable_postprocess,
        min_free_component_cells=min_free_component_cells,
        fill_hole_max_cells=fill_hole_max_cells,
        fill_hole_min_free_neighbor_ratio=fill_hole_min_free_neighbor_ratio,
        majority_fill_iterations=majority_fill_iterations,
        majority_fill_min_neighbors=majority_fill_min_neighbors,
    )
    rows, cols = trav.shape

    return ClassifiedGrid(
        width=int(cols),
        height=int(rows),
        resolution=float(msg.info.resolution),
        origin_x=float(msg.info.pose.position.x + msg.info.length_x * 0.5),
        origin_y=float(msg.info.pose.position.y + msg.info.length_y * 0.5),
        frame_id=msg.header.frame_id,
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=elevation,
        z_offset=float(z_offset),
        stats={
            "valid": int(np.count_nonzero(valid)),
            "raw_free": raw_free_count,
            "raw_obstacle": raw_obstacle_count,
            "raw_unknown": raw_unknown_count,
            "free": int(np.count_nonzero(free)),
            "obstacle": int(np.count_nonzero(obstacle)),
            "unknown": int(np.count_nonzero(unknown)),
        },
        grid_map_center_x=float(msg.info.pose.position.x),
        grid_map_center_y=float(msg.info.pose.position.y),
        grid_map_length_x=float(msg.info.length_x),
        grid_map_length_y=float(msg.info.length_y),
        grid_map_convention=True,
    )


def postprocess_classification(
    free: np.ndarray,
    obstacle: np.ndarray,
    unknown: np.ndarray,
    enabled: bool,
    min_free_component_cells: int,
    fill_hole_max_cells: int,
    fill_hole_min_free_neighbor_ratio: float,
    majority_fill_iterations: int,
    majority_fill_min_neighbors: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Clean GridMap classification topology before graph sampling"""
    if not enabled:
        return free, obstacle, unknown

    free_value = 1
    obstacle_value = 2
    unknown_value = 0
    labels = np.full(free.shape, unknown_value, dtype=np.int8)
    labels[obstacle] = obstacle_value
    labels[free] = free_value

    labels = _majority_fill_free(
        labels,
        free_value=free_value,
        iterations=max(0, int(majority_fill_iterations)),
        min_neighbors=max(1, int(majority_fill_min_neighbors)),
    )
    labels = _fill_enclosed_regions(
        labels,
        target_values={unknown_value, obstacle_value},
        free_value=free_value,
        max_cells=max(0, int(fill_hole_max_cells)),
        min_free_neighbor_ratio=min(max(float(fill_hole_min_free_neighbor_ratio), 0.0), 1.0),
    )
    labels = _remove_small_free_components(
        labels,
        free_value=free_value,
        replacement_value=unknown_value,
        min_cells=max(1, int(min_free_component_cells)),
    )

    return labels == free_value, labels == obstacle_value, labels == unknown_value


def normalize_grid_map_layer(
    layer: np.ndarray,
    valid: np.ndarray,
    enabled: bool,
    low_quantile: float,
    high_quantile: float,
) -> np.ndarray:
    """Stretch GridMap scores when upstream traversability range is compressed"""
    if not enabled:
        return layer

    finite_values = layer[valid & np.isfinite(layer)]
    if finite_values.size < 2:
        return layer

    low_q = min(max(float(low_quantile), 0.0), 1.0)
    high_q = min(max(float(high_quantile), 0.0), 1.0)
    if high_q <= low_q:
        return layer

    low = float(np.quantile(finite_values, low_q))
    high = float(np.quantile(finite_values, high_q))
    if not np.isfinite(low) or not np.isfinite(high) or high <= low:
        return layer

    normalized = (layer - low) / (high - low)
    return np.clip(normalized, 0.0, 1.0).astype(np.float32)


def orient_grid_map_array(
    array: np.ndarray,
    transpose: bool,
    flip_x: bool,
    flip_y: bool,
) -> np.ndarray:
    """Apply the same orientation controls used by the debug projection adapter"""
    oriented = array
    if transpose:
        oriented = oriented.T
    if flip_x:
        oriented = np.flip(oriented, axis=1)
    if flip_y:
        oriented = np.flip(oriented, axis=0)
    return oriented


def _majority_fill_free(
    grid: np.ndarray,
    free_value: int,
    iterations: int,
    min_neighbors: int,
) -> np.ndarray:
    """Fill one-cell cracks when most neighbors are free"""
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
    """Fill small non-free pockets surrounded by free cells"""
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
    """Remove free islands too small to support graph nodes"""
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
    """Estimate whether a small non-free region is enclosed by free space"""
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


def decode_multiarray(name: str, array_msg: Float32MultiArray) -> np.ndarray:
    """Decode GridMap layer arrays while preserving row and column ordering"""
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

    side = int(np.ceil(np.sqrt(data.size))) if data.size else 0
    return _reshape_checked(name, data, side, side, "C")


def _reshape_checked(name: str, data: np.ndarray, rows: int, cols: int, order: str) -> np.ndarray:
    if rows * cols != data.size:
        raise ValueError(f"Layer '{name}' layout size does not match data length")
    return data.reshape((rows, cols), order=order)


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


def distance_to_mask(mask: np.ndarray, resolution: float) -> np.ndarray:
    """计算 8 邻接距离场, 不依赖 scipy

    输入 mask 表示目标 cell, 例如 obstacle 或 unknown
    输出中每个 cell 的值表示它到最近目标 cell 的近似欧氏距离
    这里使用 Dijkstra 风格的多源扩散, 便于在 ROS 环境缺少 scipy 时直接运行
    """
    height, width = mask.shape
    distances = np.full((height, width), np.inf, dtype=np.float32)
    queue: List[Tuple[float, int, int]] = []

    # 所有目标 cell 同时作为距离为 0 的源点入队, 相当于多源最短路
    source_ys, source_xs = np.where(mask)
    for iy, ix in zip(source_ys.tolist(), source_xs.tolist()):
        distances[iy, ix] = 0.0
        heappush(queue, (0.0, ix, iy))

    # 没有目标 cell 时给一个足够大的距离, 避免后续半径计算出现 inf
    if not queue:
        max_distance = hypot(width * resolution, height * resolution)
        distances.fill(max_distance)
        return distances

    # 8 邻接步长中, 对角线代价使用 sqrt(2), 直线代价使用 1
    neighbor_steps = (
        (-1, -1, 2**0.5),
        (0, -1, 1.0),
        (1, -1, 2**0.5),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, 2**0.5),
        (0, 1, 1.0),
        (1, 1, 2**0.5),
    )

    while queue:
        current_distance, ix, iy = heappop(queue)
        if current_distance > distances[iy, ix]:
            continue
        for dx, dy, step in neighbor_steps:
            nx = ix + dx
            ny = iy + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            next_distance = current_distance + step * resolution
            if next_distance < distances[ny, nx]:
                distances[ny, nx] = next_distance
                heappush(queue, (next_distance, nx, ny))

    return distances


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> Iterable[GridIndex]:
    """生成两个栅格索引之间离散直线经过的 cell

    建边和 frontier 分配都会用它做快速 collision check
    这只是第一版的几何近似, 后续可以替换为带 footprint inflation 的线段检查
    """
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x = x0
    y = y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        error2 = 2 * error
        if error2 >= dy:
            error += dy
            x += sx
        if error2 <= dx:
            error += dx
            y += sy
