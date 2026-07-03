from __future__ import annotations

from collections import deque
from math import atan2
from typing import List, Set, Tuple

from grid_map_msgs.msg import GridMap
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

from graph_construction.grid_types import ClassifiedGrid, GridIndex


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
    """把 elevation GridMap layer 转成 graph 分类格式"""
    layers = {name: data for name, data in zip(msg.layers, msg.data)}
    if traversability_layer not in layers:
        raise ValueError(f"GridMap layer '{traversability_layer}' not found, available={list(msg.layers)}")

    trav = decode_grid_map_layer(traversability_layer, layers[traversability_layer], msg)
    if elevation_layer in layers:
        elevation = decode_grid_map_layer(elevation_layer, layers[elevation_layer], msg)
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
    grid_map_yaw = yaw_from_quaternion(msg.info.pose.orientation)
    length_x = float(rows) * float(msg.info.resolution)
    length_y = float(cols) * float(msg.info.resolution)

    return ClassifiedGrid(
        width=int(cols),
        height=int(rows),
        resolution=float(msg.info.resolution),
        origin_x=float(msg.info.pose.position.x),
        origin_y=float(msg.info.pose.position.y),
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
        grid_map_length_x=length_x,
        grid_map_length_y=length_y,
        grid_map_yaw=grid_map_yaw,
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
    """在 graph 采样前清理 GridMap 分类拓扑"""
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
    """上游 traversability 数值范围过窄时拉伸评分"""
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
    """应用 debug projection adapter 使用的相同朝向控制"""
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
    """多数邻居为 free 时填补单 cell 裂缝"""
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
    """填补被 free 包围的小型非 free 空洞"""
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
    """移除不足以支撑 graph node 的 free 小岛"""
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
    """从 bool mask 中收集一个 8 连通域"""
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
    """估算小型非 free 区域是否被 free 包围"""
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


def decode_grid_map_layer(name: str, array_msg: Float32MultiArray, msg: GridMap) -> np.ndarray:
    """解码单个 GridMap layer 到逻辑 row 和 column 顺序"""
    layer = decode_multiarray(name, array_msg)
    return unwrap_grid_map_buffer(
        layer,
        outer_start_index=int(msg.outer_start_index),
        inner_start_index=int(msg.inner_start_index),
    )


def decode_multiarray(name: str, array_msg: Float32MultiArray) -> np.ndarray:
    """按 MultiArray layout 解码 GridMap layer 原始数组"""
    raw_data = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim
    data_offset = max(0, int(array_msg.layout.data_offset))

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0 = dims[0].label
        label1 = dims[1].label
        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size
            cols = dims[1].size
            return _decode_strided_layer(name, raw_data, data_offset, rows, cols, dims[0].stride, dims[1].stride, "C")
        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size
            rows = dims[1].size
            return _decode_strided_layer(name, raw_data, data_offset, rows, cols, dims[1].stride, dims[0].stride, "F")

    if len(dims) >= 2:
        rows = dims[1].size
        cols = dims[0].size
        return _reshape_checked(name, raw_data[data_offset:], rows, cols, "C")

    data = raw_data[data_offset:]
    side = int(np.ceil(np.sqrt(data.size))) if data.size else 0
    return _reshape_checked(name, data, side, side, "C")


def _decode_strided_layer(
    name: str,
    data: np.ndarray,
    data_offset: int,
    rows: int,
    cols: int,
    row_stride: int,
    col_stride: int,
    fallback_order: str,
) -> np.ndarray:
    """使用 MultiArray stride 解码 row-major 或 column-major layer"""
    if rows * cols == 0:
        return np.empty((rows, cols), dtype=np.float32)

    if row_stride <= 0 or col_stride <= 0:
        return _reshape_checked(name, data[data_offset:], rows, cols, fallback_order)

    max_index = data_offset + (rows - 1) * row_stride + (cols - 1) * col_stride
    if max_index >= data.size:
        return _reshape_checked(name, data[data_offset:], rows, cols, fallback_order)

    row_offsets = np.arange(rows, dtype=np.int64) * int(row_stride)
    col_offsets = np.arange(cols, dtype=np.int64) * int(col_stride)
    indices = data_offset + row_offsets[:, None] + col_offsets[None, :]
    return data[indices].astype(np.float32, copy=False)


def _reshape_checked(name: str, data: np.ndarray, rows: int, cols: int, order: str) -> np.ndarray:
    if rows * cols != data.size:
        raise ValueError(f"Layer '{name}' layout size does not match data length")
    return data.reshape((rows, cols), order=order)


def unwrap_grid_map_buffer(
    layer: np.ndarray,
    outer_start_index: int,
    inner_start_index: int,
) -> np.ndarray:
    """按 GridMap circular buffer start index 还原逻辑 cell 顺序"""
    if layer.size == 0:
        return layer
    row_shift = -(outer_start_index % layer.shape[0])
    col_shift = -(inner_start_index % layer.shape[1])
    if row_shift == 0 and col_shift == 0:
        return layer
    return np.roll(layer, shift=(row_shift, col_shift), axis=(0, 1))


def yaw_from_quaternion(quaternion) -> float:
    """从 quaternion 提取 map yaw"""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
