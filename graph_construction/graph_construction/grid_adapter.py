from __future__ import annotations

from math import atan2
from typing import Set, Tuple

from grid_map_msgs.msg import GridMap
import numpy as np
from scipy import ndimage
from std_msgs.msg import Float32MultiArray

from graph_construction.grid_types import ClassifiedGrid, GridIndex


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
    min_free_component_cells: int,
    fill_hole_max_cells: int,
    fill_hole_min_free_neighbor_ratio: float,
    majority_fill_iterations: int,
    majority_fill_min_neighbors: int,
    fill_elevation_radius_cells: int = 5,
    variance_layer: str | None = None,
    initializer_variance: float | None = None,
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

    initializer_prior = np.zeros(trav.shape, dtype=bool)
    if (
        variance_layer
        and initializer_variance is not None
        and variance_layer in layers
    ):
        variance = decode_grid_map_layer(
            variance_layer,
            layers[variance_layer],
            msg,
        )
        initializer_prior = (
            valid
            & np.isfinite(variance)
            & (variance >= float(initializer_variance))
        )
        valid &= ~initializer_prior

    trav = normalize_grid_map_layer(
        trav,
        valid,
        enabled=normalize_traversability,
        low_quantile=normalize_low_quantile,
        high_quantile=normalize_high_quantile,
    )
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
        min_free_component_cells=min_free_component_cells,
        fill_hole_max_cells=fill_hole_max_cells,
        fill_hole_min_free_neighbor_ratio=fill_hole_min_free_neighbor_ratio,
        majority_fill_iterations=majority_fill_iterations,
        majority_fill_min_neighbors=majority_fill_min_neighbors,
    )
    elevation_filled_count = 0
    if elevation is not None:
        elevation, elevation_filled_count = fill_elevation_for_free_cells(
            elevation,
            free,
            max_radius_cells=fill_elevation_radius_cells,
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
            "initializer_prior": int(np.count_nonzero(initializer_prior)),
            "free": int(np.count_nonzero(free)),
            "obstacle": int(np.count_nonzero(obstacle)),
            "unknown": int(np.count_nonzero(unknown)),
            "elevation_filled": elevation_filled_count,
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
    min_free_component_cells: int,
    fill_hole_max_cells: int,
    fill_hole_min_free_neighbor_ratio: float,
    majority_fill_iterations: int,
    majority_fill_min_neighbors: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在 graph 采样前清理 GridMap 分类拓扑"""
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


def fill_elevation_for_free_cells(
    elevation: np.ndarray,
    free: np.ndarray,
    max_radius_cells: int,
) -> Tuple[np.ndarray, int]:
    """为已判定 free 的小洞补邻近 elevation"""
    radius = max(0, int(max_radius_cells))
    if radius <= 0:
        return elevation, 0

    processed = elevation.copy()
    finite_source = np.isfinite(elevation)
    holes = free & ~finite_source
    filled_count = 0
    height, width = processed.shape

    for iy, ix in np.argwhere(holes):
        y0 = max(0, int(iy) - radius)
        y1 = min(height, int(iy) + radius + 1)
        x0 = max(0, int(ix) - radius)
        x1 = min(width, int(ix) + radius + 1)
        local_values = elevation[y0:y1, x0:x1]
        local_finite = finite_source[y0:y1, x0:x1]
        if not np.any(local_finite):
            continue
        processed[iy, ix] = float(np.median(local_values[local_finite]))
        filled_count += 1

    return processed, filled_count


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
    component_labels, component_count = ndimage.label(
        target,
        structure=np.ones((3, 3), dtype=np.uint8),
    )
    if component_count == 0:
        return processed

    component_sizes = np.bincount(component_labels.ravel(), minlength=component_count + 1)
    border_labels = np.unique(
        np.concatenate(
            (
                component_labels[0, :],
                component_labels[-1, :],
                component_labels[:, 0],
                component_labels[:, -1],
            )
        )
    )
    boundary_count = np.zeros(component_count + 1, dtype=np.int64)
    free_boundary_count = np.zeros(component_count + 1, dtype=np.int64)
    free = processed == free_value

    # Count the same directed component boundary contacts as the previous cell loop
    for dx, dy in _NEIGHBOR_OFFSETS_8:
        src_y, dst_y = _shift_slices(processed.shape[0], dy)
        src_x, dst_x = _shift_slices(processed.shape[1], dx)
        source_labels = component_labels[src_y, src_x]
        neighbor_labels = component_labels[dst_y, dst_x]
        boundary = (source_labels > 0) & (neighbor_labels != source_labels)
        if not np.any(boundary):
            continue
        labels = source_labels[boundary]
        boundary_count += np.bincount(labels, minlength=component_count + 1)
        free_contacts = boundary & free[dst_y, dst_x]
        if np.any(free_contacts):
            free_boundary_count += np.bincount(
                source_labels[free_contacts],
                minlength=component_count + 1,
            )

    ratios = np.divide(
        free_boundary_count,
        boundary_count,
        out=np.zeros(component_count + 1, dtype=float),
        where=boundary_count > 0,
    )
    fill_labels = (
        (component_sizes <= max_cells)
        & (ratios >= min_free_neighbor_ratio)
    )
    fill_labels[border_labels] = False
    fill_labels[0] = False
    processed[fill_labels[component_labels]] = free_value

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
    component_labels, component_count = ndimage.label(
        free,
        structure=np.ones((3, 3), dtype=np.uint8),
    )
    if component_count == 0:
        return processed
    component_sizes = np.bincount(component_labels.ravel(), minlength=component_count + 1)
    small_labels = component_sizes < min_cells
    small_labels[0] = False
    processed[small_labels[component_labels]] = replacement_value

    return processed


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
