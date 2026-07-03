from __future__ import annotations

from math import cos, pi, sin
import sys
import types
from types import SimpleNamespace

import numpy as np

sys.modules.setdefault("grid_map_msgs", types.ModuleType("grid_map_msgs"))
sys.modules.setdefault("grid_map_msgs.msg", types.ModuleType("grid_map_msgs.msg"))
sys.modules["grid_map_msgs.msg"].GridMap = object
sys.modules.setdefault("nav_msgs", types.ModuleType("nav_msgs"))
sys.modules.setdefault("nav_msgs.msg", types.ModuleType("nav_msgs.msg"))
sys.modules["nav_msgs.msg"].OccupancyGrid = object
sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
sys.modules.setdefault("std_msgs.msg", types.ModuleType("std_msgs.msg"))
sys.modules["std_msgs.msg"].Float32MultiArray = object

from graph_construction.grid_adapter import classify_grid_map, decode_grid_map_layer


def test_grid_map_decode_uses_data_offset_and_rolling_indices():
    base = np.arange(12, dtype=np.float32).reshape((3, 4))
    msg = _grid_map_message(base, outer_start_index=1, inner_start_index=2, data_offset=2)

    decoded = decode_grid_map_layer("traversability", msg.data[0], msg)

    np.testing.assert_array_equal(decoded, np.roll(base, shift=(-1, -2), axis=(0, 1)))


def test_grid_map_decode_column_row_layout():
    base = np.arange(12, dtype=np.float32).reshape((3, 4))
    array_msg = _multi_array(
        base,
        labels=("column_index", "row_index"),
        data=base.reshape(-1, order="F"),
    )
    msg = _grid_map_message(base)
    msg.data = [array_msg]

    decoded = decode_grid_map_layer("traversability", msg.data[0], msg)

    np.testing.assert_array_equal(decoded, base)


def test_grid_map_coordinate_round_trip_without_yaw():
    base = np.ones((4, 3), dtype=np.float32)
    msg = _grid_map_message(base, center_x=10.0, center_y=20.0, resolution=1.0)

    grid = classify_grid_map(
        msg,
        traversability_layer="traversability",
        elevation_layer="elevation",
        free_threshold=0.5,
        obstacle_threshold=0.1,
        normalize_traversability=False,
        normalize_low_quantile=0.05,
        normalize_high_quantile=0.95,
        z_offset=0.0,
        enable_postprocess=False,
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
        transpose=False,
        flip_x=False,
        flip_y=False,
    )

    assert grid.grid_to_world(0, 0)[:2] == (11.5, 21.0)
    _assert_round_trip(grid)


def test_grid_map_coordinate_round_trip_with_yaw():
    base = np.ones((4, 3), dtype=np.float32)
    msg = _grid_map_message(base, center_x=10.0, center_y=20.0, resolution=1.0, yaw=pi * 0.5)

    grid = classify_grid_map(
        msg,
        traversability_layer="traversability",
        elevation_layer="elevation",
        free_threshold=0.5,
        obstacle_threshold=0.1,
        normalize_traversability=False,
        normalize_low_quantile=0.05,
        normalize_high_quantile=0.95,
        z_offset=0.0,
        enable_postprocess=False,
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
        transpose=False,
        flip_x=False,
        flip_y=False,
    )

    x, y, _ = grid.grid_to_world(0, 0)
    assert np.isclose(x, 9.0)
    assert np.isclose(y, 21.5)
    _assert_round_trip(grid)


def _assert_round_trip(grid):
    for iy in range(grid.height):
        for ix in range(grid.width):
            x, y, _ = grid.grid_to_world(ix, iy)
            assert grid.world_to_grid(x, y) == (ix, iy)


def _grid_map_message(
    layer: np.ndarray,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    resolution: float = 1.0,
    yaw: float = 0.0,
    outer_start_index: int = 0,
    inner_start_index: int = 0,
    data_offset: int = 0,
):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"),
        layers=["traversability", "elevation"],
        data=[
            _multi_array(layer, data_offset=data_offset),
            _multi_array(layer * 0.0, data_offset=data_offset),
        ],
        outer_start_index=outer_start_index,
        inner_start_index=inner_start_index,
        info=SimpleNamespace(
            resolution=resolution,
            length_x=float(layer.shape[0]) * resolution,
            length_y=float(layer.shape[1]) * resolution,
            pose=SimpleNamespace(
                position=SimpleNamespace(x=center_x, y=center_y),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=sin(yaw * 0.5), w=cos(yaw * 0.5)),
            ),
        ),
    )


def _multi_array(
    layer: np.ndarray,
    *,
    labels=("row_index", "column_index"),
    data=None,
    data_offset: int = 0,
):
    rows, cols = layer.shape
    if labels == ("row_index", "column_index"):
        dims = [
            SimpleNamespace(label="row_index", size=rows, stride=cols),
            SimpleNamespace(label="column_index", size=cols, stride=1),
        ]
        values = layer.reshape(-1, order="C") if data is None else np.asarray(data, dtype=np.float32)
    elif labels == ("column_index", "row_index"):
        dims = [
            SimpleNamespace(label="column_index", size=cols, stride=rows),
            SimpleNamespace(label="row_index", size=rows, stride=1),
        ]
        values = layer.reshape(-1, order="F") if data is None else np.asarray(data, dtype=np.float32)
    else:
        raise ValueError(labels)

    return SimpleNamespace(
        layout=SimpleNamespace(dim=dims, data_offset=int(data_offset)),
        data=[0.0] * int(data_offset) + values.astype(np.float32).tolist(),
    )
