from __future__ import annotations

from math import cos, pi, sin
import sys
import types
from types import SimpleNamespace

import numpy as np

try:
    from grid_map_msgs.msg import GridMap  # noqa: F401
    from std_msgs.msg import Float32MultiArray  # noqa: F401
except ImportError:
    sys.modules.setdefault("grid_map_msgs", types.ModuleType("grid_map_msgs"))
    sys.modules.setdefault("grid_map_msgs.msg", types.ModuleType("grid_map_msgs.msg"))
    sys.modules["grid_map_msgs.msg"].GridMap = object
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
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
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
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
    )

    x, y, _ = grid.grid_to_world(0, 0)
    assert np.isclose(x, 9.0)
    assert np.isclose(y, 21.5)
    _assert_round_trip(grid)


def test_grid_map_postprocess_fills_elevation_for_small_free_hole():
    trav = np.ones((5, 5), dtype=np.float32)
    elevation = np.ones((5, 5), dtype=np.float32) * 1.2
    elevation[2, 2] = np.nan
    msg = _grid_map_message(trav, elevation=elevation)

    grid = classify_grid_map(
        msg,
        traversability_layer="traversability",
        elevation_layer="elevation",
        free_threshold=0.5,
        obstacle_threshold=0.1,
        normalize_traversability=False,
        normalize_low_quantile=0.05,
        normalize_high_quantile=0.95,
        z_offset=0.08,
        min_free_component_cells=1,
        fill_hole_max_cells=4,
        fill_hole_min_free_neighbor_ratio=0.5,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
        fill_elevation_radius_cells=1,
    )

    assert grid.free[2, 2]
    assert np.isclose(grid.elevation[2, 2], 1.2)
    assert grid.stats["elevation_filled"] == 1


def test_grid_map_treats_unscored_initializer_cells_as_unknown():
    traversability = np.zeros((5, 5), dtype=np.float32)
    elevation = np.zeros((5, 5), dtype=np.float32)
    variance = np.full((5, 5), 10.0, dtype=np.float32)
    msg = _grid_map_message(
        traversability,
        elevation=elevation,
        variance=variance,
    )

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
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
        variance_layer="variance",
        initializer_variance=10.0,
    )

    assert not np.any(grid.free)
    assert not np.any(grid.obstacle)
    assert np.all(grid.unknown)
    assert grid.stats["initializer_prior"] == 25


def _assert_round_trip(grid):
    for iy in range(grid.height):
        for ix in range(grid.width):
            x, y, _ = grid.grid_to_world(ix, iy)
            assert grid.world_to_grid(x, y) == (ix, iy)


def _grid_map_message(
    layer: np.ndarray,
    *,
    elevation: np.ndarray | None = None,
    variance: np.ndarray | None = None,
    center_x: float = 0.0,
    center_y: float = 0.0,
    resolution: float = 1.0,
    yaw: float = 0.0,
    outer_start_index: int = 0,
    inner_start_index: int = 0,
    data_offset: int = 0,
):
    elevation_layer = layer * 0.0 if elevation is None else elevation
    layers = ["traversability", "elevation"]
    data = [
        _multi_array(layer, data_offset=data_offset),
        _multi_array(elevation_layer, data_offset=data_offset),
    ]
    if variance is not None:
        layers.append("variance")
        data.append(_multi_array(variance, data_offset=data_offset))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"),
        layers=layers,
        data=data,
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
