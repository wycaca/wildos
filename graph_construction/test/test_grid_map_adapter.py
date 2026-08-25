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

from graph_construction.grid_adapter import (
    build_repaired_grid_map,
    classify_grid_map,
    decode_grid_map_layer,
    postprocess_classification,
)


def test_repaired_grid_map_writes_filled_elevation_in_circular_buffer_order():
    elevation = np.full((3, 4), np.nan, dtype=np.float32)
    msg = _grid_map_message(
        np.ones((3, 4), dtype=np.float32),
        elevation=elevation,
        outer_start_index=1,
        inner_start_index=2,
        data_offset=2,
    )
    grid = classify_grid_map(
        msg,
        traversability_layer="traversability",
        elevation_layer="elevation",
        free_threshold=0.5,
        obstacle_threshold=0.1,
        z_offset=0.0,
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
    )
    grid.free[1, 2] = True
    grid.elevation[1, 2] = -0.65

    repaired, filled = build_repaired_grid_map(msg, grid)

    decoded = decode_grid_map_layer("elevation", repaired.data[1], repaired)
    assert filled == 1
    assert np.isclose(decoded[1, 2], -0.65)
    assert np.isnan(decode_grid_map_layer("elevation", msg.data[1], msg)[1, 2])


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


def test_grid_map_treats_aged_initializer_cells_as_unknown():
    traversability = np.zeros((5, 5), dtype=np.float32)
    elevation = np.zeros((5, 5), dtype=np.float32)
    variance = np.full((5, 5), 10.42, dtype=np.float32)
    variance[0, 0] = 1000.5
    variance[4, 4] = 0.1
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
    assert grid.obstacle[4, 4]
    assert np.count_nonzero(grid.unknown) == 24
    assert grid.stats["initializer_prior"] == 24


def test_majority_fill_only_promotes_unknown_cells():
    free = np.ones((5, 5), dtype=bool)
    obstacle = np.zeros((5, 5), dtype=bool)
    unknown = np.zeros((5, 5), dtype=bool)
    obstacle[2, 2] = True

    processed_free, processed_obstacle, _ = postprocess_classification(
        free,
        obstacle,
        unknown,
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=1,
        majority_fill_min_neighbors=6,
    )

    assert processed_obstacle[2, 2]
    assert not processed_free[2, 2]


def test_hole_fill_only_promotes_unknown_regions():
    free = np.ones((5, 7), dtype=bool)
    obstacle = np.zeros((5, 7), dtype=bool)
    unknown = np.zeros((5, 7), dtype=bool)
    obstacle[1:4, 1:3] = True
    unknown[2, 4] = True
    free[obstacle] = False
    free[2, 4] = False

    processed_free, processed_obstacle, processed_unknown = postprocess_classification(
        free,
        obstacle,
        unknown,
        min_free_component_cells=1,
        fill_hole_max_cells=6,
        fill_hole_min_free_neighbor_ratio=1.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
    )

    assert np.all(processed_obstacle[1:4, 1:3])
    assert not np.any(processed_free[1:4, 1:3])
    assert processed_free[2, 4]
    assert not processed_unknown[2, 4]


def test_grid_map_uses_fixed_thresholds_and_rejects_out_of_range_scores():
    traversability = np.array(
        [
            [-0.1, 0.0, 0.05],
            [0.1, 0.19, 0.2],
            [0.8, 1.0, 1.1],
        ],
        dtype=np.float32,
    )
    msg = _grid_map_message(traversability)

    grid = classify_grid_map(
        msg,
        traversability_layer="traversability",
        elevation_layer="elevation",
        free_threshold=0.2,
        obstacle_threshold=0.05,
        z_offset=0.0,
        min_free_component_cells=1,
        fill_hole_max_cells=0,
        fill_hole_min_free_neighbor_ratio=0.0,
        majority_fill_iterations=0,
        majority_fill_min_neighbors=1,
    )

    assert grid.unknown[0, 0]
    assert grid.obstacle[0, 1]
    assert grid.obstacle[0, 2]
    assert grid.unknown[1, 0]
    assert grid.unknown[1, 1]
    assert grid.free[1, 2]
    assert grid.free[2, 0]
    assert grid.free[2, 1]
    assert grid.unknown[2, 2]
    assert grid.stats["out_of_range"] == 2


def test_fixed_cell_classification_does_not_follow_frame_distribution():
    low_background = np.zeros((3, 3), dtype=np.float32)
    high_background = np.ones((3, 3), dtype=np.float32)
    low_background[1, 1] = 0.15
    high_background[1, 1] = 0.15

    def classify(layer):
        return classify_grid_map(
            _grid_map_message(layer),
            traversability_layer="traversability",
            elevation_layer="elevation",
            free_threshold=0.2,
            obstacle_threshold=0.05,
            z_offset=0.0,
            min_free_component_cells=1,
            fill_hole_max_cells=0,
            fill_hole_min_free_neighbor_ratio=0.0,
            majority_fill_iterations=0,
            majority_fill_min_neighbors=1,
        )

    assert classify(low_background).unknown[1, 1]
    assert classify(high_background).unknown[1, 1]


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
