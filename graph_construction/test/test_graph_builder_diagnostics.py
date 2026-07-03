import numpy as np

from graph_construction.graph_builder import GraphBuilderConfig, SparseGraphBuilder
from graph_construction.grid_types import ClassifiedGrid


def test_graph_builder_returns_stage_timing_diagnostics():
    """验证纯算法层会返回 stage timing 和 graph 统计"""
    grid = ClassifiedGrid(
        width=6,
        height=6,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((6, 6), dtype=bool),
        obstacle=np.zeros((6, 6), dtype=bool),
        unknown=np.zeros((6, 6), dtype=bool),
        stats={"free": 36, "obstacle": 0, "unknown": 0},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=3,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=10.0,
        )
    )

    result = builder.update(grid, robot_position=(0.5, 0.5, 0.0), stamp_seconds=1.0)
    diagnostics = result.diagnostics

    assert diagnostics.node_count > 0
    assert diagnostics.connected_components >= 1
    assert diagnostics.degree_max >= diagnostics.degree_min
    assert diagnostics.current_node_status in {"reachable", "geometry_fallback", "missing"}

    expected_stages = {
        "prepare_grid",
        "distance_fields",
        "update_nodes",
        "sample_nodes",
        "update_frontiers",
        "current_node",
        "build_edges",
        "total",
    }
    assert expected_stages.issubset(diagnostics.stage_timings_ms)
    assert all(diagnostics.stage_timings_ms[name] >= 0.0 for name in expected_stages)


def test_frontier_candidate_spacing_reduces_assignment_work():
    """验证 frontier 候选间距会减少进入 owner 分配的边界点"""
    free = np.ones((6, 20), dtype=bool)
    unknown = np.zeros((6, 20), dtype=bool)
    obstacle = np.zeros((6, 20), dtype=bool)
    free[5, :] = False
    unknown[5, :] = True
    grid = ClassifiedGrid(
        width=20,
        height=6,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        stats={"free": int(free.sum()), "obstacle": 0, "unknown": int(unknown.sum())},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=3,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=10.0,
            frontier_border_margin=0.0,
            frontier_candidate_spacing=3.0,
        )
    )

    result = builder.update(grid, robot_position=(0.5, 0.5, 0.0), stamp_seconds=1.0)

    assert result.diagnostics.frontier_cell_count > result.diagnostics.frontier_candidate_count
    assert result.diagnostics.frontier_candidate_count > 0
