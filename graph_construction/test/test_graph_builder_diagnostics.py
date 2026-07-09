import numpy as np

from graph_construction.graph_builder import GraphBuilderConfig, SparseGraphBuilder
from graph_construction.edge_builder import EdgeBuilder
from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.grid_types import ClassifiedGrid
from graph_construction.grid_types import distance_to_mask


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


def test_edge_builder_rejects_edges_without_corridor_clearance():
    """验证 graph edge 不只检查中心线, 还要满足 obstacle clearance"""
    free = np.ones((3, 5), dtype=bool)
    obstacle = np.zeros((3, 5), dtype=bool)
    unknown = np.zeros((3, 5), dtype=bool)
    obstacle[1, 2] = True
    free[1, 2] = False
    grid = ClassifiedGrid(
        width=5,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )
    graph = GraphState()
    graph.create_node(position=(0.5, 0.5, 0.0), stamp_seconds=1.0)
    graph.create_node(position=(4.5, 0.5, 0.0), stamp_seconds=1.0)
    edge_builder = EdgeBuilder(edge_radius=10.0, max_neighbors_per_node=4)

    loose_edges = edge_builder.build_edges(
        graph,
        grid,
        distance_to_mask(obstacle, grid.resolution),
        distance_to_mask(unknown, grid.resolution),
        min_clearance=0.5,
    )
    strict_edges = edge_builder.build_edges(
        graph,
        grid,
        distance_to_mask(obstacle, grid.resolution),
        distance_to_mask(unknown, grid.resolution),
        min_clearance=1.1,
    )

    assert len(loose_edges) == 1
    assert strict_edges == []


def test_historical_edge_is_kept_when_current_grid_becomes_unknown():
    """验证当前局部图变 unknown 时不会误删历史边"""
    free = np.zeros((3, 5), dtype=bool)
    obstacle = np.zeros((3, 5), dtype=bool)
    unknown = np.ones((3, 5), dtype=bool)
    grid = ClassifiedGrid(
        width=5,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )
    graph = GraphState()
    graph.create_node(position=(0.5, 1.5, 0.0), stamp_seconds=1.0)
    graph.create_node(position=(4.5, 1.5, 0.0), stamp_seconds=1.0)
    graph.set_edges([InternalEdge(from_id=0, to_id=1, cost=4.0)])
    edge_builder = EdgeBuilder(edge_radius=10.0, max_neighbors_per_node=4)

    current_edges = edge_builder.build_edges(
        graph,
        grid,
        distance_to_mask(obstacle, grid.resolution),
        distance_to_mask(unknown, grid.resolution),
        min_clearance=0.5,
    )
    merged_edges = edge_builder.merge_historical_edges(
        graph,
        current_edges,
        grid,
        distance_to_mask(obstacle, grid.resolution),
        min_clearance=0.5,
    )

    assert current_edges == []
    assert {(edge.from_id, edge.to_id) for edge in merged_edges} == {(0, 1)}


def test_historical_edge_is_removed_when_visible_segment_hits_obstacle():
    """验证跨边界历史边只要可见段碰到障碍就会删除"""
    free = np.ones((5, 5), dtype=bool)
    obstacle = np.zeros((5, 5), dtype=bool)
    unknown = np.zeros((5, 5), dtype=bool)
    obstacle[2, 2] = True
    free[2, 2] = False
    grid = ClassifiedGrid(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )
    graph = GraphState()
    graph.create_node(position=(4.5, 2.5, 0.0), stamp_seconds=1.0)
    graph.create_node(position=(-10.0, 2.5, 0.0), stamp_seconds=1.0)
    graph.set_edges([InternalEdge(from_id=0, to_id=1, cost=14.5)])
    edge_builder = EdgeBuilder(edge_radius=20.0, max_neighbors_per_node=4)

    merged_edges = edge_builder.merge_historical_edges(
        graph,
        [],
        grid,
        distance_to_mask(obstacle, grid.resolution),
        min_clearance=0.0,
    )

    assert merged_edges == []


def test_graph_builder_prunes_nodes_disconnected_from_current_component():
    """验证断开 component 不会发布给 planner 和目标评分"""
    free = np.zeros((5, 12), dtype=bool)
    free[:, :4] = True
    free[:, 8:] = True
    obstacle = np.zeros((5, 12), dtype=bool)
    unknown = ~free
    grid = ClassifiedGrid(
        width=12,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=2,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
            prune_disconnected_nodes=True,
        )
    )

    result = builder.update(grid, robot_position=(0.5, 0.5, 0.0), stamp_seconds=1.0)

    assert result.diagnostics.connected_components == 1
    assert result.diagnostics.node_count > 0
    assert all(node.position[0] < 4.0 for node in result.graph.nodes.values())
