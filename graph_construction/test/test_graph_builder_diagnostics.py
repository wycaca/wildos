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


def test_graph_builder_preserves_historical_nodes_when_grid_becomes_unknown():
    """unknown 只表示当前不可观测, 不能删除曾确认安全的历史路线"""
    unknown = np.ones((5, 5), dtype=bool)
    grid = ClassifiedGrid(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.zeros((5, 5), dtype=bool),
        obstacle=np.zeros((5, 5), dtype=bool),
        unknown=unknown,
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(min_obstacle_clearance=0.0, edge_radius=3.0)
    )
    first = builder.graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    second = builder.graph.create_node(position=(2.5, 2.5, 0.0), stamp_seconds=1.0)
    builder.graph.set_edges([InternalEdge(from_id=first.node_id, to_id=second.node_id, cost=1.0)])

    result = builder.update(grid, robot_position=(2.5, 2.5, 0.0), stamp_seconds=2.0)

    assert first.node_id in result.graph.nodes
    assert second.node_id in result.graph.nodes
    assert (first.node_id, second.node_id) in result.graph.edges


def test_graph_builder_updates_node_height_from_its_own_surface_cell():
    """长坡历史节点跟随自身地面高度, 不与机器人当前高度直接比较"""
    elevation = np.full((5, 5), 2.0, dtype=np.float32)
    grid = ClassifiedGrid(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((5, 5), dtype=bool),
        obstacle=np.zeros((5, 5), dtype=bool),
        unknown=np.zeros((5, 5), dtype=bool),
        elevation=elevation,
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=10,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
        )
    )
    node = builder.graph.create_node(position=(1.5, 1.5, 0.0), stamp_seconds=1.0)

    result = builder.update(grid, robot_position=(2.5, 2.5, 8.0), stamp_seconds=2.0)

    assert result.graph.nodes[node.node_id].position[2] == 2.0


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


def test_graph_builder_adds_robot_anchor_when_robot_cell_is_unknown():
    """验证脚下点云缺失时用机器人锚点接回近邻 graph"""
    free = np.ones((5, 5), dtype=bool)
    obstacle = np.zeros((5, 5), dtype=bool)
    unknown = np.zeros((5, 5), dtype=bool)
    free[2, 2] = False
    unknown[2, 2] = True
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
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=2,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
            current_node_max_edge_neighbors=4,
        )
    )

    result = builder.update(grid, robot_position=(2.5, 2.5, 0.0), stamp_seconds=1.0)
    current_id = result.graph.current_node_id
    assert current_id is not None
    assert result.graph.nodes[current_id].is_robot_anchor
    assert result.diagnostics.current_node_status == "robot_anchor"
    assert any(
        edge.from_id == current_id or edge.to_id == current_id
        for edge in result.graph.edges.values()
    )


def test_graph_builder_bootstraps_free_component_across_large_unknown_footprint():
    """Unity 脚下大洞时从 anchor 连边半径内的最近 free 区域恢复采样"""
    free = np.ones((11, 11), dtype=bool)
    obstacle = np.zeros((11, 11), dtype=bool)
    unknown = np.zeros((11, 11), dtype=bool)
    free[3:8, 3:8] = False
    unknown[3:8, 3:8] = True
    grid = ClassifiedGrid(
        width=11,
        height=11,
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
            edge_radius=4.0,
        )
    )

    result = builder.update(grid, robot_position=(5.5, 5.5, 0.0), stamp_seconds=1.0)

    assert len(result.graph.nodes) > 1
    assert len(result.graph.edges) > 0


def test_robot_anchor_leaves_persistent_breadcrumbs_in_unknown_ground():
    """连续脚下缺图时固化旧 anchor, 保留长距离走过的拓扑路线"""
    grid = ClassifiedGrid(
        width=8,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.zeros((3, 8), dtype=bool),
        obstacle=np.zeros((3, 8), dtype=bool),
        unknown=np.ones((3, 8), dtype=bool),
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            min_node_separation=1.0,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    builder.update(grid, robot_position=(1.5, 1.5, 0.0), stamp_seconds=1.0)
    result = builder.update(grid, robot_position=(3.0, 1.5, 0.0), stamp_seconds=2.0)

    anchors = [node for node in result.graph.nodes.values() if node.is_robot_anchor]
    breadcrumbs = [node for node in result.graph.nodes.values() if not node.is_robot_anchor]
    assert len(anchors) == 1
    assert any(node.position[:2] == (1.5, 1.5) for node in breadcrumbs)
    assert any(
        anchors[0].node_id in (edge.from_id, edge.to_id)
        for edge in result.graph.edges.values()
    )


def test_graph_builder_preserves_nodes_disconnected_from_current_component():
    """局部断边不能删除持久图中的其他历史 component"""
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
        )
    )
    historical = builder.graph.create_node(
        position=(9.5, 2.5, 0.0),
        stamp_seconds=0.0,
    )

    result = builder.update(grid, robot_position=(0.5, 0.5, 0.0), stamp_seconds=1.0)

    assert result.diagnostics.connected_components == 2
    assert result.diagnostics.node_count > 0
    assert historical.node_id in result.graph.nodes
    assert any(node.position[0] < 4.0 for node in result.graph.nodes.values())
    assert any(node.position[0] >= 8.0 for node in result.graph.nodes.values())
