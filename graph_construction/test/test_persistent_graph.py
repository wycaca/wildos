import numpy as np

from graph_construction.frontier_detector import FrontierDetector
from graph_construction.graph_builder import GraphBuilderConfig, SparseGraphBuilder
from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.grid_types import ClassifiedGrid


def _grid(
    *,
    width: int = 6,
    height: int = 6,
    origin_x: float = 0.0,
    free: np.ndarray | None = None,
    obstacle: np.ndarray | None = None,
    unknown: np.ndarray | None = None,
) -> ClassifiedGrid:
    """构造持久图测试使用的轴对齐局部栅格"""
    shape = (height, width)
    return ClassifiedGrid(
        width=width,
        height=height,
        resolution=1.0,
        origin_x=origin_x,
        origin_y=0.0,
        frame_id="odom",
        free=np.ones(shape, dtype=bool) if free is None else free,
        obstacle=np.zeros(shape, dtype=bool) if obstacle is None else obstacle,
        unknown=np.zeros(shape, dtype=bool) if unknown is None else unknown,
    )


def _frontier_detector() -> FrontierDetector:
    """创建不过滤单点 frontier 的确定性测试实例"""
    return FrontierDetector(
        frontier_assign_radius=3.0,
        frontier_min_points=1,
        frontier_min_span=0.0,
        frontier_border_margin=0.0,
        frontier_candidate_spacing=0.0,
        frontier_visited_corridor_radius=0.65,
    )


def test_historical_frontier_outside_rolling_grid_becomes_inactive():
    """局部窗口移走后只保留 owner 拓扑, 不保留活动 Frontier 状态"""
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    owner.frontier_points = [(2.5, 2.5, 0.0)]
    owner.is_frontier = True

    _frontier_detector().assign_frontiers(
        graph,
        _grid(origin_x=20.0),
        frontier_cells=[],
    )

    assert owner.node_id in graph.nodes
    assert not owner.is_frontier
    assert owner.frontier_points == []


def test_historical_frontier_is_removed_when_reobserved_as_known_free():
    """历史 frontier 再次可见且不再接触 unknown 时才允许删除"""
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    owner.frontier_points = [(2.5, 2.5, 0.0)]
    owner.is_frontier = True

    _frontier_detector().assign_frontiers(
        graph,
        _grid(),
        frontier_cells=[],
    )

    assert not owner.is_frontier
    assert owner.frontier_points == []


def test_visible_historical_frontier_keeps_original_owner():
    """同一物理 frontier 仍有效时不能被每帧最近邻重新分配 owner"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    grid = _grid(free=free, unknown=unknown)
    graph = GraphState()
    historical_owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    closer_node = graph.create_node(position=(2.5, 1.5, 0.0), stamp_seconds=1.0)
    historical_owner.frontier_points = [(2.5, 2.5, 0.0)]
    historical_owner.is_frontier = True

    _frontier_detector().assign_frontiers(
        graph,
        grid,
        frontier_cells=[(2, 2)],
    )

    assert historical_owner.frontier_points == [(2.5, 2.5, 0.0)]
    assert closer_node.frontier_points == []


def test_frontier_inside_persistent_explored_area_is_not_recreated():
    """历史节点已覆盖的区域不能因局部 unknown 再次生成活动 Frontier"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    graph = GraphState()
    explored_node = graph.create_node(position=(2.5, 2.5, 0.0), stamp_seconds=1.0)
    explored_node.explored_radius = 2.0

    _frontier_detector().assign_frontiers(
        graph,
        _grid(free=free, unknown=unknown),
        frontier_cells=[(2, 2)],
    )

    assert not explored_node.is_frontier
    assert explored_node.frontier_points == []


def test_nonfinite_explored_radius_does_not_hide_current_frontier():
    """失效的无限探索半径不能把当前可见 Frontier 全部过滤"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    graph = GraphState()
    stale_node = graph.create_node(position=(0.5, 0.5, 0.0), stamp_seconds=1.0)
    stale_node.explored_radius = float("inf")
    owner = graph.create_node(position=(2.5, 2.5, 0.0), stamp_seconds=1.0)

    _frontier_detector().assign_frontiers(
        graph,
        _grid(free=free, unknown=unknown),
        frontier_cells=[(2, 2)],
    )

    assert owner.is_frontier
    assert owner.frontier_points == [(2.5, 2.5, 0.0)]


def test_frontier_inside_visited_trajectory_corridor_is_not_created():
    """已走过平坦区域附近的 unknown 边缘不能重新成为活动 Frontier"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    graph.trajectory_points.append((2.5, 2.5, 0.0))

    _frontier_detector().assign_frontiers(
        graph,
        _grid(free=free, unknown=unknown),
        frontier_cells=[(2, 2)],
    )

    assert not owner.is_frontier
    assert owner.frontier_points == []


def test_historical_frontier_entering_visited_corridor_is_removed():
    """机器人走过历史 Frontier 后必须清除对应紫色边界点"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    owner.frontier_points = [(2.5, 2.5, 0.0)]
    owner.is_frontier = True
    graph.trajectory_points.append((2.5, 2.5, 0.0))

    _frontier_detector().assign_frontiers(
        graph,
        _grid(free=free, unknown=unknown),
        frontier_cells=[(2, 2)],
    )

    assert not owner.is_frontier
    assert owner.frontier_points == []


def test_frontier_outside_visited_corridor_remains_available():
    """轨迹走廊外的真实侧向分支仍应保留为活动 Frontier"""
    free = np.ones((6, 6), dtype=bool)
    unknown = np.zeros((6, 6), dtype=bool)
    unknown[2, 3] = True
    free[2, 3] = False
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    graph.trajectory_points.append((2.5, 1.5, 0.0))

    _frontier_detector().assign_frontiers(
        graph,
        _grid(free=free, unknown=unknown),
        frontier_cells=[(2, 2)],
    )

    assert owner.is_frontier
    assert owner.frontier_points == [(2.5, 2.5, 0.0)]


def test_navigation_clearance_does_not_delete_historical_node():
    """低于新边安全阈值的历史节点仍应保留为路线记忆"""
    obstacle = np.zeros((5, 5), dtype=bool)
    free = np.ones((5, 5), dtype=bool)
    obstacle[2, 3] = True
    free[2, 3] = False
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=10,
            min_node_separation=0.1,
            min_obstacle_clearance=2.0,
        )
    )
    node = builder.graph.create_node(position=(2.5, 2.5, 0.0), stamp_seconds=1.0)

    result = builder.update(
        _grid(width=5, height=5, free=free, obstacle=obstacle),
        robot_position=(0.5, 0.5, 0.0),
        stamp_seconds=2.0,
    )

    assert node.node_id in result.graph.nodes
    assert result.graph.nodes[node.node_id].free_radius < 2.0


def test_long_rolling_map_sequence_preserves_original_nodes_and_edge():
    """连续跨越多个局部窗口后, 起点拓扑仍必须存在"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            min_node_separation=1.0,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )
    first = builder.graph.create_node(position=(0.5, 1.5, 0.0), stamp_seconds=1.0)
    second = builder.graph.create_node(position=(1.5, 1.5, 0.0), stamp_seconds=1.0)
    edge_key = (first.node_id, second.node_id)
    builder.graph.set_edges([InternalEdge(from_id=edge_key[0], to_id=edge_key[1], cost=1.0)])

    for update_index, origin_x in enumerate((6.0, 12.0, 18.0), start=2):
        unknown = np.ones((3, 6), dtype=bool)
        result = builder.update(
            _grid(
                width=6,
                height=3,
                origin_x=origin_x,
                free=np.zeros((3, 6), dtype=bool),
                unknown=unknown,
            ),
            robot_position=(origin_x + 0.5, 1.5, 0.0),
            stamp_seconds=float(update_index),
        )

    assert first.node_id in result.graph.nodes
    assert second.node_id in result.graph.nodes
    assert edge_key in result.graph.edges


def test_graph_spatial_and_adjacency_indices_follow_mutations():
    """节点移动删除和局部边替换必须同步派生索引"""
    graph = GraphState()
    first = graph.create_node(position=(0.5, 0.5, 0.0), stamp_seconds=1.0)
    second = graph.create_node(position=(1.5, 0.5, 0.0), stamp_seconds=1.0)
    third = graph.create_node(position=(20.5, 0.5, 0.0), stamp_seconds=1.0)
    local_key = (first.node_id, second.node_id)
    remote_key = (second.node_id, third.node_id)
    graph.set_edges(
        [
            InternalEdge(*local_key, cost=1.0),
            InternalEdge(*remote_key, cost=19.0),
        ]
    )

    assert graph.node_ids_within((0.5, 0.5, 0.0), 1.1) == {
        first.node_id,
        second.node_id,
    }
    assert graph.edge_keys_for_nodes({first.node_id}) == {local_key}

    graph.move_node(first.node_id, (10.5, 0.5, 0.0))
    assert first.node_id not in graph.node_ids_within((0.5, 0.5, 0.0), 1.1)
    assert first.node_id in graph.node_ids_within((10.5, 0.5, 0.0), 0.1)

    graph.remove_node(second.node_id)
    assert local_key not in graph.edges
    assert remote_key not in graph.edges
    assert graph.edge_keys_for_nodes({first.node_id, third.node_id}) == set()


def test_graph_update_only_touches_local_history():
    """远处历史节点和边不应进入当前局部更新集合"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=10,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )
    local_a = builder.graph.create_node(
        position=(0.5, 1.5, 0.0),
        stamp_seconds=1.0,
    )
    local_b = builder.graph.create_node(
        position=(1.5, 1.5, 0.0),
        stamp_seconds=1.0,
    )
    remote_a = builder.graph.create_node(
        position=(100.5, 1.5, 0.0),
        stamp_seconds=1.0,
    )
    remote_b = builder.graph.create_node(
        position=(101.5, 1.5, 0.0),
        stamp_seconds=1.0,
    )
    local_key = (local_a.node_id, local_b.node_id)
    remote_key = (remote_a.node_id, remote_b.node_id)
    builder.graph.set_edges(
        [
            InternalEdge(*local_key, cost=1.0),
            InternalEdge(*remote_key, cost=1.0),
        ]
    )
    remote_edge = builder.graph.edges[remote_key]

    result = builder.update(
        _grid(width=6, height=3),
        robot_position=(0.5, 1.5, 0.0),
        stamp_seconds=2.0,
    )

    assert result.graph.edges[remote_key] is remote_edge
    assert result.stats.local_node_count < result.stats.total_node_count
    assert remote_key not in result.graph.edge_keys_for_nodes(
        result.graph.node_ids_in_bounds(-3.0, 9.0, -3.0, 6.0)
    )


def test_identical_grid_skips_stable_edge_rebuild():
    """地图和机器人未变化时不重复重建稳定边"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=2,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    first = builder.update(
        _grid(width=12, height=12),
        robot_position=(5.5, 5.5, 0.0),
        stamp_seconds=1.0,
    )
    stable_edges = dict(first.graph.edges)
    second = builder.update(
        _grid(width=12, height=12),
        robot_position=(5.5, 5.5, 0.0),
        stamp_seconds=2.0,
    )

    assert first.stats.edge_rebuild_node_count > 0
    assert second.stats.dirty_cell_count == 0
    assert second.stats.edge_rebuild_node_count == 0
    assert second.stats.affected_edge_count == 0
    assert second.graph.edges == stable_edges


def test_rolling_grid_marks_only_entering_cells_dirty():
    """平移一格的地图只把新进入窗口的 cell 标为变化"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=10,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=2.0,
        )
    )
    builder.update(
        _grid(width=6, height=6),
        robot_position=(1.5, 2.5, 0.0),
        stamp_seconds=1.0,
    )

    shifted = builder.update(
        _grid(width=6, height=6, origin_x=1.0),
        robot_position=(1.5, 2.5, 0.0),
        stamp_seconds=2.0,
    )

    assert shifted.stats.dirty_cell_count == 6


def test_single_cell_change_rebuilds_only_nearby_edges():
    """局部障碍变化不应触发整个窗口的边更新"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            sample_stride=1,
            min_node_separation=0.1,
            min_obstacle_clearance=0.0,
            edge_radius=2.0,
        )
    )
    builder.update(
        _grid(width=20, height=20),
        robot_position=(2.5, 2.5, 0.0),
        stamp_seconds=1.0,
    )
    free = np.ones((20, 20), dtype=bool)
    obstacle = np.zeros((20, 20), dtype=bool)
    free[10, 10] = False
    obstacle[10, 10] = True

    changed = builder.update(
        _grid(width=20, height=20, free=free, obstacle=obstacle),
        robot_position=(2.5, 2.5, 0.0),
        stamp_seconds=2.0,
    )

    assert changed.stats.dirty_cell_count == 1
    assert 0 < changed.stats.edge_rebuild_node_count < changed.stats.local_node_count
    assert changed.stats.affected_edge_count < changed.stats.total_edge_count
