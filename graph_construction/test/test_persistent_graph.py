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
    )


def test_historical_frontier_survives_outside_rolling_grid():
    """局部窗口移走后, 历史 frontier 必须保留在全局图中"""
    graph = GraphState()
    owner = graph.create_node(position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    owner.frontier_points = [(2.5, 2.5, 0.0)]
    owner.is_frontier = True

    _frontier_detector().assign_frontiers(
        graph,
        _grid(origin_x=20.0),
        frontier_cells=[],
    )

    assert owner.is_frontier
    assert owner.frontier_points == [(2.5, 2.5, 0.0)]


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
