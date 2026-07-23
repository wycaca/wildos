import math

import numpy as np
import pytest

from graph_construction.graph_builder import GraphBuilderConfig, SparseGraphBuilder
from graph_construction.edge_builder import EdgeBuilder
from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.grid_types import ClassifiedGrid
from graph_construction.grid_types import distance_to_mask


def _build_edge_delta(
    edge_builder: EdgeBuilder,
    graph: GraphState,
    grid: ClassifiedGrid,
    min_clearance: float = 0.0,
):
    """使用当前局部节点构建测试边增量"""
    return edge_builder.build_delta(
        graph,
        grid,
        distance_to_mask(grid.obstacle, grid.resolution),
        distance_to_mask(
            grid.unknown,
            grid.resolution,
            include_grid_exterior=True,
        ),
        min_clearance,
        node_ids=graph.nodes,
    )


def test_graph_builder_defaults_match_paper_geometry():
    """默认几何净空和连边半径应与论文参数一致"""
    config = GraphBuilderConfig()

    assert config.node_sample_count == 1000
    assert config.max_free_radius == 4.0
    assert config.min_obstacle_clearance == 0.5
    assert config.edge_radius == 8.0


@pytest.mark.parametrize(
    "values",
    [
        {"node_sample_count": -1},
        {"max_free_radius": 0.0},
        {"min_obstacle_clearance": -0.1},
        {"edge_radius": 0.0},
    ],
)
def test_graph_builder_rejects_invalid_topology_parameters(values):
    """节点密度和局部 pair 参数必须保持有效"""
    with pytest.raises(ValueError):
        GraphBuilderConfig(**values)


def test_unknown_distance_field_treats_grid_exterior_as_unknown():
    """局部地图没有 unknown cell 时, 地图外部仍提供有限未知距离"""
    unknown = np.zeros((5, 5), dtype=bool)

    distances = distance_to_mask(
        unknown,
        resolution=1.0,
        include_grid_exterior=True,
    )

    assert np.isfinite(distances).all()
    assert distances[0, 0] == 1.0
    assert distances[2, 2] == 3.0


def test_distance_field_uses_exact_euclidean_distance():
    """距离场应返回精确欧氏距离, 不能使用 8 连通近似"""
    mask = np.zeros((4, 4), dtype=bool)
    mask[0, 0] = True

    distances = distance_to_mask(mask, resolution=0.5)

    assert distances.dtype == np.float32
    assert distances[1, 2] == pytest.approx(math.sqrt(5.0) * 0.5)


def test_graph_builder_replaces_nonfinite_explored_radius():
    """历史 inf explored radius 必须在下一帧恢复为局部有限覆盖"""
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
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            node_sample_count=0,
            min_obstacle_clearance=0.0,
            edge_radius=10.0,
        )
    )
    node = builder.graph.create_node(position=(2.5, 2.5, 0.0), stamp_seconds=1.0)
    node.explored_radius = math.inf

    result = builder.update(grid, robot_position=(2.5, 2.5, 0.0), stamp_seconds=2.0)

    assert math.isfinite(result.graph.nodes[node.node_id].explored_radius)
    assert result.graph.nodes[node.node_id].explored_radius == 3.0


def test_node_sampling_uses_free_radius_coverage():
    """后采样节点不能落入已有节点的 free radius"""
    free = np.ones((12, 12), dtype=bool)
    unknown = np.zeros((12, 12), dtype=bool)
    unknown[:, 0] = True
    free[:, 0] = False
    grid = ClassifiedGrid(
        width=12,
        height=12,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=np.zeros((12, 12), dtype=bool),
        unknown=unknown,
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            node_sample_count=1000,
            random_seed=3,
            max_free_radius=4.0,
            min_obstacle_clearance=0.0,
            edge_radius=10.0,
        )
    )

    result = builder.update(grid, robot_position=(5.5, 5.5, 0.0), stamp_seconds=2.0)
    nodes = sorted(result.graph.nodes.values(), key=lambda node: node.node_id)

    assert len(nodes) > 2
    for node_index, node in enumerate(nodes):
        for later_node in nodes[node_index + 1:]:
            assert later_node.distance_xy(node.position) > node.free_radius
    assert any(node.free_radius < 2.0 for node in nodes)
    assert any(node.free_radius >= 4.0 for node in nodes)


def test_node_sampling_random_seed_is_reproducible():
    """相同输入和 random seed 必须生成相同节点序列"""
    def make_grid():
        return ClassifiedGrid(
            width=10,
            height=10,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            frame_id="map",
            free=np.ones((10, 10), dtype=bool),
            obstacle=np.zeros((10, 10), dtype=bool),
            unknown=np.zeros((10, 10), dtype=bool),
        )

    config = GraphBuilderConfig(
        node_sample_count=100,
        random_seed=11,
        min_obstacle_clearance=0.0,
    )
    first = SparseGraphBuilder(config).update(
        make_grid(),
        robot_position=(5.5, 5.5, 0.0),
        stamp_seconds=1.0,
    )
    second = SparseGraphBuilder(config).update(
        make_grid(),
        robot_position=(5.5, 5.5, 0.0),
        stamp_seconds=1.0,
    )

    assert [
        node.position
        for node in first.graph.nodes.values()
    ] == [
        node.position
        for node in second.graph.nodes.values()
    ]


def test_frontier_candidate_spacing_reduces_candidate_count():
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
            node_sample_count=100,
            min_obstacle_clearance=0.0,
            edge_radius=10.0,
            frontier_border_margin=0.0,
            frontier_candidate_spacing=3.0,
        )
    )

    raw_frontiers = builder.frontier_detector.detect_frontier_cells(grid)
    selected_frontiers = builder.frontier_detector._select_frontier_candidates(
        grid,
        raw_frontiers,
    )

    assert 0 < len(selected_frontiers) < len(raw_frontiers)


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
    edge_builder = EdgeBuilder(edge_radius=10.0)

    loose_delta = _build_edge_delta(edge_builder, graph, grid, min_clearance=0.5)
    strict_delta = _build_edge_delta(edge_builder, graph, grid, min_clearance=1.1)

    assert len(loose_delta.edges_to_add) == 1
    assert strict_delta.edges_to_add == []


def test_edge_builder_connects_all_safe_radius_pairs():
    """半径内全部安全无向 pair 都必须连边"""
    grid = ClassifiedGrid(
        width=8,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((3, 8), dtype=bool),
        obstacle=np.zeros((3, 8), dtype=bool),
        unknown=np.zeros((3, 8), dtype=bool),
    )
    graph = GraphState()
    first = graph.create_node((1.5, 1.5, 0.0), stamp_seconds=1.0)
    second = graph.create_node((3.5, 1.5, 0.0), stamp_seconds=1.0)
    third = graph.create_node((4.5, 1.5, 0.0), stamp_seconds=1.0)
    edge_builder = EdgeBuilder(edge_radius=3.0)

    delta = _build_edge_delta(edge_builder, graph, grid)

    assert {
        tuple(sorted((edge.from_id, edge.to_id)))
        for edge in delta.edges_to_add
    } == {
        (first.node_id, second.node_id),
        (first.node_id, third.node_id),
        (second.node_id, third.node_id),
    }


def test_edge_builder_does_not_truncate_dense_radius_pairs():
    """密集局部图不能再按候选数或邻居数截断"""
    grid = ClassifiedGrid(
        width=60,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((3, 60), dtype=bool),
        obstacle=np.zeros((3, 60), dtype=bool),
        unknown=np.zeros((3, 60), dtype=bool),
    )
    graph = GraphState()
    nodes = [
        graph.create_node((index + 0.5, 1.5, 0.0), stamp_seconds=1.0)
        for index in range(10)
    ]
    edge_builder = EdgeBuilder(edge_radius=50.0)

    delta = _build_edge_delta(edge_builder, graph, grid)

    expected_pair_count = len(nodes) * (len(nodes) - 1) // 2
    assert len(delta.edges_to_add) == expected_pair_count
    assert edge_builder.last_stats.candidate_pair_count == expected_pair_count
    assert edge_builder.last_stats.clearance_check_count == expected_pair_count


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
    edge_builder = EdgeBuilder(edge_radius=10.0)

    delta = _build_edge_delta(edge_builder, graph, grid, min_clearance=0.5)

    assert delta.edges_to_add == []
    assert delta.edge_keys_to_remove == set()
    assert edge_builder.last_stats.kept_edge_count == 1


def test_existing_local_edge_over_radius_is_removed():
    """两端都可见时, 超出连接半径的旧边必须删除"""
    grid = ClassifiedGrid(
        width=8,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((3, 8), dtype=bool),
        obstacle=np.zeros((3, 8), dtype=bool),
        unknown=np.zeros((3, 8), dtype=bool),
    )
    graph = GraphState()
    first = graph.create_node((1.5, 1.5, 0.0), stamp_seconds=1.0)
    second = graph.create_node((5.5, 1.5, 0.0), stamp_seconds=1.0)
    edge_key = (first.node_id, second.node_id)
    graph.set_edges([InternalEdge(*edge_key, cost=4.0)])

    delta = _build_edge_delta(EdgeBuilder(edge_radius=3.0), graph, grid)

    assert delta.edge_keys_to_remove == {edge_key}


def test_edge_builder_only_rebuilds_edges_inside_current_grid():
    """当前窗口只生成局部新边, 窗口外历史边仍被保留"""
    free = np.ones((3, 3), dtype=bool)
    obstacle = np.zeros((3, 3), dtype=bool)
    unknown = np.zeros((3, 3), dtype=bool)
    grid = ClassifiedGrid(
        width=3,
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
    outside_a = graph.create_node(position=(-2.5, 1.5, 0.0), stamp_seconds=1.0)
    outside_b = graph.create_node(position=(-1.5, 1.5, 0.0), stamp_seconds=1.0)
    inside_a = graph.create_node(position=(0.5, 1.5, 0.0), stamp_seconds=1.0)
    inside_b = graph.create_node(position=(1.5, 1.5, 0.0), stamp_seconds=1.0)
    graph.set_edges(
        [InternalEdge(from_id=outside_a.node_id, to_id=outside_b.node_id, cost=1.0)]
    )
    edge_builder = EdgeBuilder(edge_radius=2.0)

    delta = _build_edge_delta(edge_builder, graph, grid)
    graph.apply_edge_delta(delta.edge_keys_to_remove, delta.edges_to_add)

    assert (inside_a.node_id, inside_b.node_id) in graph.edges
    assert (outside_a.node_id, outside_b.node_id) in graph.edges


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
            node_sample_count=0,
            min_obstacle_clearance=0.0,
        )
    )
    node = builder.graph.create_node(position=(1.5, 1.5, 0.0), stamp_seconds=1.0)

    result = builder.update(grid, robot_position=(2.5, 2.5, 8.0), stamp_seconds=2.0)

    assert result.graph.nodes[node.node_id].position[2] == 2.0


def test_edge_with_outside_endpoint_is_not_changed_by_local_rebuild():
    """端点不都在窗口内的历史边保持不变"""
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
    edge_builder = EdgeBuilder(edge_radius=20.0)

    delta = _build_edge_delta(edge_builder, graph, grid)

    assert delta.edge_keys_to_remove == set()
    assert (0, 1) in graph.edges


def test_graph_builder_does_not_create_node_in_unknown_robot_cell():
    """脚下仍为 unknown 时不能虚构机器人节点和边"""
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
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    result = builder.update(grid, robot_position=(2.5, 2.5, 0.0), stamp_seconds=1.0)

    assert result.graph.current_node_id is None
    assert result.graph.nodes


def test_graph_builder_repairs_robot_blind_zone_from_nearby_ground():
    """验证脚下 unknown 只用周边地面修补且不覆盖障碍物"""
    free = np.ones((9, 9), dtype=bool)
    obstacle = np.zeros((9, 9), dtype=bool)
    unknown = np.zeros((9, 9), dtype=bool)
    elevation = np.full((9, 9), 0.25, dtype=float)
    free[3:6, 3:6] = False
    unknown[3:6, 3:6] = True
    elevation[3:6, 3:6] = np.nan
    obstacle[4, 5] = True
    unknown[4, 5] = False
    grid = ClassifiedGrid(
        width=9,
        height=9,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.8,
            robot_blind_zone_elevation_search_radius=2.0,
            min_obstacle_clearance=0.0,
        )
    )

    result = builder.update(grid, robot_position=(2.25, 2.25, 0.8), stamp_seconds=1.0)

    center = result.classified_grid.world_to_grid(2.25, 2.25)
    assert center is not None
    assert result.classified_grid.is_free_index(center[0], center[1])
    assert result.classified_grid.elevation_at_index(center[0], center[1]) == 0.25
    assert result.classified_grid.is_obstacle_index(5, 4)
    assert result.classified_grid.stats["robot_blind_zone_filled"] > 0
    current = result.graph.nodes[result.graph.current_node_id]
    assert result.classified_grid.is_world_collision_free(
        (2.25, 2.25),
        current.position[:2],
    )


def test_default_blind_zone_repairs_ground_out_to_1_2_metres():
    """验证默认盲区修补覆盖配置范围且不向外扩张"""
    resolution = 0.2
    size = 21
    center = 10
    offset_y, offset_x = np.ogrid[-center:size - center, -center:size - center]
    distance = np.hypot(offset_x * resolution, offset_y * resolution)
    unknown = distance <= 1.2
    free = ~unknown
    elevation = np.zeros((size, size), dtype=float)
    elevation[unknown] = np.nan
    grid = ClassifiedGrid(
        width=size,
        height=size,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=np.zeros((size, size), dtype=bool),
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    robot_xy = ((center + 0.5) * resolution, (center + 0.5) * resolution)
    builder = SparseGraphBuilder(GraphBuilderConfig(min_obstacle_clearance=0.0))

    result = builder.update(
        grid,
        robot_position=(robot_xy[0], robot_xy[1], 0.22),
        stamp_seconds=1.0,
    )

    assert result.classified_grid.is_free_index(center + 5, center)
    assert result.classified_grid.is_free_index(center, center + 5)
    assert result.classified_grid.stats["robot_blind_zone_status"] == "repaired"


def test_blind_zone_repair_only_runs_during_initialization():
    """初始化完成后不能在机器人新位置继续填充 unknown"""

    def make_grid() -> ClassifiedGrid:
        """构造两个相互分离的脚下 unknown 区域"""
        free = np.ones((9, 17), dtype=bool)
        unknown = np.zeros((9, 17), dtype=bool)
        elevation = np.zeros((9, 17), dtype=float)
        unknown[3:6, 2:5] = True
        unknown[3:6, 12:15] = True
        free[unknown] = False
        elevation[unknown] = np.nan
        return ClassifiedGrid(
            width=17,
            height=9,
            resolution=0.5,
            origin_x=0.0,
            origin_y=0.0,
            frame_id="map",
            free=free,
            obstacle=np.zeros((9, 17), dtype=bool),
            unknown=unknown,
            elevation=elevation,
            stats={},
        )

    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.8,
            robot_blind_zone_elevation_search_radius=2.0,
            min_obstacle_clearance=0.0,
        )
    )
    first = builder.update(
        make_grid(),
        robot_position=(1.75, 2.25, 0.22),
        stamp_seconds=1.0,
    )
    second = builder.update(
        make_grid(),
        robot_position=(6.75, 2.25, 0.22),
        stamp_seconds=2.0,
    )

    first_center = first.classified_grid.world_to_grid(1.75, 2.25)
    second_center = second.classified_grid.world_to_grid(6.75, 2.25)
    assert first_center is not None
    assert second_center is not None
    assert first.classified_grid.is_free_index(*first_center)
    assert second.classified_grid.is_unknown_index(*second_center)
    assert second.classified_grid.stats["robot_blind_zone_filled"] == 0
    assert (
        second.classified_grid.stats["robot_blind_zone_status"]
        == "initial_only_complete"
    )


def test_artificial_initial_free_does_not_count_as_sensor_new_free():
    """初始化人工 free 不得触发旧节点局部重连"""
    free = np.ones((9, 9), dtype=bool)
    unknown = np.zeros((9, 9), dtype=bool)
    elevation = np.zeros((9, 9), dtype=float)
    unknown[3:6, 3:6] = True
    free[unknown] = False
    elevation[unknown] = np.nan
    grid = ClassifiedGrid(
        width=9,
        height=9,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=np.zeros((9, 9), dtype=bool),
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.8,
            robot_blind_zone_elevation_search_radius=2.0,
            min_obstacle_clearance=0.0,
        )
    )

    result = builder.update(
        grid,
        robot_position=(2.25, 2.25, 0.22),
        stamp_seconds=1.0,
    )

    artificial_count = result.classified_grid.stats[
        "robot_blind_zone_artificial_free"
    ]
    assert artificial_count > 0
    assert result.stats.newly_free_cell_count == (
        int(np.count_nonzero(result.classified_grid.free)) - artificial_count
    )


def test_blind_zone_preserves_slope_step_pit_and_obstacle_surfaces():
    """验证扩大盲区只修补 unknown 且不抹平已有地形"""
    resolution = 0.2
    size = 21
    center = 10
    x_coordinates = (np.arange(size, dtype=float) + 0.5) * resolution
    elevation = np.broadcast_to(0.05 * x_coordinates, (size, size)).copy()
    free = np.ones((size, size), dtype=bool)
    obstacle = np.zeros((size, size), dtype=bool)
    unknown = np.zeros((size, size), dtype=bool)
    unknown[center - 2:center + 3, center - 2:center + 3] = True
    free[unknown] = False
    elevation[unknown] = np.nan

    # 已观测的台阶、坑和障碍必须保留原始分类与高程
    elevation[center, center + 5] = 0.45
    elevation[center + 5, center] = -0.35
    obstacle[center - 5, center] = True
    free[center - 5, center] = False
    grid = ClassifiedGrid(
        width=size,
        height=size,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    robot_xy = ((center + 0.5) * resolution, (center + 0.5) * resolution)
    expected_ground = 0.05 * robot_xy[0]
    builder = SparseGraphBuilder(GraphBuilderConfig(min_obstacle_clearance=0.0))

    result = builder.update(
        grid,
        robot_position=(robot_xy[0], robot_xy[1], expected_ground + 0.22),
        stamp_seconds=1.0,
    )

    assert result.classified_grid.is_free_index(center, center)
    assert abs(result.classified_grid.elevation_at_index(center, center) - expected_ground) < 0.03
    assert result.classified_grid.elevation_at_index(center + 5, center) == 0.45
    assert result.classified_grid.is_unknown_index(center, center + 5)
    assert np.isnan(result.classified_grid.elevation[center + 5, center])
    assert result.classified_grid.is_obstacle_index(center, center - 5)


def test_graph_builder_uses_safe_ordinary_current_node_on_known_ground():
    """已知地面使用安全普通节点作为规划起点"""
    grid = ClassifiedGrid(
        width=7,
        height=7,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((7, 7), dtype=bool),
        obstacle=np.zeros((7, 7), dtype=bool),
        unknown=np.zeros((7, 7), dtype=bool),
        elevation=np.zeros((7, 7), dtype=float),
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(min_obstacle_clearance=0.0, edge_radius=3.0)
    )

    result = builder.update(grid, robot_position=(3.2, 3.4, 0.8), stamp_seconds=1.0)

    current = result.graph.nodes[result.graph.current_node_id]
    assert current.distance_xy((3.2, 3.4, 0.0)) <= builder.config.edge_radius
    assert grid.is_world_collision_free((3.2, 3.4), current.position[:2])


def test_graph_builder_rejects_implausible_high_surface_near_blind_zone():
    """验证墙面或高台高程不会被当成脚下地面"""
    free = np.ones((15, 15), dtype=bool)
    obstacle = np.zeros((15, 15), dtype=bool)
    unknown = np.zeros((15, 15), dtype=bool)
    elevation = np.full((15, 15), 1.3, dtype=float)
    free[5:10, 5:10] = False
    unknown[5:10, 5:10] = True
    elevation[5:10, 5:10] = np.nan
    grid = ClassifiedGrid(
        width=15,
        height=15,
        resolution=0.2,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.4,
            robot_blind_zone_elevation_search_radius=3.0,
            robot_ground_height_offset=0.22,
            robot_ground_elevation_tolerance=0.5,
            min_obstacle_clearance=0.0,
        )
    )

    result = builder.update(grid, robot_position=(1.5, 1.5, 0.22), stamp_seconds=1.0)

    center = result.classified_grid.world_to_grid(1.5, 1.5)
    assert center is not None
    assert result.classified_grid.elevation_at_index(center[0], center[1]) == 0.0
    assert result.classified_grid.stats["robot_blind_zone_ground_source"] == "odom"


def test_graph_builder_samples_outer_free_component_after_blind_zone_repair():
    """验证脚下修补小岛不会阻断外围 free 分量采样"""
    free = np.ones((31, 31), dtype=bool)
    obstacle = np.zeros((31, 31), dtype=bool)
    unknown = np.zeros((31, 31), dtype=bool)
    elevation = np.zeros((31, 31), dtype=float)
    free[9:22, 9:22] = False
    unknown[9:22, 9:22] = True
    elevation[9:22, 9:22] = np.nan
    grid = ClassifiedGrid(
        width=31,
        height=31,
        resolution=0.2,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=obstacle,
        unknown=unknown,
        elevation=elevation,
        stats={},
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.5,
            robot_blind_zone_elevation_search_radius=3.0,
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    result = builder.update(grid, robot_position=(3.1, 3.1, 0.22), stamp_seconds=1.0)

    current_id = result.graph.current_node_id
    assert current_id is not None
    assert len(result.graph.nodes) > 2
    assert not any(
        current_id in (edge.from_id, edge.to_id)
        and not grid.is_world_collision_free(
            result.graph.nodes[edge.from_id].position[:2],
            result.graph.nodes[edge.to_id].position[:2],
        )
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
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=4.0,
        )
    )

    result = builder.update(grid, robot_position=(5.5, 5.5, 0.0), stamp_seconds=1.0)

    assert len(result.graph.nodes) > 1
    assert len(result.graph.edges) > 0


def _make_three_component_grid() -> ClassifiedGrid:
    """构造状态稳定但彼此被 unknown 分隔的三块 free 地面"""
    free_columns = np.asarray(
        [1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        dtype=bool,
    )
    free = np.tile(free_columns, (9, 1))
    unknown = ~free
    return ClassifiedGrid(
        width=21,
        height=9,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=free,
        obstacle=np.zeros((9, 21), dtype=bool),
        unknown=unknown,
        elevation=np.zeros((9, 21), dtype=float),
        stats={},
    )


def test_reachable_known_free_component_gets_nodes_and_edges_on_later_frame():
    """已知 free 区域后来可达时仍需补齐节点和局部边"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.0,
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=4.0,
            frontier_border_margin=0.0,
        )
    )

    first = builder.update(
        _make_three_component_grid(),
        robot_position=(10.5, 4.5, 0.0),
        stamp_seconds=1.0,
    )
    assert not any(
        node.position[0] >= 16.0
        for node in first.graph.nodes.values()
    )

    second = builder.update(
        _make_three_component_grid(),
        robot_position=(18.5, 4.5, 0.0),
        stamp_seconds=2.0,
    )

    right_node_ids = {
        node.node_id
        for node in second.graph.nodes.values()
        if node.position[0] >= 16.0
    }
    assert second.stats.newly_free_cell_count == 0
    assert len(right_node_ids) > 1
    assert second.graph.current_node_id in right_node_ids
    assert any(
        edge.from_id in right_node_ids and edge.to_id in right_node_ids
        for edge in second.graph.edges.values()
    )


def test_new_nodes_refresh_frontier_in_unchanged_free_component():
    """新节点出现时需要重新分配附近未变化的 Frontier"""
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.0,
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=4.0,
            frontier_assign_radius=3.0,
            frontier_min_points=1,
            frontier_min_span=0.0,
            frontier_border_margin=0.0,
            frontier_visited_corridor_radius=0.0,
        )
    )
    builder.update(
        _make_three_component_grid(),
        robot_position=(10.5, 4.5, 0.0),
        stamp_seconds=1.0,
    )

    result = builder.update(
        _make_three_component_grid(),
        robot_position=(18.5, 4.5, 0.0),
        stamp_seconds=2.0,
    )

    assert result.stats.dirty_cell_count == 0
    assert result.stats.frontier_candidate_count > 0
    assert any(
        node.is_frontier and node.position[0] >= 16.0
        for node in result.graph.nodes.values()
    )


def test_all_nearby_free_components_receive_internal_nodes_and_edges():
    """脚下周围多个 free 分量都应补点, 但不能跨 unknown 连边"""
    grid = _make_three_component_grid()
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            robot_blind_zone_radius=0.0,
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=6.0,
            frontier_border_margin=0.0,
        )
    )

    result = builder.update(
        grid,
        robot_position=(10.5, 4.5, 0.0),
        stamp_seconds=1.0,
    )

    left_node_ids = {
        node.node_id
        for node in result.graph.nodes.values()
        if node.position[0] < 5.0
    }
    right_node_ids = {
        node.node_id
        for node in result.graph.nodes.values()
        if node.position[0] >= 16.0
    }
    assert len(left_node_ids) > 1
    assert len(right_node_ids) > 1
    assert any(
        edge.from_id in left_node_ids and edge.to_id in left_node_ids
        for edge in result.graph.edges.values()
    )
    assert any(
        edge.from_id in right_node_ids and edge.to_id in right_node_ids
        for edge in result.graph.edges.values()
    )
    assert all(
        grid.is_world_collision_free(
            result.graph.nodes[edge.from_id].position[:2],
            result.graph.nodes[edge.to_id].position[:2],
        )
        for edge in result.graph.edges.values()
    )


def test_unknown_ground_does_not_create_anchor_breadcrumbs():
    """连续脚下缺图时不创建 anchor 或虚假 breadcrumb"""
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
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    builder.update(grid, robot_position=(1.5, 1.5, 0.0), stamp_seconds=1.0)
    result = builder.update(grid, robot_position=(3.0, 1.5, 0.0), stamp_seconds=2.0)

    assert result.graph.current_node_id is None
    assert result.graph.nodes == {}
    assert result.graph.edges == {}


def test_robot_motion_does_not_create_moving_anchor_breadcrumbs():
    """已知 free 地面移动时只切换普通节点, 不持续增加特殊节点"""
    grid = ClassifiedGrid(
        width=20,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        free=np.ones((5, 20), dtype=bool),
        obstacle=np.zeros((5, 20), dtype=bool),
        unknown=np.zeros((5, 20), dtype=bool),
        elevation=np.zeros((5, 20), dtype=float),
    )
    builder = SparseGraphBuilder(
        GraphBuilderConfig(
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )

    first = builder.update(grid, robot_position=(1.5, 2.5, 0.0), stamp_seconds=1.0)
    initial_node_count = len(first.graph.nodes)
    for index, robot_x in enumerate((3.5, 5.5, 7.5, 9.5), start=2):
        result = builder.update(
            grid,
            robot_position=(robot_x, 2.5, 0.0),
            stamp_seconds=float(index),
        )

    assert len(result.graph.nodes) == initial_node_count
    assert result.stats.edge_add_count == 0
    assert result.stats.edge_remove_count == 0


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
            node_sample_count=1000,
            min_obstacle_clearance=0.0,
            edge_radius=3.0,
        )
    )
    historical = builder.graph.create_node(
        position=(9.5, 2.5, 0.0),
        stamp_seconds=0.0,
    )

    result = builder.update(grid, robot_position=(0.5, 0.5, 0.0), stamp_seconds=1.0)

    assert historical.node_id in result.graph.nodes
    assert any(node.position[0] < 4.0 for node in result.graph.nodes.values())
    assert any(node.position[0] >= 8.0 for node in result.graph.nodes.values())
