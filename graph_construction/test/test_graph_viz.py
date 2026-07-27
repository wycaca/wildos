from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.viz import GraphVisualizer


def _graph_with_radius_node() -> GraphState:
    graph = GraphState()
    node = graph.create_node((1.0, 2.0, 0.0), stamp_seconds=1.0)
    node.free_radius = 2.0
    node.explored_radius = 3.0
    graph.current_node_id = node.node_id
    return graph


def test_graph_viz_hides_radius_markers_by_default():
    markers = GraphVisualizer().build_markers(
        _graph_with_radius_node(),
        Header(frame_id="odom", stamp=Time(sec=1)),
    )

    namespaces = {marker.ns for marker in markers.markers}
    assert "free_radius" not in namespaces
    assert "explored_radius" not in namespaces
    assert "grid_footprint" not in namespaces
    assert "current_node" not in namespaces
    assert "edges" in namespaces
    edge_marker = next(marker for marker in markers.markers if marker.ns == "edges")
    assert edge_marker.scale.x == 0.03


def test_graph_viz_can_enable_radius_markers_for_debugging():
    markers = GraphVisualizer(show_radius_markers=True).build_markers(
        _graph_with_radius_node(),
        Header(frame_id="odom", stamp=Time(sec=1)),
    )

    namespaces = {marker.ns for marker in markers.markers}
    assert "free_radius" in namespaces
    assert "explored_radius" in namespaces


def test_graph_viz_keeps_historical_nodes_visible():
    graph = _graph_with_radius_node()
    markers = GraphVisualizer().build_markers(
        graph,
        Header(frame_id="odom", stamp=Time(sec=2)),
    )

    memory_marker = next(
        marker for marker in markers.markers if marker.ns == "memory_free_nodes"
    )
    assert len(memory_marker.points) == 1
    assert memory_marker.scale.x == 0.25
    assert memory_marker.color.g == 0.8
    assert memory_marker.color.a == 0.9


def test_graph_viz_uses_spanning_tree_edges_by_default():
    """默认只显示生成树, 避免完整半径图遮挡地面"""
    graph = GraphState()
    nodes = [
        graph.create_node((float(index), 0.0, 0.0), stamp_seconds=1.0)
        for index in range(4)
    ]
    graph.current_node_id = nodes[0].node_id
    graph.set_edges(
        InternalEdge(nodes[first].node_id, nodes[second].node_id, 1.0)
        for first in range(4)
        for second in range(first + 1, 4)
    )

    markers = GraphVisualizer().build_markers(
        graph,
        Header(frame_id="odom", stamp=Time(sec=1)),
    )

    edge_marker = next(
        marker
        for marker in markers.markers
        if marker.ns == "edges"
    )
    assert len(edge_marker.points) == 2 * (len(nodes) - 1)


def test_graph_viz_can_show_full_edges_for_debugging():
    """调试开关打开后显示全部实际边"""
    graph = GraphState()
    nodes = [
        graph.create_node((float(index), 0.0, 0.0), stamp_seconds=1.0)
        for index in range(3)
    ]
    graph.set_edges(
        [
            InternalEdge(nodes[0].node_id, nodes[1].node_id, 1.0),
            InternalEdge(nodes[0].node_id, nodes[2].node_id, 2.0),
            InternalEdge(nodes[1].node_id, nodes[2].node_id, 1.0),
        ]
    )

    markers = GraphVisualizer(show_full_edges=True).build_markers(
        graph,
        Header(frame_id="odom", stamp=Time(sec=1)),
    )

    edge_marker = next(
        marker
        for marker in markers.markers
        if marker.ns == "edges"
    )
    assert len(edge_marker.points) == 2 * len(graph.edges)
