from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from graph_construction.graph_memory import GraphState
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
