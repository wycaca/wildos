from geometry_msgs.msg import Point
from graphnav_msgs.msg import (
    Edge,
    EdgeTraversability,
    KeyValue,
    NavigationGraph,
    Node,
    NodeTraversabilityProperties,
)
import pytest

from wildos_visualization.graph_visualizer import (
    GraphVisualizer,
    GraphVisualizerNode,
    ScoreRingVisualizer,
)


def _graph(node_count=4, complete=False):
    graph = NavigationGraph()
    graph.header.frame_id = "odom"
    graph.trav_classes = ["default"]
    graph.current_node_idx = 1
    for index in range(node_count):
        properties = NodeTraversabilityProperties(
            is_frontier=index == node_count - 1,
            free_radius=2.0,
            explored_radius=3.0,
        )
        if properties.is_frontier:
            properties.frontier_points = [Point(x=float(index), y=1.0)]
        node = Node(trav_properties=[properties])
        node.pose.position.x = float(index)
        graph.nodes.append(node)
    pairs = (
        (
            (first, second)
            for first in range(node_count)
            for second in range(first + 1, node_count)
        )
        if complete
        else ((index, index + 1) for index in range(node_count - 1))
    )
    graph.edges = [
        Edge(
            from_idx=first,
            to_idx=second,
            traversability=[EdgeTraversability(traversability_cost=1.0)],
        )
        for first, second in pairs
    ]
    return graph


def _marker(markers, namespace):
    return next(marker for marker in markers.markers if marker.ns == namespace)


def test_default_markers_preserve_public_graph_view():
    markers = GraphVisualizer().build_markers(
        _graph(),
        trajectory=((0.0, 0.0, 0.0),),
        odom_position=(1.0, 0.0, 0.0),
    )

    assert len(_marker(markers, "free_nodes").points) == 3
    assert len(_marker(markers, "frontier_nodes").points) == 1
    assert len(_marker(markers, "frontier_points").points) == 1
    assert len(_marker(markers, "trajectory").points) == 1
    assert _marker(markers, "robot_position").pose.position.x == 1.0
    assert _marker(markers, "robot_odom_position").pose.position.x == 1.0
    assert not {"free_radius", "explored_radius"} & {
        marker.ns for marker in markers.markers
    }


def test_default_edges_form_a_spanning_forest():
    graph = _graph(4, complete=True)

    markers = GraphVisualizer().build_markers(graph)

    assert len(_marker(markers, "edges").points) == 2 * (len(graph.nodes) - 1)


def test_debug_switches_show_full_edges_and_bounded_radius():
    graph = _graph(3, complete=True)
    graph.nodes[0].trav_properties[0].free_radius = 100.0

    markers = GraphVisualizer(
        show_full_edges=True,
        show_radius_markers=True,
    ).build_markers(graph)

    assert len(_marker(markers, "edges").points) == 2 * len(graph.edges)
    assert _marker(markers, "free_radius").scale.x == 40.0
    assert "explored_radius" in {marker.ns for marker in markers.markers}


def test_callback_skips_marker_build_without_rviz_subscriber():
    class Publisher:
        def get_subscription_count(self):
            return 0

        def publish(self, _message):
            raise AssertionError("must not publish")

    class Visualizer:
        def build_markers(self, *_args):
            raise AssertionError("must not build markers")

    node = GraphVisualizerNode.__new__(GraphVisualizerNode)
    node.publisher = Publisher()
    node.visualizer = Visualizer()
    node.trajectory = ()
    node.latest_odom_position = None

    node._on_graph(_graph())


def test_score_rings_rebuild_public_frontier_scores():
    graph = _graph(1)
    graph.nodes[0].properties = [
        KeyValue(key="frontier_scores", value=[0.0, 0.5, 1.0, 0.25])
    ]

    markers = ScoreRingVisualizer().build_markers(graph)

    assert markers.markers[0].action == markers.markers[0].DELETEALL
    rings = _marker(markers, "geofrontier_score_ring")
    assert len(rings.points) == 40
    assert len(rings.colors) == 40
    assert rings.colors[0].b == 1.0
    assert rings.colors[20].r == 1.0
    assert rings.colors[20].g == pytest.approx(0.15)


def test_score_ring_callback_skips_work_without_rviz_subscriber():
    class Publisher:
        def get_subscription_count(self):
            return 0

        def publish(self, _message):
            raise AssertionError("must not publish")

    class Visualizer:
        def build_markers(self, *_args):
            raise AssertionError("must not build markers")

    node = GraphVisualizerNode.__new__(GraphVisualizerNode)
    node.score_ring_publisher = Publisher()
    node.score_ring_visualizer = Visualizer()

    node._on_scored_graph(_graph())
