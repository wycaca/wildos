from graphnav_msgs.msg import (
    KeyValue,
    NavigationGraph,
    Node,
    NodeTraversabilityProperties,
)

from visual_navigation.stable_frontier_selector import (
    StableFrontierSelector,
    StableFrontierSelectorConfig,
)


def make_graph(nodes):
    graph = NavigationGraph()
    graph.header.frame_id = "map"
    graph.trav_classes = ["default"]
    graph.current_node_idx = 0
    graph.nodes = nodes
    return graph


def make_frontier(uuid_value, x, y, score):
    node = Node()
    node.uuid.id = [uuid_value] * 16
    node.pose.position.x = float(x)
    node.pose.position.y = float(y)
    node.pose.orientation.w = 1.0
    trav = NodeTraversabilityProperties()
    trav.is_frontier = True
    node.trav_properties = [trav]
    prop = KeyValue()
    prop.key = "frontier_scores"
    prop.value = [float(score)]
    node.properties = [prop]
    return node


def make_selector(**overrides):
    config = StableFrontierSelectorConfig(
        min_dwell_sec=10.0,
        switch_min_score_margin=0.1,
        distance_weight=0.0,
        switch_penalty=0.0,
        **overrides,
    )
    return StableFrontierSelector(config)


def test_selector_keeps_current_frontier_during_dwell():
    selector = make_selector()
    graph = make_graph([
        make_frontier(1, 5.0, 0.0, 0.8),
        make_frontier(2, 2.0, 0.0, 0.4),
    ])

    first = selector.select(graph, (0.0, 0.0), 0.0, graph.header.stamp)
    assert first.uuid == "001" * 16

    updated_graph = make_graph([
        make_frontier(1, 5.0, 0.0, 0.8),
        make_frontier(2, 2.0, 0.0, 1.0),
    ])
    second = selector.select(updated_graph, (0.0, 0.0), 5.0, graph.header.stamp)

    assert second.uuid == first.uuid


def test_selector_switches_after_dwell_when_candidate_is_better():
    selector = make_selector()
    graph = make_graph([
        make_frontier(1, 5.0, 0.0, 0.8),
        make_frontier(2, 2.0, 0.0, 0.4),
    ])

    selector.select(graph, (0.0, 0.0), 0.0, graph.header.stamp)
    updated_graph = make_graph([
        make_frontier(1, 5.0, 0.0, 0.8),
        make_frontier(2, 2.0, 0.0, 1.0),
    ])
    selected = selector.select(
        updated_graph,
        (0.0, 0.0),
        12.0,
        graph.header.stamp,
    )

    assert selected.uuid == "002" * 16


def test_selector_inherits_nearby_frontier_when_uuid_changes():
    selector = make_selector(same_position_radius=0.5)
    graph = make_graph([make_frontier(1, 5.0, 0.0, 0.8)])

    first = selector.select(graph, (0.0, 0.0), 0.0, graph.header.stamp)
    shifted_graph = make_graph([make_frontier(3, 5.2, 0.1, 0.8)])
    selected = selector.select(
        shifted_graph,
        (0.0, 0.0),
        5.0,
        graph.header.stamp,
    )

    assert first.uuid == "001" * 16
    assert selected.uuid == "003" * 16
    assert selector.selected.uuid == "003" * 16


def test_selector_prefers_frontier_in_search_heading():
    selector = make_selector(
        min_forward_dot=0.0,
        forward_weight=0.8,
    )
    graph = make_graph([
        make_frontier(1, -2.0, 0.0, 1.0),
        make_frontier(2, 4.0, 0.0, 0.2),
    ])

    selected = selector.select(
        graph,
        (0.0, 0.0),
        0.0,
        graph.header.stamp,
        heading_yaw=0.0,
    )

    assert selected.uuid == "002" * 16
    assert selected.heading_alignment > 0.0


def test_selector_falls_back_when_no_frontier_is_in_front():
    selector = make_selector(
        min_forward_dot=0.0,
        forward_fallback_to_any=True,
    )
    graph = make_graph([
        make_frontier(1, -2.0, 0.0, 0.6),
        make_frontier(2, -4.0, 1.0, 0.8),
    ])

    selected = selector.select(
        graph,
        (0.0, 0.0),
        0.0,
        graph.header.stamp,
        heading_yaw=0.0,
    )

    assert selected is not None
    assert selected.heading_alignment < 0.0
