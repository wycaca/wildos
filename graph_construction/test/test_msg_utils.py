from std_msgs.msg import Header

from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.msg_utils import GraphMessageCache, graph_to_msg


def test_graph_message_cache_reuses_stable_elements():
    """稳定节点和边复用消息对象, 变化节点单独重建"""
    graph = GraphState()
    first = graph.create_node((0.0, 0.0, 0.0), stamp_seconds=1.0)
    second = graph.create_node((1.0, 0.0, 0.0), stamp_seconds=1.0)
    graph.set_edges([InternalEdge(first.node_id, second.node_id, 1.0)])
    graph.current_node_id = first.node_id
    cache = GraphMessageCache()

    initial = graph_to_msg(graph, Header(), "default", cache=cache)
    repeated = graph_to_msg(graph, Header(), "default", cache=cache)

    assert repeated.nodes[0] is initial.nodes[0]
    assert repeated.nodes[1] is initial.nodes[1]
    assert repeated.edges[0] is initial.edges[0]

    graph.move_node(first.node_id, (0.0, 0.0, 0.5))
    changed = graph_to_msg(graph, Header(), "default", cache=cache)

    assert changed.nodes[0] is not repeated.nodes[0]
    assert changed.nodes[1] is repeated.nodes[1]
    assert changed.edges[0] is repeated.edges[0]
    assert changed.nodes[0].pose.position.z == 0.5


def test_graph_message_cache_keeps_edges_when_nodes_append():
    """追加节点不改变旧索引时继续复用历史边消息"""
    graph = GraphState()
    first = graph.create_node((0.0, 0.0, 0.0), stamp_seconds=1.0)
    second = graph.create_node((1.0, 0.0, 0.0), stamp_seconds=1.0)
    graph.set_edges([InternalEdge(first.node_id, second.node_id, 1.0)])
    cache = GraphMessageCache()

    initial = graph_to_msg(graph, Header(), "default", cache=cache)
    graph.create_node((2.0, 0.0, 0.0), stamp_seconds=2.0)
    appended = graph_to_msg(graph, Header(), "default", cache=cache)

    assert appended.edges[0] is initial.edges[0]


def test_graph_message_generation_does_not_mutate_internal_state():
    """完整消息转换只能读取内部图, 不能修改图状态和索引"""
    graph = GraphState()
    first = graph.create_node((0.0, 0.0, 0.0), stamp_seconds=1.0)
    second = graph.create_node((1.0, 0.0, 0.0), stamp_seconds=1.0)
    first.free_radius = 0.8
    second.frontier_points = [(1.5, 0.0, 0.0)]
    second.is_frontier = True
    graph.set_edges([InternalEdge(first.node_id, second.node_id, 1.0)])
    graph.current_node_id = first.node_id
    graph.update_robot_position((0.2, 0.0, 0.3), (0.2, 0.0, 0.0))
    graph.append_trajectory_point((0.2, 0.0, 0.0), 0.1)
    before = _graph_state_snapshot(graph)

    graph_to_msg(graph, Header(), "default", cache=GraphMessageCache())

    assert _graph_state_snapshot(graph) == before


def _graph_state_snapshot(graph):
    return {
        "nodes": {
            node_id: (
                node.position,
                node.free_radius,
                node.explored_radius,
                tuple(node.frontier_points),
                node.is_frontier,
                node.last_seen_time,
            )
            for node_id, node in graph.nodes.items()
        },
        "edges": {
            edge_key: (edge.from_id, edge.to_id, edge.cost)
            for edge_key, edge in graph.edges.items()
        },
        "adjacency": {
            node_id: frozenset(edge_keys)
            for node_id, edge_keys in graph.adjacency.items()
        },
        "current": graph.current_node_id,
        "odom": graph.latest_robot_odom_position,
        "ground": graph.latest_robot_position,
        "trajectory": tuple(graph.trajectory_points),
        "next_node_id": graph.next_node_id,
    }
