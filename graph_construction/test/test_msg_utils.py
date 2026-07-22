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
