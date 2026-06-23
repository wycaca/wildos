from __future__ import annotations

from typing import Dict, Tuple

from geometry_msgs.msg import Point
from graphnav_msgs.msg import (
    Edge,
    EdgeTraversability,
    NavigationGraph,
    Node,
    NodeTraversabilityProperties,
    UUID,
)
from std_msgs.msg import Header

from graph_construction.graph_memory import GraphState, InternalNode


def graph_to_msg(graph: GraphState, header: Header, trav_class: str) -> NavigationGraph:
    """将内部图记忆转换为公开的 NavigationGraph 消息

    graphnav_planner 使用数组下标作为节点索引
    因此这里必须先对内部 node_id 排序, 再建立 node_id 到数组 index 的映射
    Edge.from_idx 和 Edge.to_idx 必须使用转换后的数组 index, 不能直接使用内部 node_id
    """
    msg = NavigationGraph()
    msg.header = header
    msg.trav_classes = [trav_class]

    sorted_nodes = sorted(graph.nodes.values(), key=lambda node: node.node_id)
    id_to_index: Dict[int, int] = {
        node.node_id: index
        for index, node in enumerate(sorted_nodes)
    }

    msg.nodes = [_node_to_msg(node) for node in sorted_nodes]
    msg.edges = [
        _edge_to_msg(edge_key, edge.cost, id_to_index)
        for edge_key, edge in sorted(graph.edges.items())
        if edge.from_id in id_to_index and edge.to_id in id_to_index
    ]

    if graph.current_node_id in id_to_index:
        msg.current_node_idx = id_to_index[graph.current_node_id]
    else:
        msg.current_node_idx = 0

    return msg


def _node_to_msg(node: InternalNode) -> Node:
    """转换单个内部节点为 graphnav_msgs/Node"""
    msg = Node()
    msg.uuid = UUID()
    msg.uuid.id = list(node.uuid_bytes)
    msg.pose.position.x = float(node.position[0])
    msg.pose.position.y = float(node.position[1])
    msg.pose.position.z = float(node.position[2])
    msg.pose.orientation.w = 1.0

    trav = NodeTraversabilityProperties()
    trav.is_frontier = bool(node.is_frontier)
    trav.frontier_points = [_point_to_msg(point) for point in node.frontier_points]
    trav.explored_radius = float(node.explored_radius)
    trav.free_radius = float(node.free_radius)
    trav.properties = []

    # WildOS 约定 trav_properties 顺序必须和 NavigationGraph.trav_classes 对齐
    # 第一版只输出 default 一个类别, 所以数组长度固定为 1
    msg.trav_properties = [trav]
    msg.properties = []
    return msg


def _edge_to_msg(edge_key: Tuple[int, int], cost: float, id_to_index: Dict[int, int]) -> Edge:
    """转换内部无向边为 graphnav_msgs/Edge"""
    msg = Edge()
    msg.from_idx = id_to_index[edge_key[0]]
    msg.to_idx = id_to_index[edge_key[1]]

    traversability = EdgeTraversability()
    traversability.traversability_cost = float(cost)
    traversability.properties = []

    msg.traversability = [traversability]
    msg.properties = []
    return msg


def _point_to_msg(point: Tuple[float, float, float]) -> Point:
    """转换内部三维点为 geometry_msgs/Point"""
    msg = Point()
    msg.x = float(point[0])
    msg.y = float(point[1])
    msg.z = float(point[2])
    return msg
