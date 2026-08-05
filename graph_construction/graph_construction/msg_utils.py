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


class GraphMessageCache:
    """复用未变化节点和边的 ROS 消息对象"""

    def __init__(self) -> None:
        self._node_cache: Dict[int, tuple[tuple, Node]] = {}
        self._edge_cache: Dict[Tuple[int, int], tuple[tuple, Edge]] = {}

    def build(
        self,
        graph: GraphState,
        header: Header,
        trav_class: str,
    ) -> NavigationGraph:
        """只重新转换内容发生变化的图元素"""
        msg = NavigationGraph()
        msg.header = header
        msg.trav_classes = [trav_class]

        node_ids = tuple(sorted(graph.nodes))
        id_to_index = {
            node_id: index
            for index, node_id in enumerate(node_ids)
        }

        msg.nodes = [
            self._cached_node(graph.nodes[node_id])
            for node_id in node_ids
        ]
        msg.edges = [
            self._cached_edge(edge_key, edge.cost, id_to_index)
            for edge_key, edge in sorted(graph.edges.items())
            if edge.from_id in id_to_index and edge.to_id in id_to_index
        ]
        self._prune_stale_entries(set(node_ids), set(graph.edges))

        msg.current_node_idx = id_to_index.get(graph.current_node_id, 0)
        return msg

    def _prune_stale_entries(
        self,
        active_node_ids: set[int],
        active_edge_keys: set[Tuple[int, int]],
    ) -> None:
        """缓存明显膨胀时再清理, 避免每帧复制完整缓存字典"""
        if len(self._node_cache) > len(active_node_ids) + 64:
            self._node_cache = {
                node_id: cached
                for node_id, cached in self._node_cache.items()
                if node_id in active_node_ids
            }
        if len(self._edge_cache) > len(active_edge_keys) + 128:
            self._edge_cache = {
                edge_key: cached
                for edge_key, cached in self._edge_cache.items()
                if edge_key in active_edge_keys
            }

    def _cached_node(self, node: InternalNode) -> Node:
        signature = (
            node.uuid_bytes,
            node.position,
            node.free_radius,
            node.explored_radius,
            node.is_frontier,
            tuple(node.frontier_points),
        )
        cached = self._node_cache.get(node.node_id)
        if cached is None or cached[0] != signature:
            cached = (signature, _node_to_msg(node))
            self._node_cache[node.node_id] = cached
        return cached[1]

    def _cached_edge(
        self,
        edge_key: Tuple[int, int],
        cost: float,
        id_to_index: Dict[int, int],
    ) -> Edge:
        signature = (
            float(cost),
            id_to_index[edge_key[0]],
            id_to_index[edge_key[1]],
        )
        cached = self._edge_cache.get(edge_key)
        if cached is None or cached[0] != signature:
            cached = (signature, _edge_to_msg(edge_key, cost, id_to_index))
            self._edge_cache[edge_key] = cached
        return cached[1]


def graph_to_msg(
    graph: GraphState,
    header: Header,
    trav_class: str,
    cache: GraphMessageCache | None = None,
) -> NavigationGraph:
    """将内部图记忆转换为公开的 NavigationGraph 消息

    graphnav_planner 使用数组下标作为节点索引
    因此这里必须先对内部 node_id 排序, 再建立 node_id 到数组 index 的映射
    Edge.from_idx 和 Edge.to_idx 必须使用转换后的数组 index, 不能直接使用内部 node_id
    """
    message_cache = cache if cache is not None else GraphMessageCache()
    return message_cache.build(graph, header, trav_class)


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
