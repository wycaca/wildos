from __future__ import annotations

from typing import Iterable, Tuple

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from graph_construction.graph_memory import GraphState, InternalNode


class GraphVisualizer:
    """为构建出的导航图生成 RViz marker

    这些 marker 只用于调试 Graph Construction 自身
    WildOS scoring 还会另外发布 nav_graph_viz, score_rings, model_visualization
    第一版可视化重点是检查节点, 边, frontier_points, radius 是否合理
    """

    def build_markers(self, graph: GraphState, header: Header) -> MarkerArray:
        """构建一帧完整 marker array"""
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(header))

        free_nodes = [node for node in graph.nodes.values() if not node.is_frontier]
        frontier_nodes = [node for node in graph.nodes.values() if node.is_frontier]

        markers.markers.append(
            self._sphere_list(
                header,
                marker_id=1,
                namespace="free_nodes",
                nodes=free_nodes,
                color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
                scale=0.25,
            )
        )
        markers.markers.append(
            self._sphere_list(
                header,
                marker_id=2,
                namespace="frontier_nodes",
                nodes=frontier_nodes,
                color=ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.95),
                scale=0.4,
            )
        )
        markers.markers.append(self._edge_marker(header, graph))
        markers.markers.append(self._frontier_point_marker(header, frontier_nodes))

        marker_id = 10
        for node in graph.nodes.values():
            markers.markers.append(
                self._radius_marker(
                    header,
                    marker_id=marker_id,
                    namespace="free_radius",
                    node=node,
                    radius=node.free_radius,
                    color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.16),
                )
            )
            marker_id += 1
            markers.markers.append(
                self._radius_marker(
                    header,
                    marker_id=marker_id,
                    namespace="explored_radius",
                    node=node,
                    radius=node.explored_radius,
                    color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.10),
                )
            )
            marker_id += 1

        if graph.current_node_id in graph.nodes:
            markers.markers.append(self._current_node_marker(header, graph.nodes[graph.current_node_id], marker_id))

        return markers

    def _delete_all_marker(self, header: Header) -> Marker:
        """清理上一帧 marker, 避免删除节点后 RViz 残留旧图元"""
        marker = Marker()
        marker.header = header
        marker.ns = "graph_construction"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _sphere_list(
        self,
        header: Header,
        marker_id: int,
        namespace: str,
        nodes: Iterable[InternalNode],
        color: ColorRGBA,
        scale: float,
    ) -> Marker:
        """将一组节点显示为同一个 SPHERE_LIST marker"""
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        marker.points = [self._point(node.position) for node in nodes]
        return marker

    def _edge_marker(self, header: Header, graph: GraphState) -> Marker:
        """将无向边显示为红色 LINE_LIST"""
        marker = Marker()
        marker.header = header
        marker.ns = "edges"
        marker.id = 3
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.04
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.75)
        for edge in graph.edges.values():
            node_a = graph.nodes.get(edge.from_id)
            node_b = graph.nodes.get(edge.to_id)
            if node_a is None or node_b is None:
                continue
            marker.points.append(self._point(node_a.position))
            marker.points.append(self._point(node_b.position))
        return marker

    def _frontier_point_marker(self, header: Header, frontier_nodes: Iterable[InternalNode]) -> Marker:
        """显示所有 frontier_points, 用于确认 frontier heading 是否正确"""
        marker = Marker()
        marker.header = header
        marker.ns = "frontier_points"
        marker.id = 4
        marker.action = Marker.ADD
        marker.type = Marker.CUBE_LIST
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=0.7, g=0.0, b=1.0, a=0.9)
        for node in frontier_nodes:
            marker.points.extend([self._point(point) for point in node.frontier_points])
        return marker

    def _radius_marker(
        self,
        header: Header,
        marker_id: int,
        namespace: str,
        node: InternalNode,
        radius: float,
        color: ColorRGBA,
    ) -> Marker:
        """用扁圆柱显示 free_radius 或 explored_radius"""
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.CYLINDER
        marker.pose.position = self._point(node.position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.01, radius * 2.0)
        marker.scale.y = max(0.01, radius * 2.0)
        marker.scale.z = 0.02
        marker.color = color
        return marker

    def _current_node_marker(self, header: Header, node: InternalNode, marker_id: int) -> Marker:
        """突出显示当前机器人所在或最近的 graph node"""
        marker = Marker()
        marker.header = header
        marker.ns = "current_node"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position = self._point(node.position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        return marker

    def _point(self, position: Tuple[float, float, float]) -> Point:
        """转换内部坐标为 RViz marker 使用的 Point"""
        point = Point()
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = float(position[2])
        return point
