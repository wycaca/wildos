from __future__ import annotations

from math import isfinite
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from graph_construction.graph_memory import GraphState, InternalNode
from graph_construction.grid_types import ClassifiedGrid


class GraphVisualizer:
    """为构建出的导航图生成 RViz marker

    这些 marker 只用于调试 Graph Construction 自身
    WildOS scoring 还会另外发布 nav_graph_viz, score_rings, model_visualization
    第一版可视化重点是检查节点, 边, frontier_points, radius 是否合理
    """

    MAX_RADIUS_MARKER = 20.0
    GRAPH_MARKER_Z_LIFT = 0.25

    def build_markers(
        self,
        graph: GraphState,
        header: Header,
        grid: Optional[ClassifiedGrid] = None,
    ) -> MarkerArray:
        """构建一帧完整 marker array"""
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(header))

        stamp_seconds = _stamp_to_seconds(header)
        current_nodes = [
            node
            for node in graph.nodes.values()
            if self._is_current_node(node, stamp_seconds)
        ]
        memory_nodes = [
            node
            for node in graph.nodes.values()
            if not self._is_current_node(node, stamp_seconds)
        ]
        free_nodes = [node for node in current_nodes if not node.is_frontier]
        memory_free_nodes = [node for node in memory_nodes if not node.is_frontier]
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
                namespace="memory_free_nodes",
                nodes=memory_free_nodes,
                color=ColorRGBA(r=0.0, g=0.35, b=0.0, a=0.25),
                scale=0.18,
            )
        )
        markers.markers.append(
            self._sphere_list(
                header,
                marker_id=3,
                namespace="frontier_nodes",
                nodes=frontier_nodes,
                color=ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.95),
                scale=0.32,
            )
        )
        markers.markers.append(self._edge_marker(header, graph))
        markers.markers.append(self._frontier_point_marker(header, frontier_nodes))
        markers.markers.append(self._trajectory_marker(header, graph))
        if graph.latest_robot_position is not None:
            markers.markers.append(self._robot_position_marker(header, graph.latest_robot_position))
        if graph.latest_robot_odom_position is not None:
            markers.markers.append(
                self._robot_odom_position_marker(
                    header,
                    graph.latest_robot_odom_position,
                    graph.latest_robot_ground_projected,
                )
            )
        if grid is not None:
            markers.markers.append(self._grid_footprint_marker(header, grid))

        marker_id = 20
        for node in current_nodes:
            free_radius = self._safe_radius_for_marker(node.free_radius)
            if free_radius is not None:
                markers.markers.append(
                    self._radius_marker(
                        header,
                        marker_id=marker_id,
                        namespace="free_radius",
                        node=node,
                        radius=free_radius,
                        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.05),
                    )
                )
            marker_id += 1
            explored_radius = self._safe_radius_for_marker(node.explored_radius)
            if explored_radius is not None:
                markers.markers.append(
                    self._radius_marker(
                        header,
                        marker_id=marker_id,
                        namespace="explored_radius",
                        node=node,
                        radius=explored_radius,
                        color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.04),
                    )
                )
            marker_id += 1

        if graph.current_node_id in graph.nodes:
            markers.markers.append(self._current_node_marker(header, graph.nodes[graph.current_node_id]))

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
        marker.points = [self._graph_point(node.position) for node in nodes]
        return marker

    def _edge_marker(self, header: Header, graph: GraphState) -> Marker:
        """将无向边显示为半透明 LINE_LIST, 避免 RViz 中遮挡地面"""
        marker = Marker()
        marker.header = header
        marker.ns = "edges"
        marker.id = 4
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.02
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.35)
        for edge in graph.edges.values():
            node_a = graph.nodes.get(edge.from_id)
            node_b = graph.nodes.get(edge.to_id)
            if node_a is None or node_b is None:
                continue
            marker.points.append(self._graph_point(node_a.position))
            marker.points.append(self._graph_point(node_b.position))
        return marker

    def _frontier_point_marker(self, header: Header, frontier_nodes: Iterable[InternalNode]) -> Marker:
        """显示所有 frontier_points, 用于确认 frontier heading 是否正确"""
        marker = Marker()
        marker.header = header
        marker.ns = "frontier_points"
        marker.id = 5
        marker.action = Marker.ADD
        marker.type = Marker.CUBE_LIST
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=0.7, g=0.0, b=1.0, a=0.9)
        for node in frontier_nodes:
            marker.points.extend([self._graph_point(point) for point in node.frontier_points])
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
        marker.pose.position = self._graph_point(node.position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.01, radius * 2.0)
        marker.scale.y = max(0.01, radius * 2.0)
        marker.scale.z = 0.02
        marker.color = color
        return marker

    def _safe_radius_for_marker(self, radius: float) -> Optional[float]:
        """过滤 RViz 无法合理显示的半径值, 避免异常包围盒影响视角"""
        if not isfinite(radius) or radius <= 0.0:
            return None
        return min(radius, self.MAX_RADIUS_MARKER)

    def _trajectory_marker(self, header: Header, graph: GraphState) -> Marker:
        """单独显示机器人走过的位置, 避免和 current node 混淆"""
        marker = Marker()
        marker.header = header
        marker.ns = "trajectory"
        marker.id = 6
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.65)
        marker.points = [self._graph_point(point) for point in graph.trajectory_points]
        return marker

    def _current_node_marker(self, header: Header, node: InternalNode) -> Marker:
        """突出显示当前机器人所在或最近的 graph node"""
        marker = Marker()
        marker.header = header
        marker.ns = "current_node"
        marker.id = 7
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position = self._graph_point(node.position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
        return marker

    def _robot_position_marker(self, header: Header, position: Tuple[float, float, float]) -> Marker:
        """显示投影到 GridMap elevation 表面的机器人地面位置"""
        marker = Marker()
        marker.header = header
        marker.ns = "robot_position"
        marker.id = 9
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position = self._point(position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        return marker

    def _robot_odom_position_marker(
        self,
        header: Header,
        position: Tuple[float, float, float],
        ground_projected: bool,
    ) -> Marker:
        """显示原始 odom / base 位置, 便于和地面投影点对比"""
        marker = Marker()
        marker.header = header
        marker.ns = "robot_odom_position"
        marker.id = 10
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position = self._point(position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        if ground_projected:
            marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
        else:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        return marker

    def _grid_footprint_marker(self, header: Header, grid: ClassifiedGrid) -> Marker:
        """显示当前局部 grid footprint, 便于区分历史 graph memory"""
        marker = Marker()
        marker.header = header
        marker.ns = "grid_footprint"
        marker.id = 8
        marker.action = Marker.ADD
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)
        z = grid.z_offset + 0.03
        if grid.grid_map_convention:
            length_x = grid.grid_map_length_x or grid.height * grid.resolution
            length_y = grid.grid_map_length_y or grid.width * grid.resolution
            corners = (
                (grid.origin_x, grid.origin_y, z),
                (grid.origin_x - length_x, grid.origin_y, z),
                (grid.origin_x - length_x, grid.origin_y - length_y, z),
                (grid.origin_x, grid.origin_y - length_y, z),
                (grid.origin_x, grid.origin_y, z),
            )
        else:
            width = grid.width * grid.resolution
            height = grid.height * grid.resolution
            corners = (
                (grid.origin_x, grid.origin_y, z),
                (grid.origin_x + width, grid.origin_y, z),
                (grid.origin_x + width, grid.origin_y + height, z),
                (grid.origin_x, grid.origin_y + height, z),
                (grid.origin_x, grid.origin_y, z),
            )
        marker.points = [self._point(corner) for corner in corners]
        return marker

    def _point(self, position: Tuple[float, float, float]) -> Point:
        """转换内部坐标为 RViz marker 使用的 Point"""
        point = Point()
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = float(position[2])
        return point

    def _graph_point(self, position: Tuple[float, float, float]) -> Point:
        """抬高 graph marker, 避免被 GridMap surface 深度遮挡"""
        return self._point((position[0], position[1], position[2] + self.GRAPH_MARKER_Z_LIFT))

    def _is_current_node(self, node: InternalNode, stamp_seconds: float) -> bool:
        """判断节点是否来自当前局部 grid 更新"""
        return abs(node.last_seen_time - stamp_seconds) < 1e-6


def _stamp_to_seconds(header: Header) -> float:
    """将 marker header 时间转为秒"""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
