from __future__ import annotations

from collections import deque
from math import hypot, isfinite
from typing import Iterable, Sequence

from geometry_msgs.msg import Point
from graphnav_msgs.msg import NavigationGraph, Node as GraphNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


class GraphVisualizer:
    """只使用公开 NavigationGraph 构建 RViz marker"""

    MAX_RADIUS_MARKER = 20.0
    GRAPH_MARKER_Z_LIFT = 0.25

    def __init__(
        self,
        show_radius_markers: bool = False,
        show_full_edges: bool = False,
    ) -> None:
        self.show_radius_markers = bool(show_radius_markers)
        self.show_full_edges = bool(show_full_edges)

    def build_markers(
        self,
        graph: NavigationGraph,
        trajectory: Sequence[tuple[float, float, float]] = (),
        odom_position: tuple[float, float, float] | None = None,
    ) -> MarkerArray:
        """构建完整 marker 帧, 不依赖图核心内部状态"""
        header = graph.header
        trav_index = _traversability_index(graph)
        frontier_nodes = [
            node
            for node in graph.nodes
            if _is_frontier(node, trav_index)
        ]
        free_nodes = [
            node
            for node in graph.nodes
            if not _is_frontier(node, trav_index)
        ]
        markers = MarkerArray()
        markers.markers = [
            self._delete_all_marker(header),
            self._sphere_list(
                header,
                1,
                "free_nodes",
                free_nodes,
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
                0.25,
            ),
            self._sphere_list(
                header,
                2,
                "memory_free_nodes",
                (),
                ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9),
                0.25,
            ),
            self._sphere_list(
                header,
                3,
                "frontier_nodes",
                frontier_nodes,
                ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.95),
                0.32,
            ),
            self._edge_marker(graph),
            self._frontier_point_marker(header, frontier_nodes, trav_index),
            self._trajectory_marker(header, trajectory),
        ]
        current_node = _current_node(graph)
        if current_node is not None:
            markers.markers.append(
                self._position_marker(
                    header,
                    current_node.pose.position,
                    "robot_position",
                    9,
                    0.5,
                    ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                )
            )
        if odom_position is not None:
            markers.markers.append(
                self._position_marker(
                    header,
                    self._point(odom_position),
                    "robot_odom_position",
                    10,
                    0.35,
                    ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8),
                )
            )
        if self.show_radius_markers:
            self._append_radius_markers(
                markers,
                header,
                graph.nodes,
                trav_index,
            )
        return markers

    def _delete_all_marker(self, header: Header) -> Marker:
        marker = Marker(header=header, ns="graph_construction", id=0)
        marker.action = Marker.DELETEALL
        return marker

    def _sphere_list(
        self,
        header: Header,
        marker_id: int,
        namespace: str,
        nodes: Iterable[GraphNode],
        color: ColorRGBA,
        scale: float,
    ) -> Marker:
        marker = Marker(header=header, ns=namespace, id=marker_id)
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        marker.points = [self._graph_point(node.pose.position) for node in nodes]
        return marker

    def _edge_marker(self, graph: NavigationGraph) -> Marker:
        marker = Marker(header=graph.header, ns="edges", id=4)
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.03
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.35)
        for edge_index in self._visible_edge_indices(graph):
            edge = graph.edges[edge_index]
            if edge.from_idx >= len(graph.nodes) or edge.to_idx >= len(graph.nodes):
                continue
            marker.points.append(
                self._graph_point(graph.nodes[edge.from_idx].pose.position)
            )
            marker.points.append(
                self._graph_point(graph.nodes[edge.to_idx].pose.position)
            )
        return marker

    def _visible_edge_indices(self, graph: NavigationGraph) -> list[int]:
        """默认显示从 current node 开始的最小代价生成森林"""
        valid_edges = [
            index
            for index, edge in enumerate(graph.edges)
            if edge.from_idx < len(graph.nodes)
            and edge.to_idx < len(graph.nodes)
            and edge.from_idx != edge.to_idx
        ]
        if self.show_full_edges:
            return valid_edges

        adjacency: list[list[int]] = [[] for _ in graph.nodes]
        for edge_index in valid_edges:
            edge = graph.edges[edge_index]
            adjacency[edge.from_idx].append(edge_index)
            adjacency[edge.to_idx].append(edge_index)
        roots = list(range(len(graph.nodes)))
        if graph.current_node_idx < len(graph.nodes):
            roots.remove(graph.current_node_idx)
            roots.insert(0, graph.current_node_idx)

        visited: set[int] = set()
        visible: list[int] = []
        for root in roots:
            if root in visited:
                continue
            visited.add(root)
            pending = deque([root])
            while pending:
                node_index = pending.popleft()
                for edge_index in sorted(
                    adjacency[node_index],
                    key=lambda index: (_edge_cost(graph.edges[index]), index),
                ):
                    edge = graph.edges[edge_index]
                    other = (
                        edge.to_idx
                        if edge.from_idx == node_index
                        else edge.from_idx
                    )
                    if other in visited:
                        continue
                    visited.add(other)
                    pending.append(other)
                    visible.append(edge_index)
        return visible

    def _frontier_point_marker(
        self,
        header: Header,
        nodes: Iterable[GraphNode],
        trav_index: int | None,
    ) -> Marker:
        marker = Marker(header=header, ns="frontier_points", id=5)
        marker.action = Marker.ADD
        marker.type = Marker.CUBE_LIST
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=0.7, g=0.0, b=1.0, a=0.9)
        if trav_index is None:
            return marker
        for node in nodes:
            if trav_index >= len(node.trav_properties):
                continue
            marker.points.extend(
                self._graph_point(point)
                for point in node.trav_properties[trav_index].frontier_points
            )
        return marker

    def _append_radius_markers(
        self,
        markers: MarkerArray,
        header: Header,
        nodes: Iterable[GraphNode],
        trav_index: int | None,
    ) -> None:
        if trav_index is None:
            return
        marker_id = 20
        for node in nodes:
            if trav_index >= len(node.trav_properties):
                continue
            properties = node.trav_properties[trav_index]
            for namespace, radius, color in (
                (
                    "free_radius",
                    properties.free_radius,
                    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.05),
                ),
                (
                    "explored_radius",
                    properties.explored_radius,
                    ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.04),
                ),
            ):
                safe_radius = self._safe_radius(radius)
                if safe_radius is not None:
                    markers.markers.append(
                        self._radius_marker(
                            header,
                            marker_id,
                            namespace,
                            node,
                            safe_radius,
                            color,
                        )
                    )
                marker_id += 1

    def _radius_marker(
        self,
        header: Header,
        marker_id: int,
        namespace: str,
        node: GraphNode,
        radius: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker(header=header, ns=namespace, id=marker_id)
        marker.action = Marker.ADD
        marker.type = Marker.CYLINDER
        marker.pose.position = self._graph_point(node.pose.position)
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.02
        marker.color = color
        return marker

    def _trajectory_marker(
        self,
        header: Header,
        trajectory: Sequence[tuple[float, float, float]],
    ) -> Marker:
        marker = Marker(header=header, ns="trajectory", id=6)
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.65)
        marker.points = [self._graph_point(point) for point in trajectory]
        return marker

    def _position_marker(
        self,
        header: Header,
        position: Point,
        namespace: str,
        marker_id: int,
        scale: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker(header=header, ns=namespace, id=marker_id)
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def _safe_radius(self, radius: float) -> float | None:
        if not isfinite(radius) or radius <= 0.0:
            return None
        return min(float(radius), self.MAX_RADIUS_MARKER)

    def _graph_point(self, position) -> Point:
        point = self._point(position)
        point.z += self.GRAPH_MARKER_Z_LIFT
        return point

    @staticmethod
    def _point(position) -> Point:
        point = Point()
        if hasattr(position, "x"):
            point.x = float(position.x)
            point.y = float(position.y)
            point.z = float(position.z)
        else:
            point.x = float(position[0])
            point.y = float(position[1])
            point.z = float(position[2])
        return point


class GraphVisualizerNode(Node):
    """独立订阅公开图和 odom, 不进入图构建 executor"""

    def __init__(self) -> None:
        super().__init__("graph_visualizer")
        self.declare_parameter("nav_graph_topic", "/spot1/nav_graph")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("viz_topic", "/spot1/graph_construction_viz")
        self.declare_parameter("show_radius_markers", False)
        self.declare_parameter("show_full_edges", False)
        self.declare_parameter("trajectory_min_separation", 0.25)
        self.declare_parameter("trajectory_max_points", 4000)

        self.visualizer = GraphVisualizer(
            self.get_parameter("show_radius_markers").value,
            self.get_parameter("show_full_edges").value,
        )
        self.trajectory_min_separation = float(
            self.get_parameter("trajectory_min_separation").value
        )
        max_points = max(1, int(self.get_parameter("trajectory_max_points").value))
        self.trajectory: deque[tuple[float, float, float]] = deque(maxlen=max_points)
        self.latest_odom_position: tuple[float, float, float] | None = None
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("viz_topic").value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(
            NavigationGraph,
            str(self.get_parameter("nav_graph_topic").value),
            self._on_graph,
            1,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        current = (float(position.x), float(position.y), float(position.z))
        self.latest_odom_position = current
        if not self.trajectory:
            self.trajectory.append(current)
            return
        previous = self.trajectory[-1]
        if hypot(current[0] - previous[0], current[1] - previous[1]) >= (
            self.trajectory_min_separation
        ):
            self.trajectory.append(current)

    def _on_graph(self, msg: NavigationGraph) -> None:
        if self.publisher.get_subscription_count() == 0:
            return
        self.publisher.publish(
            self.visualizer.build_markers(
                msg,
                tuple(self.trajectory),
                self.latest_odom_position,
            )
        )


def _traversability_index(graph: NavigationGraph) -> int | None:
    if not graph.trav_classes:
        return None
    try:
        return graph.trav_classes.index("default")
    except ValueError:
        return 0


def _is_frontier(node: GraphNode, trav_index: int | None) -> bool:
    return (
        trav_index is not None
        and trav_index < len(node.trav_properties)
        and node.trav_properties[trav_index].is_frontier
    )


def _current_node(graph: NavigationGraph) -> GraphNode | None:
    if graph.current_node_idx >= len(graph.nodes):
        return None
    return graph.nodes[graph.current_node_idx]


def _edge_cost(edge) -> float:
    if not edge.traversability:
        return 0.0
    return float(edge.traversability[0].traversability_cost)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraphVisualizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
