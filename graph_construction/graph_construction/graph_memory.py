from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, floor, hypot
from typing import Dict, Iterable, List, Optional, Set, Tuple
import uuid


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]
EdgeKey = Tuple[int, int]
SpatialKey = Tuple[int, int]


class NodeSpatialIndex:
    """持久维护节点空间桶, 支持局部范围和半径查询"""

    def __init__(self, bucket_size: float = 1.0) -> None:
        self.bucket_size = max(0.1, float(bucket_size))
        self._buckets: Dict[SpatialKey, Set[int]] = {}
        self._node_keys: Dict[int, SpatialKey] = {}
        self._positions: Dict[int, Point3] = {}

    def insert(self, node_id: int, position: Point3) -> None:
        """插入节点或同步已经存在的节点位置"""
        key = self._key(position)
        if self._node_keys.get(node_id) == key:
            self._positions[node_id] = position
            return
        self.remove(node_id)
        self._buckets.setdefault(key, set()).add(node_id)
        self._node_keys[node_id] = key
        self._positions[node_id] = position

    def remove(self, node_id: int) -> None:
        """删除节点并清理空桶"""
        key = self._node_keys.pop(node_id, None)
        self._positions.pop(node_id, None)
        if key is None:
            return
        bucket = self._buckets.get(key)
        if bucket is None:
            return
        bucket.discard(node_id)
        if not bucket:
            self._buckets.pop(key, None)

    def query_bounds(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
    ) -> Set[int]:
        """返回轴对齐包围盒内的节点 id"""
        if min_x > max_x or min_y > max_y:
            return set()
        min_key = self._key((min_x, min_y, 0.0))
        max_key = self._key((max_x, max_y, 0.0))
        result: Set[int] = set()
        for bucket_y in range(min_key[1], max_key[1] + 1):
            for bucket_x in range(min_key[0], max_key[0] + 1):
                for node_id in self._buckets.get((bucket_x, bucket_y), ()):
                    position = self._positions[node_id]
                    if (
                        min_x <= position[0] <= max_x
                        and min_y <= position[1] <= max_y
                    ):
                        result.add(node_id)
        return result

    def query_radius(self, position: Point3, radius: float) -> Set[int]:
        """返回 XY 半径内的节点 id"""
        safe_radius = max(0.0, float(radius))
        candidates = self.query_bounds(
            position[0] - safe_radius,
            position[0] + safe_radius,
            position[1] - safe_radius,
            position[1] + safe_radius,
        )
        radius_squared = safe_radius * safe_radius
        return {
            node_id
            for node_id in candidates
            if (
                (self._positions[node_id][0] - position[0]) ** 2
                + (self._positions[node_id][1] - position[1]) ** 2
                <= radius_squared
            )
        }

    def _key(self, position: Point3) -> SpatialKey:
        return (
            int(floor(position[0] / self.bucket_size)),
            int(floor(position[1] / self.bucket_size)),
        )


class EdgeSpatialIndex:
    """按世界空间桶记录边经过的区域, 支持局部地图变化精确查边"""

    def __init__(self, bucket_size: float = 0.5) -> None:
        self.bucket_size = max(0.1, float(bucket_size))
        self._buckets: Dict[SpatialKey, Set[EdgeKey]] = {}
        self._edge_keys: Dict[EdgeKey, Set[SpatialKey]] = {}

    def insert(self, edge_key: EdgeKey, start: Point3, end: Point3) -> None:
        """插入边经过的空间桶, 替换已有索引"""
        self.remove(edge_key)
        spatial_keys = set(self._line_keys(start, end))
        self._edge_keys[edge_key] = spatial_keys
        for spatial_key in spatial_keys:
            self._buckets.setdefault(spatial_key, set()).add(edge_key)

    def remove(self, edge_key: EdgeKey) -> None:
        """删除边及其全部空间桶引用"""
        for spatial_key in self._edge_keys.pop(edge_key, ()):
            bucket = self._buckets.get(spatial_key)
            if bucket is None:
                continue
            bucket.discard(edge_key)
            if not bucket:
                self._buckets.pop(spatial_key, None)

    def clear(self) -> None:
        """清空全部边空间索引"""
        self._buckets.clear()
        self._edge_keys.clear()

    def query_points(self, points: Iterable[Point2], radius: float = 0.0) -> Set[EdgeKey]:
        """返回经过查询点附近空间桶的边"""
        bucket_radius = max(0, int(ceil(max(0.0, float(radius)) / self.bucket_size)))
        result: Set[EdgeKey] = set()
        for point in points:
            center_x = int(floor(point[0] / self.bucket_size))
            center_y = int(floor(point[1] / self.bucket_size))
            for offset_y in range(-bucket_radius, bucket_radius + 1):
                for offset_x in range(-bucket_radius, bucket_radius + 1):
                    result.update(
                        self._buckets.get((center_x + offset_x, center_y + offset_y), ())
                    )
        return result

    def _line_keys(self, start: Point3, end: Point3) -> Iterable[SpatialKey]:
        """使用整数栅格线生成边中心线经过的世界空间桶"""
        x0 = int(floor(start[0] / self.bucket_size))
        y0 = int(floor(start[1] / self.bucket_size))
        x1 = int(floor(end[0] / self.bucket_size))
        y1 = int(floor(end[1] / self.bucket_size))
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx - dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            doubled_error = 2 * error
            if doubled_error > -dy:
                error -= dy
                x += step_x
            if doubled_error < dx:
                error += dx
                y += step_y


@dataclass
class InternalNode:
    """内部图节点, 保存稳定身份和导航元数据

    node_id 是内部递增 id, 用于快速索引和生成稳定 UUID
    position 是图节点在全局 frame 下的位置, 第一版主要使用 XY 平面
    free_radius 和 explored_radius 对应论文中的自由半径和探索半径
    frontier_points 是当前地图中该节点关联的未知边界点, WildOS scoring 用它计算当前 heading
    """

    node_id: int
    uuid_bytes: bytes
    position: Point3
    free_radius: float = 0.0
    explored_radius: float = 0.0
    frontier_points: List[Point3] = field(default_factory=list)
    is_frontier: bool = False
    last_seen_time: float = 0.0

    def distance_xy(self, position: Point3) -> float:
        """计算 XY 平面距离, 忽略高度差以匹配当前地面机器人规划假设"""
        return hypot(self.position[0] - position[0], self.position[1] - position[1])


@dataclass
class InternalEdge:
    """内部节点之间的无向加权边

    cost 会被转换为 EdgeTraversability.traversability_cost
    第一版 cost 使用欧氏距离, 后续可以叠加坡度, 粗糙度, clearance 等代价
    """

    from_id: int
    to_id: int
    cost: float


@dataclass
class GraphState:
    """跨局部地图更新保留的图记忆

    这个对象是死路回退和长期探索记忆的核心
    即使局部几何地图滑窗移动, 已有节点也会尽量保留
    只有节点落入障碍或自由半径过小时才删除
    """

    nodes: Dict[int, InternalNode] = field(default_factory=dict)
    edges: Dict[EdgeKey, InternalEdge] = field(default_factory=dict)
    current_node_id: Optional[int] = None
    latest_robot_odom_position: Optional[Point3] = None
    latest_robot_position: Optional[Point3] = None
    latest_robot_ground_projected: bool = False
    trajectory_points: List[Point3] = field(default_factory=list)
    next_node_id: int = 0
    spatial_index: NodeSpatialIndex = field(
        default_factory=NodeSpatialIndex,
        init=False,
        repr=False,
    )
    adjacency: Dict[int, Set[EdgeKey]] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    edge_spatial_index: EdgeSpatialIndex = field(
        default_factory=EdgeSpatialIndex,
        init=False,
        repr=False,
    )

    def __post_init__(self) -> None:
        """为显式传入的节点和边恢复派生索引"""
        for node in self.nodes.values():
            self.spatial_index.insert(node.node_id, node.position)
        initial_edges = list(self.edges.values())
        self.edges.clear()
        self.set_edges(initial_edges)

    def create_node(
        self,
        position: Point3,
        stamp_seconds: float,
    ) -> InternalNode:
        """创建节点, 使用 node id 生成稳定 UUID

        UUID 必须稳定, 因为 traversal memory 和 deferred branch 都引用持久节点
        如果同一个物理节点频繁换 UUID, 历史路线和分支入口都会失效
        """
        node_id = self.next_node_id
        self.next_node_id += 1
        node_uuid = uuid.uuid5(uuid.NAMESPACE_URL, f"wildos-graph-node-{node_id}")
        node = InternalNode(
            node_id=node_id,
            uuid_bytes=node_uuid.bytes,
            position=position,
            last_seen_time=stamp_seconds,
        )
        self.nodes[node_id] = node
        self.spatial_index.insert(node_id, position)
        self.adjacency.setdefault(node_id, set())
        return node

    def move_node(self, node_id: int, position: Point3) -> None:
        """移动节点并同步持久空间索引"""
        node = self.nodes.get(node_id)
        if node is None:
            return
        xy_changed = node.position[:2] != position[:2]
        node.position = position
        self.spatial_index.insert(node_id, position)
        if not xy_changed:
            return
        for edge_key in self.adjacency.get(node_id, ()):
            edge = self.edges.get(edge_key)
            if edge is None:
                continue
            self.edge_spatial_index.insert(
                edge_key,
                self.nodes[edge.from_id].position,
                self.nodes[edge.to_id].position,
            )

    def remove_node(self, node_id: int) -> None:
        """删除节点, 同时删除所有关联边

        图边引用的是 node_id, 删除节点后必须同步清理边
        否则转换 NavigationGraph 时会产生非法 from_idx 或 to_idx
        """
        self.nodes.pop(node_id, None)
        self.spatial_index.remove(node_id)
        for edge_key in tuple(self.adjacency.pop(node_id, ())):
            self._remove_edge(edge_key)
        if self.current_node_id == node_id:
            self.current_node_id = None

    def set_edges(self, edges: Iterable[InternalEdge]) -> None:
        """使用规范化无向边键写入完整边集合

        该接口用于初始化和测试, 在线更新使用 apply_edge_delta
        normalize_edge_key 保证 (a,b) 和 (b,a) 不会重复存储
        """
        replacement_edges = list(edges)
        self.edges.clear()
        self.edge_spatial_index.clear()
        self.adjacency = {
            node_id: set()
            for node_id in self.nodes
        }
        for edge in replacement_edges:
            self._set_edge(edge)

    def apply_edge_delta(
        self,
        edge_keys_to_remove: Iterable[EdgeKey],
        edges_to_add: Iterable[InternalEdge],
    ) -> None:
        """应用最小边增量, 未变化边保持对象和索引不动"""
        for edge_key in edge_keys_to_remove:
            self._remove_edge(normalize_edge_key(*edge_key))
        for edge in edges_to_add:
            key = normalize_edge_key(edge.from_id, edge.to_id)
            if key not in self.edges:
                self._set_edge(edge)

    def edge_keys_for_nodes(self, node_ids: Iterable[int]) -> Set[EdgeKey]:
        """通过邻接索引收集节点关联边"""
        edge_keys: Set[EdgeKey] = set()
        for node_id in node_ids:
            edge_keys.update(self.adjacency.get(node_id, ()))
        return edge_keys

    def edge_keys_near_points(
        self,
        points: Iterable[Point2],
        radius: float = 0.0,
    ) -> Set[EdgeKey]:
        """通过持久边空间索引查询地图变化附近的边"""
        return self.edge_spatial_index.query_points(points, radius)

    def node_ids_in_bounds(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
    ) -> Set[int]:
        """通过持久索引查询包围盒内节点"""
        return self.spatial_index.query_bounds(min_x, max_x, min_y, max_y)

    def node_ids_within(self, position: Point3, radius: float) -> Set[int]:
        """通过持久索引查询半径内节点"""
        return self.spatial_index.query_radius(position, radius)

    def nearest_node(
        self,
        position: Point3,
        max_distance: Optional[float] = None,
        candidates: Optional[Iterable[int]] = None,
    ) -> Optional[InternalNode]:
        """在 XY 平面查找最近节点

        max_distance 用于限制匹配范围, 例如采样新节点时避免过密
        candidates 用于只在指定节点集合内搜索, 当前第一版暂未大量使用
        """
        best_node = None
        best_distance = float("inf")
        if candidates is not None:
            node_ids = candidates
        elif max_distance is not None:
            node_ids = self.node_ids_within(position, max_distance)
        else:
            node_ids = self.nodes.keys()
        for node_id in node_ids:
            node = self.nodes.get(node_id)
            if node is None:
                continue
            distance = node.distance_xy(position)
            if max_distance is not None and distance > max_distance:
                continue
            if distance < best_distance or (
                distance == best_distance
                and best_node is not None
                and node.node_id < best_node.node_id
            ):
                best_node = node
                best_distance = distance
        return best_node

    def _set_edge(self, edge: InternalEdge) -> None:
        """规范化单条边并同步邻接索引"""
        key = normalize_edge_key(edge.from_id, edge.to_id)
        if key[0] == key[1] or key[0] not in self.nodes or key[1] not in self.nodes:
            return
        if key in self.edges:
            self._remove_edge(key)
        self.edges[key] = InternalEdge(
            from_id=key[0],
            to_id=key[1],
            cost=edge.cost,
        )
        self.adjacency.setdefault(key[0], set()).add(key)
        self.adjacency.setdefault(key[1], set()).add(key)
        self.edge_spatial_index.insert(
            key,
            self.nodes[key[0]].position,
            self.nodes[key[1]].position,
        )

    def _remove_edge(self, edge_key: EdgeKey) -> None:
        """删除单条边并同步邻接索引"""
        edge = self.edges.pop(edge_key, None)
        if edge is None:
            return
        self.edge_spatial_index.remove(edge_key)
        self.adjacency.get(edge.from_id, set()).discard(edge_key)
        self.adjacency.get(edge.to_id, set()).discard(edge_key)

    def update_robot_position(self, odom_position: Point3, ground_position: Optional[Point3]) -> None:
        """记录机器人原始 odom 位置和投影到高程图的地面位置"""
        self.latest_robot_odom_position = odom_position
        self.latest_robot_position = ground_position
        self.latest_robot_ground_projected = ground_position is not None

    def append_trajectory_point(self, position: Point3, min_separation: float) -> None:
        """记录机器人走过的位置, 与 graph nodes 分开显示"""
        if self.trajectory_points:
            last = self.trajectory_points[-1]
            if hypot(last[0] - position[0], last[1] - position[1]) < min_separation:
                return
        self.trajectory_points.append(position)


def normalize_edge_key(from_id: int, to_id: int) -> EdgeKey:
    """规范化无向边键, 便于稳定存储"""
    return (from_id, to_id) if from_id <= to_id else (to_id, from_id)
