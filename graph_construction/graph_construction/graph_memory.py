from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
from typing import Dict, Iterable, List, Optional, Set, Tuple
import uuid


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]
EdgeKey = Tuple[int, int]


@dataclass
class InternalNode:
    """内部图节点, 保存稳定身份和导航元数据

    node_id 是内部递增 id, 用于快速索引和生成稳定 UUID
    position 是图节点在全局 frame 下的位置, 第一版主要使用 XY 平面
    free_radius 和 explored_radius 对应论文中的自由半径和探索半径
    frontier_points 是该节点关联的未知边界点, WildOS scoring 会依赖这些点计算 frontier heading
    """

    node_id: int
    uuid_bytes: bytes
    position: Point3
    free_radius: float = 0.0
    explored_radius: float = 0.0
    frontier_points: List[Point3] = field(default_factory=list)
    is_frontier: bool = False
    last_seen_time: float = 0.0
    failed_frontier_count: int = 0
    is_robot_anchor: bool = False

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

    def create_node(
        self,
        position: Point3,
        stamp_seconds: float,
        is_robot_anchor: bool = False,
    ) -> InternalNode:
        """创建节点, 使用 node id 生成稳定 UUID

        UUID 必须稳定, 因为 WildOS 视觉评分会按 frontier UUID 缓存 score
        如果同一个物理节点频繁换 UUID, score ring 和 deadend memory 都会抖动
        """
        node_id = self.next_node_id
        self.next_node_id += 1
        node_uuid = uuid.uuid5(uuid.NAMESPACE_URL, f"wildos-graph-node-{node_id}")
        node = InternalNode(
            node_id=node_id,
            uuid_bytes=node_uuid.bytes,
            position=position,
            last_seen_time=stamp_seconds,
            is_robot_anchor=is_robot_anchor,
        )
        self.nodes[node_id] = node
        return node

    def remove_node(self, node_id: int) -> None:
        """删除节点, 同时删除所有关联边

        图边引用的是 node_id, 删除节点后必须同步清理边
        否则转换 NavigationGraph 时会产生非法 from_idx 或 to_idx
        """
        self.nodes.pop(node_id, None)
        self.edges = {
            key: edge
            for key, edge in self.edges.items()
            if edge.from_id != node_id and edge.to_id != node_id
        }
        if self.current_node_id == node_id:
            self.current_node_id = None

    def set_edges(self, edges: Iterable[InternalEdge]) -> None:
        """使用规范化无向边键写入合并后的持久边集合

        调用方先生成当前新边, 再合并未被可见障碍证伪的历史边
        normalize_edge_key 保证 (a,b) 和 (b,a) 不会重复存储
        """
        self.edges.clear()
        for edge in edges:
            key = normalize_edge_key(edge.from_id, edge.to_id)
            if key[0] != key[1]:
                self.edges[key] = InternalEdge(from_id=key[0], to_id=key[1], cost=edge.cost)

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
        node_ids = candidates if candidates is not None else self.nodes.keys()
        for node_id in node_ids:
            node = self.nodes.get(node_id)
            if node is None:
                continue
            distance = node.distance_xy(position)
            if max_distance is not None and distance > max_distance:
                continue
            if distance < best_distance:
                best_node = node
                best_distance = distance
        return best_node

    def node_ids_within(self, position: Point3, radius: float) -> Set[int]:
        """返回 XY 平面指定半径内的所有节点 id"""
        return {
            node_id
            for node_id, node in self.nodes.items()
            if node.distance_xy(position) <= radius
        }

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
