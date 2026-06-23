from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import floor, hypot
from typing import Iterable, List, Optional, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid


GridIndex = Tuple[int, int]


@dataclass
class ClassifiedGrid:
    """由 OccupancyGrid 转换得到的三值栅格

    Graph Construction 后续只关心 free, obstacle, unknown 三类 cell
    这里把 ROS 的概率栅格先压缩成布尔掩码, 避免图构建逻辑依赖具体地图来源
    后续接入 GridMap 或 elevation map 时, 也应先转换成同样的三值结构
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    free: np.ndarray
    obstacle: np.ndarray
    unknown: np.ndarray

    def in_bounds(self, ix: int, iy: int) -> bool:
        """判断栅格索引是否落在地图范围内"""
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        """将世界坐标转换为栅格索引, 超出地图时返回 None"""
        ix = int(floor((x - self.origin_x) / self.resolution))
        iy = int(floor((y - self.origin_y) / self.resolution))
        if not self.in_bounds(ix, iy):
            return None
        return ix, iy

    def grid_to_world(self, ix: int, iy: int, z: float = 0.0) -> Tuple[float, float, float]:
        """将栅格索引转换为 cell 中心的世界坐标"""
        x = self.origin_x + (ix + 0.5) * self.resolution
        y = self.origin_y + (iy + 0.5) * self.resolution
        return x, y, z

    def is_free_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是已知可通行区域"""
        return self.in_bounds(ix, iy) and bool(self.free[iy, ix])

    def is_obstacle_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是障碍, 地图外默认按障碍处理"""
        return (not self.in_bounds(ix, iy)) or bool(self.obstacle[iy, ix])

    def is_unknown_index(self, ix: int, iy: int) -> bool:
        """判断指定 cell 是否是未知区域"""
        return self.in_bounds(ix, iy) and bool(self.unknown[iy, ix])

    def is_world_collision_free(self, start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> bool:
        """检查世界坐标下两点之间的直线是否穿过 obstacle 或 unknown

        第一版将 unknown 也视为不可穿越, 这样生成的图会更保守
        后续如果需要更激进探索, 可以允许边接近 unknown, 但不能穿过 obstacle
        """
        start = self.world_to_grid(start_xy[0], start_xy[1])
        end = self.world_to_grid(end_xy[0], end_xy[1])
        if start is None or end is None:
            return False
        for ix, iy in bresenham_line(start[0], start[1], end[0], end[1]):
            if self.is_obstacle_index(ix, iy) or self.is_unknown_index(ix, iy):
                return False
        return True


def classify_occupancy_grid(
    msg: OccupancyGrid,
    free_threshold: int,
    obstacle_threshold: int,
) -> ClassifiedGrid:
    """将 OccupancyGrid 数值转换为 free, obstacle, unknown 掩码

    OccupancyGrid 中 -1 表示 unknown
    小于 free_threshold 的非负值视为 free
    大于 obstacle_threshold 的值视为 obstacle
    中间灰区暂时不作为可通行区域使用, 避免第一版生成过于激进的边
    """
    data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    unknown = data < 0
    free = np.logical_and(data >= 0, data <= free_threshold)
    obstacle = data >= obstacle_threshold
    return ClassifiedGrid(
        width=msg.info.width,
        height=msg.info.height,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
        frame_id=msg.header.frame_id,
        free=free,
        obstacle=obstacle,
        unknown=unknown,
    )


def distance_to_mask(mask: np.ndarray, resolution: float) -> np.ndarray:
    """计算 8 邻接距离场, 不依赖 scipy

    输入 mask 表示目标 cell, 例如 obstacle 或 unknown
    输出中每个 cell 的值表示它到最近目标 cell 的近似欧氏距离
    这里使用 Dijkstra 风格的多源扩散, 便于在 ROS 环境缺少 scipy 时直接运行
    """
    height, width = mask.shape
    distances = np.full((height, width), np.inf, dtype=np.float32)
    queue: List[Tuple[float, int, int]] = []

    # 所有目标 cell 同时作为距离为 0 的源点入队, 相当于多源最短路
    source_ys, source_xs = np.where(mask)
    for iy, ix in zip(source_ys.tolist(), source_xs.tolist()):
        distances[iy, ix] = 0.0
        heappush(queue, (0.0, ix, iy))

    # 没有目标 cell 时给一个足够大的距离, 避免后续半径计算出现 inf
    if not queue:
        max_distance = hypot(width * resolution, height * resolution)
        distances.fill(max_distance)
        return distances

    # 8 邻接步长中, 对角线代价使用 sqrt(2), 直线代价使用 1
    neighbor_steps = (
        (-1, -1, 2**0.5),
        (0, -1, 1.0),
        (1, -1, 2**0.5),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, 2**0.5),
        (0, 1, 1.0),
        (1, 1, 2**0.5),
    )

    while queue:
        current_distance, ix, iy = heappop(queue)
        if current_distance > distances[iy, ix]:
            continue
        for dx, dy, step in neighbor_steps:
            nx = ix + dx
            ny = iy + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            next_distance = current_distance + step * resolution
            if next_distance < distances[ny, nx]:
                distances[ny, nx] = next_distance
                heappush(queue, (next_distance, nx, ny))

    return distances


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> Iterable[GridIndex]:
    """生成两个栅格索引之间离散直线经过的 cell

    建边和 frontier 分配都会用它做快速 collision check
    这只是第一版的几何近似, 后续可以替换为带 footprint inflation 的线段检查
    """
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x = x0
    y = y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        error2 = 2 * error
        if error2 >= dy:
            error += dy
            x += sx
        if error2 <= dx:
            error += dx
            y += sy
