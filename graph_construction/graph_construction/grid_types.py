from __future__ import annotations

from dataclasses import dataclass
from math import cos, floor, hypot, sin
from typing import Iterable, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt


GridIndex = Tuple[int, int]


@dataclass
class ClassifiedGrid:
    """图算法使用的通用 free, obstacle, unknown grid"""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    free: np.ndarray
    obstacle: np.ndarray
    unknown: np.ndarray
    elevation: Optional[np.ndarray] = None
    z_offset: float = 0.0
    stats: Optional[dict] = None
    grid_map_center_x: Optional[float] = None
    grid_map_center_y: Optional[float] = None
    grid_map_length_x: Optional[float] = None
    grid_map_length_y: Optional[float] = None
    grid_map_yaw: float = 0.0
    grid_map_convention: bool = False

    def in_bounds(self, ix: int, iy: int) -> bool:
        """判断 cell index 是否在 grid 内"""
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        """把 world XY 转成 grid index"""
        if self.grid_map_convention:
            local_x, local_y = self._world_to_map_axes(x, y)
            length_x = self.grid_map_length_x or self.height * self.resolution
            length_y = self.grid_map_length_y or self.width * self.resolution
            iy = int(floor((length_x * 0.5 - local_x) / self.resolution))
            ix = int(floor((length_y * 0.5 - local_y) / self.resolution))
        else:
            ix = int(floor((x - self.origin_x) / self.resolution))
            iy = int(floor((y - self.origin_y) / self.resolution))
        if not self.in_bounds(ix, iy):
            return None
        return ix, iy

    def grid_to_world(
        self,
        ix: int,
        iy: int,
        z: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """把 grid index 转成 world-space cell 中心"""
        if self.grid_map_convention:
            length_x = self.grid_map_length_x or self.height * self.resolution
            length_y = self.grid_map_length_y or self.width * self.resolution
            local_x = length_x * 0.5 - (iy + 0.5) * self.resolution
            local_y = length_y * 0.5 - (ix + 0.5) * self.resolution
            x, y = self._map_axes_to_world(local_x, local_y)
        else:
            x = self.origin_x + (ix + 0.5) * self.resolution
            y = self.origin_y + (iy + 0.5) * self.resolution
        if z is None:
            z = self.elevation_at_index(ix, iy)
        return x, y, z

    def elevation_at_index(self, ix: int, iy: int) -> float:
        """返回 cell 的 elevation, 没有数据时使用 z_offset"""
        if self.elevation is None or not self.in_bounds(ix, iy):
            return self.z_offset
        value = float(self.elevation[iy, ix])
        if not np.isfinite(value):
            return self.z_offset
        return value + self.z_offset

    def elevation_at_world(self, x: float, y: float) -> Optional[float]:
        """返回 world XY 对应的 GridMap elevation"""
        if self.elevation is None:
            return None
        grid_index = self.world_to_grid(x, y)
        if grid_index is None:
            return None
        ix, iy = grid_index
        value = float(self.elevation[iy, ix])
        if not np.isfinite(value):
            value = self._nearest_finite_elevation(ix, iy)
        if value is None:
            return None
        return value + self.z_offset

    def _nearest_finite_elevation(
        self,
        ix: int,
        iy: int,
        radius_cells: int = 3,
    ) -> Optional[float]:
        """为小范围空洞查找附近有限 elevation"""
        if self.elevation is None:
            return None
        best_value = None
        best_distance = float("inf")
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = ix + dx
                ny = iy + dy
                if not self.in_bounds(nx, ny):
                    continue
                value = float(self.elevation[ny, nx])
                if not np.isfinite(value):
                    continue
                distance = hypot(float(dx), float(dy))
                if distance < best_distance:
                    best_distance = distance
                    best_value = value
        return best_value

    def project_to_elevation(
        self,
        position: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """尽量把 XY 位置投影到 elevation 表面"""
        projected, _ = self.project_to_elevation_with_status(position)
        return projected

    def project_to_elevation_with_status(
        self,
        position: Tuple[float, float, float],
    ) -> Tuple[Tuple[float, float, float], bool]:
        """把 XY 位置投影到 elevation, 并返回是否成功"""
        elevation = self.elevation_at_world(position[0], position[1])
        if elevation is None:
            return position, False
        return (position[0], position[1], elevation), True

    def is_free_index(self, ix: int, iy: int) -> bool:
        """判断 cell 是否为已知 free"""
        return self.in_bounds(ix, iy) and bool(self.free[iy, ix])

    def is_obstacle_index(self, ix: int, iy: int) -> bool:
        """判断 cell 是否为 obstacle 或越界"""
        return (not self.in_bounds(ix, iy)) or bool(self.obstacle[iy, ix])

    def is_unknown_index(self, ix: int, iy: int) -> bool:
        """判断 cell 是否为 unknown"""
        return self.in_bounds(ix, iy) and bool(self.unknown[iy, ix])

    def is_world_collision_free(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> bool:
        """检查 world-space 线段是否始终位于已知 free cell"""
        line_cells = list(self.world_line_cells(start_xy, end_xy))
        if not line_cells:
            return False
        for ix, iy in line_cells:
            if self.is_obstacle_index(ix, iy) or self.is_unknown_index(ix, iy):
                return False
        return True

    def world_line_cells(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> Iterable[GridIndex]:
        """生成 world-space 线段经过的 grid cell, 越界时返回空序列"""
        start = self.world_to_grid(start_xy[0], start_xy[1])
        end = self.world_to_grid(end_xy[0], end_xy[1])
        if start is None or end is None:
            return ()
        return bresenham_line(start[0], start[1], end[0], end[1])

    def world_line_cells_clipped(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> Iterable[GridIndex]:
        """生成线段在当前 grid 可见范围内经过的 cell"""
        start = self._world_to_grid_float(start_xy[0], start_xy[1])
        end = self._world_to_grid_float(end_xy[0], end_xy[1])
        clipped = _clip_float_segment_to_bounds(start, end, self.width, self.height)
        if clipped is None:
            return ()

        (x0, y0), (x1, y1) = clipped
        ix0 = _clamp_cell_index(x0, self.width)
        iy0 = _clamp_cell_index(y0, self.height)
        ix1 = _clamp_cell_index(x1, self.width)
        iy1 = _clamp_cell_index(y1, self.height)
        return bresenham_line(ix0, iy0, ix1, iy1)

    def _world_to_grid_float(self, x: float, y: float) -> Tuple[float, float]:
        """把 world XY 转成连续 grid 坐标, 允许越界"""
        if self.grid_map_convention:
            local_x, local_y = self._world_to_map_axes(x, y)
            length_x = self.grid_map_length_x or self.height * self.resolution
            length_y = self.grid_map_length_y or self.width * self.resolution
            fy = (length_x * 0.5 - local_x) / self.resolution
            fx = (length_y * 0.5 - local_y) / self.resolution
            return fx, fy
        return (
            (x - self.origin_x) / self.resolution,
            (y - self.origin_y) / self.resolution,
        )

    def _world_to_map_axes(self, x: float, y: float) -> Tuple[float, float]:
        """把 world XY 转到 GridMap 本地轴坐标"""
        center_x = self.grid_map_center_x if self.grid_map_center_x is not None else self.origin_x
        center_y = self.grid_map_center_y if self.grid_map_center_y is not None else self.origin_y
        dx = x - center_x
        dy = y - center_y
        yaw_cos = cos(self.grid_map_yaw)
        yaw_sin = sin(self.grid_map_yaw)
        local_x = yaw_cos * dx + yaw_sin * dy
        local_y = -yaw_sin * dx + yaw_cos * dy
        return local_x, local_y

    def _map_axes_to_world(self, local_x: float, local_y: float) -> Tuple[float, float]:
        """把 GridMap 本地轴坐标转到 world XY"""
        center_x = self.grid_map_center_x if self.grid_map_center_x is not None else self.origin_x
        center_y = self.grid_map_center_y if self.grid_map_center_y is not None else self.origin_y
        yaw_cos = cos(self.grid_map_yaw)
        yaw_sin = sin(self.grid_map_yaw)
        x = center_x + yaw_cos * local_x - yaw_sin * local_y
        y = center_y + yaw_sin * local_x + yaw_cos * local_y
        return x, y


def distance_to_mask(
    mask: np.ndarray,
    resolution: float,
    include_grid_exterior: bool = False,
) -> np.ndarray:
    """计算精确欧氏距离场, 可将局部地图外部视为目标区域"""
    if include_grid_exterior:
        padded_mask = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=bool)
        padded_mask[0, :] = True
        padded_mask[-1, :] = True
        padded_mask[:, 0] = True
        padded_mask[:, -1] = True
        padded_mask[1:-1, 1:-1] = mask
        return distance_to_mask(padded_mask, resolution)[1:-1, 1:-1]

    height, width = mask.shape
    if not np.any(mask):
        max_distance = hypot(width * resolution, height * resolution)
        return np.full((height, width), max_distance, dtype=np.float32)

    distances = distance_transform_edt(~mask, sampling=float(resolution))
    return distances.astype(np.float32, copy=False)


def _clip_float_segment_to_bounds(
    start: Tuple[float, float],
    end: Tuple[float, float],
    width: int,
    height: int,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """把连续 grid 线段裁剪到当前 map bounds"""
    if width <= 0 or height <= 0:
        return None

    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    max_x = float(np.nextafter(float(width), -np.inf))
    max_y = float(np.nextafter(float(height), -np.inf))
    t_min = 0.0
    t_max = 1.0

    for p, q in (
        (-dx, x0),
        (dx, max_x - x0),
        (-dy, y0),
        (dy, max_y - y0),
    ):
        if abs(p) < 1e-12:
            if q < 0.0:
                return None
            continue
        ratio = q / p
        if p < 0.0:
            if ratio > t_max:
                return None
            t_min = max(t_min, ratio)
        else:
            if ratio < t_min:
                return None
            t_max = min(t_max, ratio)

    return (
        (x0 + t_min * dx, y0 + t_min * dy),
        (x0 + t_max * dx, y0 + t_max * dy),
    )


def _clamp_cell_index(value: float, size: int) -> int:
    """把连续 grid 坐标收敛到合法 cell index"""
    return min(max(int(floor(value)), 0), max(0, size - 1))


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> Iterable[GridIndex]:
    """生成 Bresenham 线经过的 grid cell"""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x += sx
        if err2 < dx:
            err += dx
            y += sy
