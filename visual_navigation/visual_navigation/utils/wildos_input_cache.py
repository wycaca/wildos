from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Any


@dataclass(frozen=True)
class MatchedWildOSInputs:
    """保存相机测量时刻对应的非视觉输入"""

    odom: Any
    nav_graph: Any
    camera_infos: tuple[Any, ...]
    odom_delta_seconds: float
    nav_graph_age_seconds: float


@dataclass(frozen=True)
class WildOSInputCacheSnapshot:
    """保存低频诊断需要的缓存状态"""

    odom_count: int
    camera_info_count: int
    has_nav_graph: bool


class WildOSInputCache:
    """独立缓存 CameraInfo、odom 和最新有效导航图

    相机同步完成后再执行时间匹配, 避免低频导航图阻塞三相机同步
    """

    def __init__(
        self,
        num_cameras: int,
        odom_cache_size: int,
        odom_max_delta_seconds: float,
        nav_graph_max_age_seconds: float,
        nav_graph_future_tolerance_seconds: float = 0.1,
    ) -> None:
        self.num_cameras = max(1, int(num_cameras))
        self.odom_max_delta_seconds = max(
            0.0,
            float(odom_max_delta_seconds),
        )
        self.nav_graph_max_age_seconds = max(
            0.0,
            float(nav_graph_max_age_seconds),
        )
        self.nav_graph_future_tolerance_seconds = max(
            0.0,
            float(nav_graph_future_tolerance_seconds),
        )
        self._odom_messages: deque[Any] = deque(
            maxlen=max(1, int(odom_cache_size))
        )
        self._camera_infos: dict[int, Any] = {}
        self._latest_nav_graph: Any | None = None
        self._latest_nav_graph_stamp = -1
        self._lock = Lock()

    def add_camera_info(self, camera_idx: int, msg: Any) -> None:
        """缓存每路相机最新内参"""
        if camera_idx < 0 or camera_idx >= self.num_cameras:
            return
        with self._lock:
            self._camera_infos[camera_idx] = msg

    def add_odom(self, msg: Any) -> None:
        """缓存短时间窗口内的 odom"""
        with self._lock:
            self._odom_messages.append(msg)

    def update_nav_graph(self, msg: Any) -> bool:
        """只接受时间更新且 current node 有效的导航图"""
        if not _nav_graph_is_valid(msg):
            return False
        stamp = message_stamp_nanoseconds(msg)
        with self._lock:
            if stamp < self._latest_nav_graph_stamp:
                return False
            self._latest_nav_graph = msg
            self._latest_nav_graph_stamp = stamp
        return True

    def match(self, measurement_stamp: Any) -> tuple[MatchedWildOSInputs | None, str]:
        """为相机测量选择最近 odom 和最新有效导航图"""
        measurement_ns = stamp_nanoseconds(measurement_stamp)
        with self._lock:
            odom_messages = tuple(self._odom_messages)
            camera_infos = tuple(
                self._camera_infos.get(camera_idx)
                for camera_idx in range(self.num_cameras)
            )
            nav_graph = self._latest_nav_graph
            nav_graph_stamp = self._latest_nav_graph_stamp

        if any(camera_info is None for camera_info in camera_infos):
            return None, "camera_info_missing"
        if not odom_messages:
            return None, "odom_missing"
        if nav_graph is None:
            return None, "nav_graph_missing"

        odom = min(
            odom_messages,
            key=lambda msg: abs(message_stamp_nanoseconds(msg) - measurement_ns),
        )
        odom_delta = abs(message_stamp_nanoseconds(odom) - measurement_ns) * 1e-9
        if odom_delta > self.odom_max_delta_seconds:
            return None, "odom_too_far"

        nav_graph_age = (measurement_ns - nav_graph_stamp) * 1e-9
        if nav_graph_age > self.nav_graph_max_age_seconds:
            return None, "nav_graph_stale"
        if nav_graph_age < -self.nav_graph_future_tolerance_seconds:
            return None, "nav_graph_from_future"

        return (
            MatchedWildOSInputs(
                odom=odom,
                nav_graph=nav_graph,
                camera_infos=camera_infos,
                odom_delta_seconds=odom_delta,
                nav_graph_age_seconds=max(0.0, nav_graph_age),
            ),
            "ok",
        )

    def snapshot(self) -> WildOSInputCacheSnapshot:
        """返回线程安全的缓存统计"""
        with self._lock:
            return WildOSInputCacheSnapshot(
                odom_count=len(self._odom_messages),
                camera_info_count=len(self._camera_infos),
                has_nav_graph=self._latest_nav_graph is not None,
            )


def stamp_nanoseconds(stamp: Any) -> int:
    """把 ROS Time 风格对象转换为纳秒"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def message_stamp_nanoseconds(msg: Any) -> int:
    """返回带 Header 消息的时间戳纳秒值"""
    return stamp_nanoseconds(msg.header.stamp)


def _nav_graph_is_valid(msg: Any) -> bool:
    """检查导航图是否包含合法 current node"""
    nodes = getattr(msg, "nodes", ())
    current_node_idx = int(getattr(msg, "current_node_idx", len(nodes)))
    return bool(nodes) and 0 <= current_node_idx < len(nodes)
