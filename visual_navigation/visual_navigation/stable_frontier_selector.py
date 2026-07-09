import copy
import math
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import PoseStamped, Quaternion
from graphnav_msgs.msg import NavigationGraph


@dataclass
class StableFrontierSelectorConfig:
    traversability_class: str = "default"
    frame_id: str = "map"
    min_dwell_sec: float = 8.0
    switch_min_score_margin: float = 0.15
    progress_timeout_sec: float = 12.0
    progress_min_delta: float = 0.25
    reached_radius: float = 1.5
    same_position_radius: float = 1.2
    deadend_blacklist_timeout_sec: float = 20.0
    score_weight: float = 1.0
    distance_weight: float = 0.06
    switch_penalty: float = 0.2
    forward_weight: float = 0.8
    min_forward_dot: float = 0.0
    forward_fallback_to_any: bool = True


@dataclass
class FrontierCandidate:
    uuid: str
    pose: PoseStamped
    score: float
    distance: float
    utility: float
    source: str
    heading_alignment: float = 0.0


class StableFrontierSelector:
    """在 scored graph 中锁定一个稳定 frontier, 避免每帧重选"""

    def __init__(
        self,
        config: StableFrontierSelectorConfig,
        logger: Any | None = None,
    ) -> None:
        self.config = config
        self.logger = logger
        self.selected: FrontierCandidate | None = None
        self.selected_time_sec: float | None = None
        self.selected_best_distance = math.inf
        self.selected_last_progress_sec: float | None = None
        self.blacklisted_frontiers: dict[str, float] = {}

    @property
    def blacklisted_count(self) -> int:
        return len(self.blacklisted_frontiers)

    def clear(self) -> None:
        self.selected = None
        self.selected_time_sec = None
        self.selected_best_distance = math.inf
        self.selected_last_progress_sec = None

    def select(
        self,
        graph: NavigationGraph,
        reference_xy: tuple[float, float],
        now_sec: float,
        stamp,
        target_pose: PoseStamped | None = None,
        heading_yaw: float | None = None,
    ) -> FrontierCandidate | None:
        """从当前图中选择一个稳定 frontier, 只在明确更优或失败时切换"""
        if not graph.nodes:
            self.clear()
            return None

        self._expire_blacklisted_frontiers(now_sec)
        candidates = self.collect_candidates(
            graph,
            reference_xy,
            stamp,
            target_pose,
            heading_yaw,
        )
        if not candidates:
            self.clear()
            return None

        best = max(candidates, key=lambda item: item.utility)
        current = self._current_frontier_candidate(candidates)
        return self._choose_frontier_candidate(best, current, candidates, now_sec)

    def collect_candidates(
        self,
        graph: NavigationGraph,
        reference_xy: tuple[float, float],
        stamp,
        target_pose: PoseStamped | None = None,
        heading_yaw: float | None = None,
    ) -> list[FrontierCandidate]:
        """把 graph frontier node 转成 selector 候选, 不修改 graph 消息"""
        trav_idx = self._traversability_index(graph)
        if trav_idx is None:
            return []

        ref_x, ref_y = reference_xy
        candidates: list[FrontierCandidate] = []
        for node in graph.nodes:
            if (
                trav_idx >= len(node.trav_properties)
                or not node.trav_properties[trav_idx].is_frontier
            ):
                continue
            uuid = _uuid_to_str(node.uuid)
            if self._frontier_is_blacklisted(uuid):
                continue

            score = self._frontier_score(node, target_pose)
            dx = node.pose.position.x - ref_x
            dy = node.pose.position.y - ref_y
            distance = math.hypot(
                dx,
                dy,
            )
            heading_alignment = _heading_alignment(dx, dy, heading_yaw)
            utility = (
                self.config.score_weight * score
                - self.config.distance_weight * distance
                + self.config.forward_weight * heading_alignment
            )
            if self.selected is not None and uuid != self.selected.uuid:
                utility -= self.config.switch_penalty
            if self.selected is not None and uuid == self.selected.uuid:
                utility += self.config.switch_penalty

            goal = PoseStamped()
            goal.header = copy.deepcopy(graph.header)
            goal.header.frame_id = graph.header.frame_id or self.config.frame_id
            goal.header.stamp = stamp
            goal.pose = copy.deepcopy(node.pose)
            goal.pose.orientation = _yaw_to_quaternion(
                math.atan2(
                    node.pose.position.y - ref_y,
                    node.pose.position.x - ref_x,
                )
            )
            candidates.append(
                FrontierCandidate(
                    uuid=uuid,
                    pose=goal,
                    score=score,
                    distance=distance,
                    utility=utility,
                    source="graph_frontier",
                    heading_alignment=heading_alignment,
                )
            )
        return self._filter_forward_candidates(candidates, heading_yaw)

    def _filter_forward_candidates(
        self,
        candidates: list[FrontierCandidate],
        heading_yaw: float | None,
    ) -> list[FrontierCandidate]:
        """目标未知时优先保留 odom 朝向前方 frontier"""
        if heading_yaw is None:
            return candidates
        forward_candidates = [
            candidate
            for candidate in candidates
            if candidate.heading_alignment >= self.config.min_forward_dot
        ]
        if forward_candidates:
            return forward_candidates
        if self.config.forward_fallback_to_any:
            return candidates
        return []

    def _choose_frontier_candidate(
        self,
        best: FrontierCandidate,
        current: FrontierCandidate | None,
        candidates: list[FrontierCandidate],
        now_sec: float,
    ) -> FrontierCandidate | None:
        if current is None:
            self._start_selected_frontier(best, now_sec, "NEW_FRONTIER")
            return best

        self._update_frontier_progress(current, now_sec)
        if current.distance <= self.config.reached_radius:
            self._blacklist_frontier(current.uuid, now_sec)
            self.clear()
            next_best = self._best_unblocked_candidate(candidates, current.uuid)
            if next_best is None:
                return None
            self._start_selected_frontier(next_best, now_sec, "FRONTIER_REACHED")
            return next_best

        if self._selected_frontier_stalled(now_sec):
            self._blacklist_frontier(current.uuid, now_sec)
            self.clear()
            next_best = self._best_unblocked_candidate(candidates, current.uuid)
            if next_best is None:
                return None
            self._start_selected_frontier(
                next_best,
                now_sec,
                "FRONTIER_NO_PROGRESS",
            )
            return next_best

        dwell_age = self._age_seconds(now_sec, self.selected_time_sec)
        if dwell_age < self.config.min_dwell_sec:
            self.selected = current
            return current

        if best.uuid == current.uuid:
            self.selected = current
            return current

        if best.utility <= current.utility + self.config.switch_min_score_margin:
            self.selected = current
            return current

        self._start_selected_frontier(best, now_sec, "BETTER_FRONTIER")
        return best

    @staticmethod
    def _best_unblocked_candidate(
        candidates: list[FrontierCandidate],
        blocked_uuid: str,
    ) -> FrontierCandidate | None:
        remaining = [
            candidate for candidate in candidates
            if candidate.uuid != blocked_uuid
        ]
        if not remaining:
            return None
        return max(remaining, key=lambda item: item.utility)

    def _start_selected_frontier(
        self,
        candidate: FrontierCandidate,
        now_sec: float,
        reason: str,
    ) -> None:
        self.selected = candidate
        self.selected_time_sec = now_sec
        self.selected_best_distance = candidate.distance
        self.selected_last_progress_sec = now_sec
        if self.logger is not None:
            self.logger.info(
                f"选择搜索 frontier, reason={reason}, uuid={candidate.uuid}, "
                f"score={candidate.score:.2f}, distance={candidate.distance:.2f}, "
                f"front_dot={candidate.heading_alignment:.2f}, utility={candidate.utility:.2f}"
            )

    def _update_frontier_progress(
        self,
        candidate: FrontierCandidate,
        now_sec: float,
    ) -> None:
        if candidate.distance < (
            self.selected_best_distance - self.config.progress_min_delta
        ):
            self.selected_best_distance = candidate.distance
            self.selected_last_progress_sec = now_sec

    def _selected_frontier_stalled(self, now_sec: float) -> bool:
        if self.selected_last_progress_sec is None:
            return False
        if self._age_seconds(now_sec, self.selected_time_sec) < self.config.min_dwell_sec:
            return False
        return (
            self._age_seconds(now_sec, self.selected_last_progress_sec)
            > self.config.progress_timeout_sec
        )

    def _blacklist_frontier(self, uuid: str, now_sec: float) -> None:
        if not uuid or self.config.deadend_blacklist_timeout_sec <= 0.0:
            return
        self.blacklisted_frontiers[uuid] = now_sec
        if self.logger is not None:
            self.logger.warn(f"frontier 长时间无进展, 短时屏蔽 uuid={uuid}")

    def _expire_blacklisted_frontiers(self, now_sec: float) -> None:
        expired = [
            uuid
            for uuid, stamp_sec in self.blacklisted_frontiers.items()
            if now_sec - stamp_sec > self.config.deadend_blacklist_timeout_sec
        ]
        for uuid in expired:
            del self.blacklisted_frontiers[uuid]

    def _frontier_is_blacklisted(self, uuid: str) -> bool:
        return uuid in self.blacklisted_frontiers

    def _candidate_by_uuid(
        self,
        candidates: list[FrontierCandidate],
        uuid: str,
    ) -> FrontierCandidate | None:
        if not uuid:
            return None
        for candidate in candidates:
            if candidate.uuid == uuid:
                return candidate
        return None

    def _current_frontier_candidate(
        self,
        candidates: list[FrontierCandidate],
    ) -> FrontierCandidate | None:
        if self.selected is None:
            return None
        current = self._candidate_by_uuid(candidates, self.selected.uuid)
        if current is not None:
            return current

        prev = self.selected.pose.pose.position
        nearby = [
            candidate
            for candidate in candidates
            if math.hypot(
                candidate.pose.pose.position.x - prev.x,
                candidate.pose.pose.position.y - prev.y,
            )
            <= self.config.same_position_radius
        ]
        if not nearby:
            return None
        return max(nearby, key=lambda item: item.utility)

    def _traversability_index(self, graph: NavigationGraph) -> int | None:
        if self.config.traversability_class in graph.trav_classes:
            return graph.trav_classes.index(self.config.traversability_class)
        if graph.trav_classes:
            return 0
        return None

    @staticmethod
    def _frontier_score(node, target_pose: PoseStamped | None) -> float:
        scores = _node_float_property(node, "frontier_scores")
        if not scores:
            return 0.1
        if target_pose is None:
            return max(scores)
        dx = target_pose.pose.position.x - node.pose.position.x
        dy = target_pose.pose.position.y - node.pose.position.y
        if math.hypot(dx, dy) < 1e-6:
            return max(scores)
        angle = math.atan2(dy, dx)
        if angle < 0.0:
            angle += 2.0 * math.pi
        best_bin = int(round(angle / (2.0 * math.pi / len(scores)))) % len(scores)
        return scores[best_bin]

    @staticmethod
    def _age_seconds(now_sec: float, then_sec: float | None) -> float:
        if then_sec is None:
            return math.inf
        return now_sec - then_sec


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def _heading_alignment(dx: float, dy: float, heading_yaw: float | None) -> float:
    """计算候选方向和 odom 朝向的点积, 无 heading 时不加偏置"""
    if heading_yaw is None:
        return 0.0
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return 1.0
    heading_x = math.cos(heading_yaw)
    heading_y = math.sin(heading_yaw)
    return (dx * heading_x + dy * heading_y) / distance


def _uuid_to_str(uuid) -> str:
    return "".join(f"{part:03}" for part in uuid.id)


def _node_float_property(node, key: str) -> list[float]:
    for prop in node.properties:
        if prop.key != key:
            continue
        values = []
        for value in prop.value:
            number = float(value)
            if math.isfinite(number):
                values.append(number)
        return values
    return []
