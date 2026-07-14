from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped


class ObjectSearchState:
    """目标搜索状态名集中定义, 避免不同模块拼写分叉"""

    WAIT_FOR_ODOM = "WAIT_FOR_ODOM"
    TARGET_MEMORY_GUIDED_SEARCH = "TARGET_MEMORY_GUIDED_SEARCH"
    TARGET_APPROACH = "TARGET_APPROACH"
    TARGET_REACHED_VIEWPOINT = "TARGET_REACHED_VIEWPOINT"
    SEARCHING_WITH_INITIAL_GOAL = "SEARCHING_WITH_INITIAL_GOAL"


@dataclass
class TargetHypothesis:
    """目标证据快照, 当前可先表示目标相关 frontier"""

    pose: PoseStamped
    last_seen_sec: float
    confidence: float = 1.0
    source: str = "object_related_frontier"
    is_metric_pose: bool = False

    def age(self, now_sec: float) -> float:
        return now_sec - self.last_seen_sec

    def is_active(self, now_sec: float, timeout_sec: float) -> bool:
        return self.age(now_sec) <= timeout_sec
