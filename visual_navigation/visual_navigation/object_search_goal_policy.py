from dataclasses import dataclass
from enum import Enum, auto
import math


class GoalMode(Enum):
    """High-level action selected before ROS messages are built"""

    WAIT_FOR_ODOM = auto()
    ACTIVATE_REACHED = auto()
    HOLD_REACHED = auto()
    EXPIRE_COARSE_TARGET = auto()
    FOLLOW_COARSE_TARGET = auto()
    FOLLOW_STABLE_TARGET = auto()
    CONTINUE_SEARCH = auto()


@dataclass(frozen=True)
class GoalSelectionInput:
    reached_latched: bool
    reached_hold_available: bool
    odom_available: bool
    completion_ready: bool
    metric_target_available: bool
    metric_target_stable: bool
    metric_target_age_sec: float = math.inf
    coarse_target_timeout_sec: float = 0.0


class ObjectSearchGoalPolicy:
    """Pure priority policy for completion, target following, and exploration"""

    @staticmethod
    def select(inputs: GoalSelectionInput) -> GoalMode:
        if inputs.reached_latched:
            if not inputs.reached_hold_available and not inputs.odom_available:
                return GoalMode.WAIT_FOR_ODOM
            return GoalMode.HOLD_REACHED
        if inputs.odom_available and inputs.completion_ready:
            return GoalMode.ACTIVATE_REACHED
        if inputs.metric_target_available:
            if (
                not inputs.metric_target_stable
                and inputs.metric_target_age_sec
                > inputs.coarse_target_timeout_sec
            ):
                return GoalMode.EXPIRE_COARSE_TARGET
            if inputs.metric_target_stable:
                return GoalMode.FOLLOW_STABLE_TARGET
            return GoalMode.FOLLOW_COARSE_TARGET
        if not inputs.odom_available:
            return GoalMode.WAIT_FOR_ODOM
        return GoalMode.CONTINUE_SEARCH
