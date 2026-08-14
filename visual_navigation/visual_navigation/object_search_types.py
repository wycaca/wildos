class ObjectSearchState:
    """目标搜索状态名集中定义, 避免不同模块拼写分叉"""

    WAIT_FOR_ODOM = "WAIT_FOR_ODOM"
    STARTUP_OBSERVATION = "STARTUP_OBSERVATION"
    TARGET_PENDING_OBSERVATION = "TARGET_PENDING_OBSERVATION"
    TARGET_PENDING_REPOSITION = "TARGET_PENDING_REPOSITION"
    TARGET_APPROACH_COARSE = "TARGET_APPROACH_COARSE"
    TARGET_OBSERVATION = "TARGET_OBSERVATION"
    TARGET_APPROACH_METRIC = "TARGET_APPROACH_METRIC"
    TARGET_FINAL_OBSERVATION = "TARGET_FINAL_OBSERVATION"
    TARGET_FINAL_REPOSITION = "TARGET_FINAL_REPOSITION"
    TARGET_REACHED_VIEWPOINT = "TARGET_REACHED_VIEWPOINT"
    SEARCHING_WITH_INITIAL_GOAL = "SEARCHING_WITH_INITIAL_GOAL"


def normalize_object_search_target(value: str) -> str | None:
    """压缩空白并拒绝空目标"""
    target = " ".join(str(value).split())
    return target or None


def coarse_target_evidence_ready(
    state: str,
    accepted_views: int,
    confidence: float,
    minimum_views: int = 2,
    minimum_confidence: float = 0.45,
) -> bool:
    """统一判断黄色粗目标是否已具备导航接管证据"""
    return (
        state in {"TRACKING", "STABLE_VISION", "LIDAR_LOCKED"}
        and int(accepted_views) >= int(minimum_views)
        and float(confidence) >= float(minimum_confidence)
    )
