from visual_navigation.object_search_goal_policy import (
    GoalMode,
    GoalSelectionInput,
    ObjectSearchGoalPolicy,
)


def _select(**changes):
    values = {
        "reached_latched": False,
        "reached_hold_available": False,
        "odom_available": True,
        "completion_ready": False,
        "metric_target_available": False,
        "metric_target_stable": False,
        "metric_target_age_sec": 0.0,
        "coarse_target_timeout_sec": 3.0,
    }
    values.update(changes)
    return ObjectSearchGoalPolicy.select(GoalSelectionInput(**values))


def test_completion_has_priority_over_target_following():
    assert _select(
        completion_ready=True,
        metric_target_available=True,
        metric_target_stable=True,
    ) == GoalMode.ACTIVATE_REACHED


def test_reached_latch_is_terminal_even_without_odom():
    assert _select(
        reached_latched=True,
        odom_available=False,
    ) == GoalMode.WAIT_FOR_ODOM
    assert _select(
        reached_latched=True,
        reached_hold_available=True,
        odom_available=False,
    ) == GoalMode.HOLD_REACHED


def test_target_quality_and_age_select_the_expected_branch():
    assert _select(
        metric_target_available=True,
        metric_target_stable=True,
    ) == GoalMode.FOLLOW_STABLE_TARGET
    assert _select(
        metric_target_available=True,
    ) == GoalMode.FOLLOW_COARSE_TARGET
    assert _select(
        metric_target_available=True,
        metric_target_age_sec=3.1,
    ) == GoalMode.EXPIRE_COARSE_TARGET


def test_missing_odom_waits_and_ready_inputs_continue_search():
    assert _select(odom_available=False) == GoalMode.WAIT_FOR_ODOM
    assert _select() == GoalMode.CONTINUE_SEARCH
