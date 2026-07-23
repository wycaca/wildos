from visual_navigation.object_detection_confirmation import DetectionConfirmationWindow


def test_two_of_three_frames_allows_single_missing_frame():
    """两帧证据之间单帧丢失时仍应在第三帧完成确认"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert not window.update(True)
    assert not window.update(False)
    assert window.update(True)
    assert window.ready
    assert window.evidence_count == 2


def test_current_missing_frame_never_reports_detection():
    """历史窗口已达标时当前丢帧也不能发布旧 mask"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert not window.update(True)
    assert window.update(True)
    assert not window.update(False)
    assert window.ready


def test_sustained_missing_frames_expire_confirmation():
    """持续丢失证据后确认状态必须自动失效"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    window.update(True)
    window.update(True)
    window.update(False)
    window.update(False)
    window.update(False)

    assert not window.ready
    assert window.evidence_count == 0


def test_window_is_never_smaller_than_required_frames():
    """错误配置不能让确认门槛永远无法达到"""
    window = DetectionConfirmationWindow(required_frames=3, window_frames=2)

    assert window.window_frames == 3
    assert not window.update(True)
    assert not window.update(True)
    assert window.update(True)


def _update_score(
    window: DetectionConfirmationWindow,
    score: float,
) -> bool:
    """按当前阶段门槛把视觉分数写入确认窗口"""
    threshold = window.active_threshold(0.120, 0.110)
    return window.update(score >= threshold)


def test_peak_threshold_drops_only_after_entry_evidence():
    """0.110 只能确认已有候选, 不能单独创建候选"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert window.active_threshold(0.120, 0.110) == 0.120
    assert not window.update(True)
    assert window.active_threshold(0.120, 0.110) == 0.110

    window.update(False)
    window.update(False)
    window.update(False)
    assert window.active_threshold(0.120, 0.110) == 0.120


def test_entry_then_lower_confirmation_score_forms_mask():
    """0.120 首帧后允许 0.110 后续帧完成确认"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert not _update_score(window, 0.120)
    assert _update_score(window, 0.110)


def test_score_below_entry_threshold_cannot_start_candidate():
    """低于 0.120 的首帧不能创建视觉候选"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert not _update_score(window, 0.119)
    assert window.evidence_count == 0
    assert window.active_threshold(0.120, 0.110) == 0.120


def test_scores_below_confirmation_threshold_cannot_form_mask():
    """已有首帧后持续低于 0.110 仍不能形成 Mask"""
    window = DetectionConfirmationWindow(required_frames=2, window_frames=3)

    assert not _update_score(window, 0.120)
    assert not _update_score(window, 0.109)
    assert not _update_score(window, 0.109)
    assert not _update_score(window, 0.109)
    assert not window.ready
