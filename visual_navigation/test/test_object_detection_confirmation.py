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
