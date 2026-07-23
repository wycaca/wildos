from types import SimpleNamespace

import pytest

from visual_navigation.utils.wildos_input_cache import WildOSInputCache


def _stamp(seconds: float):
    whole_seconds = int(seconds)
    return SimpleNamespace(
        sec=whole_seconds,
        nanosec=int(round((seconds - whole_seconds) * 1_000_000_000)),
    )


def _message(seconds: float, **values):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(seconds)),
        **values,
    )


def _valid_graph(seconds: float, marker: str = "graph"):
    return _message(
        seconds,
        nodes=[object(), object()],
        current_node_idx=0,
        marker=marker,
    )


def _ready_cache(**overrides) -> WildOSInputCache:
    config = {
        "num_cameras": 3,
        "odom_cache_size": 10,
        "odom_max_delta_seconds": 0.2,
        "nav_graph_max_age_seconds": 1.0,
    }
    config.update(overrides)
    cache = WildOSInputCache(**config)
    for camera_idx in range(3):
        cache.add_camera_info(camera_idx, _message(1.0, camera=camera_idx))
    return cache


def test_match_uses_latest_camera_info_nearest_odom_and_latest_graph():
    """相机组应使用缓存内参、最近 odom 和最新有效导航图"""
    cache = _ready_cache()
    cache.add_odom(_message(9.7, marker="older"))
    cache.add_odom(_message(10.04, marker="nearest"))
    cache.add_odom(_message(10.18, marker="later"))
    cache.update_nav_graph(_valid_graph(9.35))

    matched, reason = cache.match(_stamp(10.0))

    assert reason == "ok"
    assert matched is not None
    assert matched.odom.marker == "nearest"
    assert matched.nav_graph.marker == "graph"
    assert matched.odom_delta_seconds == pytest.approx(0.04)
    assert matched.nav_graph_age_seconds == pytest.approx(0.65)
    assert [info.camera for info in matched.camera_infos] == [0, 1, 2]


def test_stale_navigation_graph_rejects_only_current_camera_group():
    """超过年龄门槛的导航图不能和当前相机组匹配"""
    cache = _ready_cache(nav_graph_max_age_seconds=1.0)
    cache.add_odom(_message(10.0))
    cache.update_nav_graph(_valid_graph(8.9))

    matched, reason = cache.match(_stamp(10.0))

    assert matched is None
    assert reason == "nav_graph_stale"


def test_odom_outside_time_gate_is_rejected():
    """最近 odom 仍超过门槛时应等待后续相机组"""
    cache = _ready_cache(odom_max_delta_seconds=0.2)
    cache.add_odom(_message(9.7))
    cache.update_nav_graph(_valid_graph(9.5))

    matched, reason = cache.match(_stamp(10.0))

    assert matched is None
    assert reason == "odom_too_far"


def test_invalid_or_older_graph_does_not_replace_latest_valid_graph():
    """无效图和乱序旧图不能覆盖最新有效图"""
    cache = _ready_cache()
    cache.add_odom(_message(10.0))
    assert cache.update_nav_graph(_valid_graph(9.5, marker="latest"))
    assert not cache.update_nav_graph(
        _message(9.8, nodes=[], current_node_idx=0)
    )
    assert not cache.update_nav_graph(_valid_graph(9.4, marker="older"))

    matched, reason = cache.match(_stamp(10.0))

    assert reason == "ok"
    assert matched is not None
    assert matched.nav_graph.marker == "latest"


def test_all_camera_info_messages_are_required_once():
    """缺少任意一路内参时不能处理三相机图像"""
    cache = WildOSInputCache(
        num_cameras=3,
        odom_cache_size=10,
        odom_max_delta_seconds=0.2,
        nav_graph_max_age_seconds=1.0,
    )
    cache.add_camera_info(0, _message(1.0))
    cache.add_camera_info(1, _message(1.0))
    cache.add_odom(_message(10.0))
    cache.update_nav_graph(_valid_graph(9.5))

    matched, reason = cache.match(_stamp(10.0))

    assert matched is None
    assert reason == "camera_info_missing"
