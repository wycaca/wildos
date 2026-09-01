import numpy as np
import torch
from types import SimpleNamespace
from torchvision import transforms

from graphnav_msgs.msg import NavigationGraph, Node, NodeTraversabilityProperties

from visual_navigation.utils.performance_stats import TimingSummary, TimingWindow
from visual_navigation.utils.wildos_input_cache import WildOSInputCacheSnapshot
from visual_navigation.wildos.current_frontier_scores import (
    CurrentFrontierScores,
    frontier_score_snapshots_equal,
    navigation_graph_content_equal,
    scored_graph_publish_due,
)
from visual_navigation.wildos.nav import (
    WildOS_Nav,
    _rgb_images_to_tensor,
    _wildos_diagnostic_metrics,
)


def test_rgb_batch_conversion_matches_previous_pipeline_exactly():
    image = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    images = [image, np.flip(image, axis=0), np.flip(image, axis=1)]
    convert = transforms.ToTensor()

    previous = torch.stack([convert(item.copy()) for item in images])
    optimized = _rgb_images_to_tensor(images)

    assert torch.equal(optimized, previous)


def test_drops_entries_not_observed_in_current_frame():
    frame = CurrentFrontierScores({"old": (np.array([0.9]), object())})

    entries, removed, updated = frame.finish()

    assert entries == {}
    assert removed == ["old"]
    assert updated == set()


def test_merges_only_current_multi_camera_evidence():
    node = object()
    frame = CurrentFrontierScores({"frontier": (np.array([1.0, 1.0]), node)})
    frame.add_visual("frontier", [0.2, 0.8], node)
    frame.add_visual("frontier", [0.7, 0.3], node)

    entries, removed, updated = frame.finish()

    np.testing.assert_allclose(entries["frontier"][0], [0.7, 0.8])
    assert removed == []
    assert updated == {"frontier"}


def test_default_score_does_not_replace_current_visual_score():
    node = object()
    frame = CurrentFrontierScores({})
    frame.add_visual("frontier", [0.6, 0.4], node)
    frame.add_default("frontier", [0.1, 0.1], node)

    assert frame.is_visual("frontier")
    np.testing.assert_allclose(frame.scores("frontier"), [0.6, 0.4])


def test_score_snapshot_ignores_subthreshold_noise_but_tracks_score_source():
    previous = {"frontier": (np.array([0.5, 0.6]), True)}

    assert frontier_score_snapshots_equal(
        previous,
        {"frontier": (np.array([0.5005, 0.5995]), True)},
        epsilon=0.001,
    )
    assert not frontier_score_snapshots_equal(
        previous,
        {"frontier": (np.array([0.5, 0.6]), False)},
        epsilon=0.001,
    )
    assert not frontier_score_snapshots_equal(
        previous,
        {"frontier": (np.array([0.502, 0.6]), True)},
        epsilon=0.001,
    )


def test_navigation_graph_heartbeat_ignores_only_header():
    graph = SimpleNamespace(
        header=SimpleNamespace(stamp=1),
        trav_classes=["default"],
        current_node_idx=0,
        nodes=["node"],
        edges=["edge"],
    )
    heartbeat = SimpleNamespace(**vars(graph))
    heartbeat.header = SimpleNamespace(stamp=2)

    assert navigation_graph_content_equal(graph, heartbeat)
    heartbeat.current_node_idx = 1
    assert not navigation_graph_content_equal(graph, heartbeat)


def test_scored_graph_changes_publish_immediately_and_heartbeat_at_one_hz():
    assert scored_graph_publish_due(True, 10.0, 10.1, 1.0)
    assert not scored_graph_publish_due(False, 10.0, 10.999, 1.0)
    assert scored_graph_publish_due(False, 10.0, 11.0, 1.0)


def test_unchanged_scored_graph_reuses_cached_message_without_deepcopy():
    nav = object.__new__(WildOS_Nav)
    nav.frontier_uuid_to_scores = {}
    nav.traversability_class = "default"
    nav.num_cameras = 1
    nav.std_for_frontier_heading = None
    nav.std_for_default_scores = 30.0
    nav.default_max_score = 0.5
    nav.frontier_score_publish_epsilon = 0.001
    nav._last_scored_source_graph = None
    nav._last_scored_navgraph = None
    nav._last_scored_signature = {}
    nav._last_scored_graph_size_bytes = 0
    nav._processing_timings = {"graph_copy": TimingWindow()}
    nav.scorer = SimpleNamespace(
        get_default_scores=lambda *args, **kwargs: np.array([0.5, 0.4])
    )
    node = Node()
    node.uuid.id[0] = 1
    node.trav_properties = [NodeTraversabilityProperties(is_frontier=True)]
    graph = NavigationGraph(trav_classes=["default"], nodes=[node])

    first, _, _, first_changed = nav.update_navgraph_with_scores(
        graph,
        [None],
        [{}],
    )
    second, _, _, second_changed = nav.update_navgraph_with_scores(
        graph,
        [None],
        [{}],
    )

    assert first_changed
    assert not second_changed
    assert second is first


def test_wildos_diagnostics_keep_core_and_input_metrics_machine_readable():
    timing = TimingSummary(4, 10.0, 12.0, 15.0)

    metrics = _wildos_diagnostic_metrics(
        {"total": timing, "inference": timing},
        {"source_age": timing},
        {"odom_delta": timing},
        {"matched": 2.0},
        1.5,
        0.2,
        0.3,
        WildOSInputCacheSnapshot(3, 3, True),
        {"nav_graph_stale": 2},
        4,
        1,
        3,
        4096,
        {"gpu.memory.allocated_mib": 128.0},
    )

    assert metrics["cycle.total.p95_ms"] == 12.0
    assert metrics["stage.inference.average_ms"] == 10.0
    assert metrics["input.rate.matched_hz"] == 2.0
    assert metrics["input.reject.nav_graph_stale_count"] == 2
    assert metrics["output.scored_graph.size_bytes"] == 4096
