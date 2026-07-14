import numpy as np

from visual_navigation.wildos.current_frontier_scores import CurrentFrontierScores


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
