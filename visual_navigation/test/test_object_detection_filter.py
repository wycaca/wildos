import numpy as np
import pytest

from visual_navigation.object_detection_filter import filter_object_detection_mask


def test_filter_rejects_single_pixel_false_positive():
    scores = np.zeros((3, 1, 20, 20), dtype=np.float32)
    scores[2, 0, 4, 5] = 0.3
    mask = (scores > 0.09).astype(np.uint8)

    filtered, components = filter_object_detection_mask(scores, mask, 0.12, 8, 0.0)

    assert not np.any(filtered)
    assert components == []


def test_filter_rejects_large_low_confidence_response():
    scores = np.zeros((3, 1, 20, 20), dtype=np.float32)
    scores[1, 0, 2:8, 3:9] = 0.1
    mask = (scores > 0.09).astype(np.uint8)

    filtered, components = filter_object_detection_mask(scores, mask, 0.12, 8, 0.0)

    assert not np.any(filtered)
    assert components == []


def test_filter_keeps_only_best_component_per_camera():
    scores = np.zeros((3, 1, 20, 20), dtype=np.float32)
    scores[2, 0, 2:6, 2:6] = 0.14
    scores[2, 0, 10:15, 10:15] = 0.2
    mask = (scores > 0.09).astype(np.uint8)

    filtered, components = filter_object_detection_mask(scores, mask, 0.12, 8, 0.0)

    assert int(np.sum(filtered)) == 25
    assert len(components) == 1
    assert components[0].camera_idx == 2
    assert components[0].pixel_count == 25
    assert components[0].peak_score == pytest.approx(0.2)


def test_filter_applies_fraction_as_minimum_component_size():
    scores = np.zeros((1, 1, 20, 20), dtype=np.float32)
    scores[0, 0, 2:6, 2:6] = 0.2
    mask = (scores > 0.09).astype(np.uint8)

    filtered, components = filter_object_detection_mask(scores, mask, 0.12, 1, 0.05)

    assert not np.any(filtered)
    assert components == []
