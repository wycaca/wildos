import torch
from std_msgs.msg import String

from visual_navigation.object_detection_confirmation import DetectionConfirmationWindow
from visual_navigation.object_reached_evidence import VisualReachedEvidence
from visual_navigation.object_search_types import normalize_object_search_target
from visual_navigation.wildos.nav import WildOS_Nav


class _Logger:
    def __init__(self):
        self.warnings = []

    def info(self, message):
        pass

    def warn(self, message):
        self.warnings.append(message)


class _TextModel:
    def __init__(self):
        self.calls = []

    def forward_on_text(self, queries):
        self.calls.append(list(queries))
        return torch.tensor([[2.0]])


class _WildOSTargetHarness:
    _on_object_search_target = WildOS_Nav._on_object_search_target

    def __init__(self):
        self.model = _TextModel()
        self.text_queries = ["old target"]
        self.text_feats = torch.tensor([[1.0]])
        self.object_detection_confirmation = DetectionConfirmationWindow(2, 3)
        self.object_detection_confirmation.update(True)
        self.object_reached_evidence = VisualReachedEvidence(1, 0.0, 1)
        self.object_reached_evidence.confirm_count = 1
        self._object_detection_ready = True
        self._visual_reached_active = True
        self._object_missing_log_count = 5
        self.object_search_completed = True
        self.frontier_uuid_to_scores = {"old": [1.0]}
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def test_target_normalization_rejects_empty_and_collapses_whitespace():
    assert normalize_object_search_target("  red   bucket \n") == "red bucket"
    assert normalize_object_search_target("  \t ") is None


def test_wildos_target_change_rebuilds_text_features_and_resets_old_state():
    node = _WildOSTargetHarness()

    node._on_object_search_target(String(data="new target"))

    assert node.model.calls == [["new target"]]
    assert node.text_queries == ["new target"]
    assert torch.equal(node.text_feats, torch.tensor([[2.0]]))
    assert not node.object_detection_confirmation.ready
    assert node.object_reached_evidence.confirm_count == 0
    assert not node._object_detection_ready
    assert not node._visual_reached_active
    assert node._object_missing_log_count == 0
    assert not node.object_search_completed
    assert node.frontier_uuid_to_scores == {}


def test_empty_and_duplicate_targets_do_not_rebuild_text_features():
    node = _WildOSTargetHarness()

    node._on_object_search_target(String(data="  "))
    node._on_object_search_target(String(data="old   target"))

    assert node.model.calls == []
    assert len(node.logger.warnings) == 1
