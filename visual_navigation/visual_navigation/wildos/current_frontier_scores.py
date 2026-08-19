"""Current-frame Frontier score aggregation independent from ROS"""

from collections.abc import Mapping

import numpy as np


class CurrentFrontierScores:
    """Aggregate only Frontier scores produced in the current processing frame"""

    def __init__(self, previous_entries: Mapping):
        self.previous_uuids = set(previous_entries)
        self.entries = {}
        self.visually_scored_uuids = set()

    def add_visual(self, uuid: str, scores, node) -> None:
        """Merge simultaneous camera evidence without retaining older frames"""
        current_scores = np.asarray(scores, dtype=np.float32)
        if uuid in self.entries:
            current_scores = np.maximum(self.entries[uuid][0], current_scores)
        self.entries[uuid] = (current_scores, node)
        self.visually_scored_uuids.add(uuid)

    def add_default(self, uuid: str, scores, node) -> None:
        """Add a geometric fallback only when no camera scored this Frontier"""
        if uuid not in self.entries:
            self.entries[uuid] = (np.asarray(scores, dtype=np.float32), node)

    def has(self, uuid: str) -> bool:
        return uuid in self.entries

    def scores(self, uuid: str) -> np.ndarray:
        return self.entries[uuid][0]

    def set_scores(self, uuid: str, scores) -> None:
        self.entries[uuid] = (np.asarray(scores, dtype=np.float32), self.entries[uuid][1])

    def is_visual(self, uuid: str) -> bool:
        return uuid in self.visually_scored_uuids

    def finish(self):
        """Return current entries and UUID changes for marker cleanup"""
        updated_uuids = set(self.entries)
        removed_uuids = sorted(self.previous_uuids - updated_uuids)
        return self.entries, removed_uuids, updated_uuids

    def snapshot(self) -> dict:
        """Copy the score values that affect the published graph"""
        return {
            uuid: (scores.copy(), self.is_visual(uuid))
            for uuid, (scores, _) in self.entries.items()
        }


def navigation_graph_content_equal(lhs, rhs) -> bool:
    """Compare graph content while ignoring the heartbeat header"""
    if lhs is None or rhs is None:
        return False
    return (
        lhs.trav_classes == rhs.trav_classes
        and lhs.current_node_idx == rhs.current_node_idx
        and lhs.nodes == rhs.nodes
        and lhs.edges == rhs.edges
    )


def frontier_score_snapshots_equal(lhs, rhs, epsilon: float) -> bool:
    """Compare only score changes large enough to affect planning"""
    if lhs.keys() != rhs.keys():
        return False
    tolerance = max(float(epsilon), 0.0)
    for uuid, (lhs_scores, lhs_visual) in lhs.items():
        rhs_scores, rhs_visual = rhs[uuid]
        if lhs_visual != rhs_visual or lhs_scores.shape != rhs_scores.shape:
            return False
        if not np.allclose(lhs_scores, rhs_scores, rtol=0.0, atol=tolerance):
            return False
    return True


def scored_graph_publish_due(
    content_changed: bool,
    last_publish_time: float | None,
    now: float,
    heartbeat_sec: float,
) -> bool:
    """Publish changes immediately and unchanged content as a heartbeat"""
    return (
        content_changed
        or last_publish_time is None
        or now - last_publish_time >= max(float(heartbeat_sec), 0.0)
    )
