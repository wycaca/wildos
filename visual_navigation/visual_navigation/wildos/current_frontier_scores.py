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
