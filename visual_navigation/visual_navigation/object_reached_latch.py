import numpy as np


class ObjectReachedLatch:
    """连续近距离证据确认后永久锁定任务完成状态"""

    def __init__(self, min_pixel_count: int, min_mask_fraction: float, confirm_frames: int):
        self.min_pixel_count = max(int(min_pixel_count), 1)
        self.min_mask_fraction = max(float(min_mask_fraction), 0.0)
        self.confirm_frames = max(int(confirm_frames), 1)
        self.confirm_count = 0
        self.completed = False

    def update(self, binary_mask) -> bool:
        """未完成时累计连续证据, 完成后忽略后续空帧并始终返回 True"""
        if self.completed:
            return True
        if binary_mask is None:
            self.confirm_count = 0
            return False

        masks = np.asarray(binary_mask)
        if masks.ndim != 4 or masks.shape[1] == 0:
            self.confirm_count = 0
            return False
        masks = masks[:, 0]
        image_area = max(float(masks.shape[-2] * masks.shape[-1]), 1.0)
        max_pixel_count = int(np.max(np.sum(masks > 0, axis=(-2, -1))))
        max_fraction = max_pixel_count / image_area
        reached_candidate = (
            max_pixel_count >= self.min_pixel_count
            and max_fraction >= self.min_mask_fraction
        )
        self.confirm_count = self.confirm_count + 1 if reached_candidate else 0
        if self.confirm_count >= self.confirm_frames:
            self.completed = True
        return self.completed
