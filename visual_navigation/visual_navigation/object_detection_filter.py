from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectionComponent:
    """通过目标检测门槛的单相机连通区域统计"""

    camera_idx: int
    pixel_count: int
    pixel_fraction: float
    peak_score: float


def filter_object_detection_mask(
    similarity_map: np.ndarray,
    binary_mask: np.ndarray,
    min_peak_score: float,
    min_component_pixels: int,
    min_component_fraction: float,
) -> tuple[np.ndarray, list[DetectionComponent]]:
    """过滤弱响应和零散响应, 每路相机只保留最可信的目标区域

    输入形状必须是 B,Q,H,W, 当前目标搜索只支持单文本 query
    连通区域需要同时满足峰值, 像素数和图像占比门槛
    每路相机只保留排序最优的区域, 避免多个背景响应让检测射线质心偏移
    """
    scores = np.asarray(similarity_map)
    masks = np.asarray(binary_mask)
    if scores.shape != masks.shape or scores.ndim != 4:
        raise ValueError("similarity_map and binary_mask must share B,Q,H,W shape")
    if scores.shape[1] != 1:
        raise ValueError("object detection filtering requires exactly one text query")

    filtered_mask = np.zeros_like(masks, dtype=np.uint8)
    accepted_components: list[DetectionComponent] = []
    image_area = max(int(masks.shape[-2] * masks.shape[-1]), 1)
    required_pixels = max(
        int(min_component_pixels),
        int(np.ceil(max(float(min_component_fraction), 0.0) * image_area)),
    )

    for camera_idx in range(masks.shape[0]):
        camera_mask = masks[camera_idx, 0].astype(np.uint8)
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            camera_mask,
            connectivity=8,
        )
        best_component = None
        best_rank = None

        for label in range(1, component_count):
            pixel_count = int(stats[label, cv2.CC_STAT_AREA])
            if pixel_count < required_pixels:
                continue

            component_pixels = labels == label
            peak_score = float(np.max(scores[camera_idx, 0][component_pixels]))
            if not np.isfinite(peak_score) or peak_score < float(min_peak_score):
                continue

            rank = (peak_score, pixel_count)
            if best_rank is None or rank > best_rank:
                best_rank = rank
                best_component = (label, pixel_count, peak_score)

        if best_component is None:
            continue

        label, pixel_count, peak_score = best_component
        filtered_mask[camera_idx, 0][labels == label] = 1
        accepted_components.append(
            DetectionComponent(
                camera_idx=camera_idx,
                pixel_count=pixel_count,
                pixel_fraction=pixel_count / image_area,
                peak_score=peak_score,
            )
        )

    return filtered_mask, accepted_components
