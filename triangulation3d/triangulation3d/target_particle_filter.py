from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


class FusionState:
    EMPTY = "EMPTY"
    PENDING = "PENDING"
    TRACKING = "TRACKING"
    STABLE_VISION = "STABLE_VISION"
    LIDAR_LOCKED = "LIDAR_LOCKED"
    REACHED = "REACHED"


class EstimateSource:
    NONE = 0
    VISION = 1
    FUSED = 2
    LIDAR = 3


@dataclass(frozen=True)
class ParticleFilterConfig:
    particle_count: int = 1500
    min_depth: float = 1.0
    max_depth: float = 100.0
    max_observations: int = 12
    independent_view_translation: float = 0.12
    duplicate_view_translation: float = 0.03
    duplicate_view_angle_deg: float = 0.5
    full_quality_translation: float = 0.3
    full_quality_angle_deg: float = 3.0
    resample_ess_ratio: float = 0.5
    resample_jitter: float = 0.25
    injected_particle_ratio: float = 0.2
    stable_min_views: int = 2
    stable_max_horizontal_std: float = 4.0
    stable_min_confidence: float = 0.6
    lidar_sigma: float = 0.8
    lidar_consistency_radius: float = 1.5
    lidar_lock_frames: int = 2
    association_min_radius: float = 2.0
    association_sigma_factor: float = 2.0


@dataclass(frozen=True)
class CameraObservation:
    mask: np.ndarray
    intrinsic: np.ndarray
    rotation_world_from_camera: np.ndarray
    translation_world_from_camera: np.ndarray
    confidence: float = 1.0
    camera_id: str = ""

    def __post_init__(self):
        mask = np.asarray(self.mask, dtype=bool)
        intrinsic = np.asarray(self.intrinsic, dtype=np.float64)
        rotation = np.asarray(self.rotation_world_from_camera, dtype=np.float64)
        translation = np.asarray(self.translation_world_from_camera, dtype=np.float64).reshape(3)
        if mask.ndim != 2 or not np.any(mask):
            raise ValueError("camera observation requires a non-empty 2D mask")
        if intrinsic.shape != (3, 3) or rotation.shape != (3, 3):
            raise ValueError("camera observation requires 3x3 intrinsic and rotation matrices")
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "intrinsic", intrinsic)
        object.__setattr__(self, "rotation_world_from_camera", rotation)
        object.__setattr__(self, "translation_world_from_camera", translation)
        object.__setattr__(self, "confidence", float(np.clip(self.confidence, 0.0, 1.0)))

    def center_ray(self) -> tuple[np.ndarray, np.ndarray, float]:
        """计算 mask 质心射线和覆盖该 mask 的近似角半径"""
        rows, cols = np.nonzero(self.mask)
        center = np.array([np.mean(cols), np.mean(rows), 1.0], dtype=np.float64)
        ray_camera = np.linalg.inv(self.intrinsic) @ center
        ray_camera /= np.linalg.norm(ray_camera)
        ray_world = self.rotation_world_from_camera @ ray_camera
        ray_world /= np.linalg.norm(ray_world)

        width = max(float(np.max(cols) - np.min(cols) + 1), 1.0)
        height = max(float(np.max(rows) - np.min(rows) + 1), 1.0)
        focal = max(float(self.intrinsic[0, 0] + self.intrinsic[1, 1]) * 0.5, 1.0)
        angular_radius = max(math.atan2(0.5 * max(width, height), focal), math.radians(0.5))
        return self.translation_world_from_camera, ray_world, angular_radius


@dataclass(frozen=True)
class _AcceptedView:
    """保存观测质量和是否形成独立视差"""

    observation: CameraObservation
    quality: float
    independent: bool


@dataclass(frozen=True)
class TargetEstimate:
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    source: int
    stable: bool
    accepted_views: int
    lidar_support: int
    state: str


class TargetParticleFilter:
    """融合多个相机观测和可选 LiDAR 测量的单目标粒子滤波器"""

    def __init__(self, config: ParticleFilterConfig = ParticleFilterConfig(), seed: int = 42):
        self.config = config
        self.rng = np.random.default_rng(seed)
        self.particles: np.ndarray | None = None
        self.weights: np.ndarray | None = None
        self.observations: deque[_AcceptedView] = deque(maxlen=config.max_observations)
        self.accepted_views = 0
        self.view_support = 0.0
        self.weak_view_updates = 0
        self.duplicate_views_rejected = 0
        self.lidar_support = 0
        self.lidar_consistent_frames = 0
        self.lidar_association_rejected = 0
        self.last_lidar_position: np.ndarray | None = None
        self.reached_estimate_stable = False
        self.state = FusionState.EMPTY

    @property
    def completed(self) -> bool:
        return self.state == FusionState.REACHED

    def mark_reached(self) -> None:
        self.reached_estimate_stable = self.state in {
            FusionState.STABLE_VISION,
            FusionState.LIDAR_LOCKED,
        }
        self.state = FusionState.REACHED

    def update_vision(self, observations: list[CameraObservation]) -> TargetEstimate | None:
        """过滤重复帧, 让弱视角按质量参与粒子更新"""
        if self.completed:
            return self.estimate()

        accepted: list[_AcceptedView] = []
        for observation in observations:
            if not self._is_associated_with_track(observation):
                continue
            accepted_view = self._assess_view(observation, accepted)
            if accepted_view is not None:
                accepted.append(accepted_view)
        if not accepted:
            return self.estimate()

        self.observations.extend(accepted)
        independent_views = [view for view in accepted if view.independent]
        self.accepted_views += len(independent_views)
        self.view_support += sum(view.quality for view in independent_views)
        self.weak_view_updates += len(accepted) - len(independent_views)
        accepted_observations = [view.observation for view in accepted]

        if self.particles is None:
            self.particles = self._sample_from_observations(
                accepted_observations,
                self.config.particle_count,
            )
            self.weights = np.full(self.config.particle_count, 1.0 / self.config.particle_count)
        else:
            inject_count = max(
                1,
                int(round(self.config.particle_count * self.config.injected_particle_ratio)),
            )
            replace_indices = self.rng.choice(
                self.config.particle_count,
                inject_count,
                replace=False,
            )
            self.particles[replace_indices] = self._sample_from_observations(
                accepted_observations,
                inject_count,
            )
            self.weights[replace_indices] = 1.0 / self.config.particle_count

        self._apply_camera_likelihood(list(self.observations))
        self._resample_if_needed()
        self._update_state()
        return self.estimate()

    def update_lidar(self, position, support: int) -> TargetEstimate | None:
        """连续一致的 LiDAR 测量才能进入锁定状态, 单帧异常不能永久覆盖视觉目标"""
        if self.completed:
            return self.estimate()
        measurement = np.asarray(position, dtype=np.float64).reshape(3)
        support = max(int(support), 0)

        has_visual_track = self.particles is not None and self.accepted_views > 0
        if has_visual_track and not self._is_lidar_associated_with_track(measurement):
            self.lidar_consistent_frames = 0
            self.last_lidar_position = None
            self.lidar_support = 0
            self.lidar_association_rejected += 1
            self._update_state()
            return self.estimate()

        if (
            self.last_lidar_position is not None
            and np.linalg.norm(measurement - self.last_lidar_position)
            <= self.config.lidar_consistency_radius
        ):
            self.lidar_consistent_frames += 1
        else:
            self.lidar_consistent_frames = 1
        self.last_lidar_position = measurement
        self.lidar_support = support

        # 视觉轨迹存在时先等待连续 LiDAR 证据, 避免单帧背景点改写目标
        if (
            has_visual_track
            and self.lidar_consistent_frames < self.config.lidar_lock_frames
        ):
            self._update_state()
            return self.estimate()

        if self.particles is None:
            self.particles = self.rng.normal(
                measurement,
                self.config.lidar_sigma,
                size=(self.config.particle_count, 3),
            )
            self.weights = np.full(self.config.particle_count, 1.0 / self.config.particle_count)
        else:
            distances = np.linalg.norm(self.particles - measurement, axis=1)
            likelihood = np.exp(-0.5 * np.square(distances / self.config.lidar_sigma)) + 1e-12
            self.weights *= likelihood
            self._normalize_weights()
            self._resample_if_needed()
        self._update_state()
        return self.estimate()

    def _is_lidar_associated_with_track(self, measurement: np.ndarray) -> bool:
        """使用视觉后验的水平不确定度关联 LiDAR 测量"""
        estimate = self.estimate()
        if estimate is None:
            return True
        horizontal_std = math.sqrt(
            max(float(estimate.covariance[0, 0] + estimate.covariance[1, 1]), 0.0)
        )
        association_radius = max(
            self.config.association_min_radius,
            self.config.association_sigma_factor * horizontal_std,
        )
        association_distance = float(
            np.linalg.norm(measurement[:2] - estimate.position[:2])
        )
        return association_distance <= association_radius

    def estimate(self) -> TargetEstimate | None:
        if self.particles is None or self.weights is None:
            return None
        position = np.average(self.particles, axis=0, weights=self.weights)
        centered = self.particles - position
        covariance = (centered * self.weights[:, None]).T @ centered
        horizontal_std = math.sqrt(max(float(covariance[0, 0] + covariance[1, 1]), 0.0))
        confidence = self._confidence(horizontal_std)
        stable = self.state in {
            FusionState.STABLE_VISION,
            FusionState.LIDAR_LOCKED,
        }
        if self.state == FusionState.REACHED:
            stable = self.reached_estimate_stable
        source = EstimateSource.VISION
        if self.accepted_views == 0 and self.lidar_support > 0:
            source = EstimateSource.LIDAR
        elif self.lidar_consistent_frames >= self.config.lidar_lock_frames:
            source = EstimateSource.FUSED
        if self.state == FusionState.LIDAR_LOCKED:
            source = EstimateSource.LIDAR
        return TargetEstimate(
            position=position,
            covariance=covariance,
            confidence=confidence,
            source=source,
            stable=stable,
            accepted_views=self.accepted_views,
            lidar_support=self.lidar_support,
            state=self.state,
        )

    def _assess_view(
        self,
        observation: CameraObservation,
        pending: list[_AcceptedView],
    ) -> _AcceptedView | None:
        """丢弃完全重复帧, 用最弱几何关系评估新增视角质量"""
        previous_views = [*self.observations, *pending]
        if not previous_views:
            return _AcceptedView(observation, quality=1.0, independent=True)

        origin, direction, _ = observation.center_ray()
        geometry = []
        for previous_view in previous_views:
            previous_origin, previous_direction, _ = (
                previous_view.observation.center_ray()
            )
            lateral_baseline, angle = _view_geometry(
                origin,
                direction,
                previous_origin,
                previous_direction,
            )
            if (
                lateral_baseline < self.config.duplicate_view_translation
                and angle < self.config.duplicate_view_angle_deg
            ):
                self.duplicate_views_rejected += 1
                return None
            if previous_view.independent:
                geometry.append((lateral_baseline, angle))

        if not geometry:
            return _AcceptedView(observation, quality=0.1, independent=False)
        quality = min(
            _view_quality(self.config, lateral_baseline, angle)
            for lateral_baseline, angle in geometry
        )
        independent = all(
            lateral_baseline >= self.config.independent_view_translation
            for lateral_baseline, _ in geometry
        )
        return _AcceptedView(observation, quality=quality, independent=independent)

    def _is_associated_with_track(self, observation: CameraObservation) -> bool:
        """稳定轨迹形成后使用射线距离门控, 防止远处误检污染持久目标"""
        if self.particles is None:
            return True
        if self.state not in {FusionState.STABLE_VISION, FusionState.LIDAR_LOCKED}:
            return True
        estimate = self.estimate()
        if estimate is None:
            return True
        origin, direction, _ = observation.center_ray()
        relative = estimate.position - origin
        depth = float(np.dot(relative, direction))
        if depth <= 0.0:
            return False
        closest = origin + depth * direction
        ray_distance = float(np.linalg.norm(estimate.position - closest))
        horizontal_std = math.sqrt(
            max(float(estimate.covariance[0, 0] + estimate.covariance[1, 1]), 0.0)
        )
        association_radius = max(
            self.config.association_min_radius,
            self.config.association_sigma_factor * horizontal_std,
        )
        return ray_distance <= association_radius

    def _sample_from_observations(
        self,
        observations: list[CameraObservation],
        particle_count: int,
    ) -> np.ndarray:
        observation_indices = self.rng.integers(0, len(observations), size=particle_count)
        particles = np.empty((particle_count, 3), dtype=np.float64)
        for observation_idx, observation in enumerate(observations):
            selected = np.flatnonzero(observation_indices == observation_idx)
            if selected.size == 0:
                continue
            rows, cols = np.nonzero(observation.mask)
            pixel_indices = self.rng.integers(0, len(rows), size=selected.size)
            pixels = np.vstack(
                (cols[pixel_indices], rows[pixel_indices], np.ones(selected.size))
            )
            depths = self.rng.uniform(
                self.config.min_depth,
                self.config.max_depth,
                size=selected.size,
            )
            camera_points = (np.linalg.inv(observation.intrinsic) @ pixels) * depths
            world_points = (
                observation.rotation_world_from_camera @ camera_points
                + observation.translation_world_from_camera.reshape(3, 1)
            )
            particles[selected] = world_points.T
        return particles

    def _apply_camera_likelihood(self, observations: list[_AcceptedView]) -> None:
        log_weights = np.log(np.maximum(self.weights, 1e-12))
        for accepted_view in observations:
            observation = accepted_view.observation
            camera_from_world = observation.rotation_world_from_camera.T
            camera_points = (
                camera_from_world
                @ (self.particles - observation.translation_world_from_camera).T
            ).T
            positive_depth = camera_points[:, 2] > 0.1
            projected = observation.intrinsic @ camera_points.T
            safe_depth = np.where(positive_depth, projected[2], 1.0)
            cols = np.rint(projected[0] / safe_depth).astype(np.int64)
            rows = np.rint(projected[1] / safe_depth).astype(np.int64)
            inside_image = (
                positive_depth
                & (rows >= 0)
                & (rows < observation.mask.shape[0])
                & (cols >= 0)
                & (cols < observation.mask.shape[1])
            )
            inside_mask = np.zeros(self.config.particle_count, dtype=bool)
            inside_mask[inside_image] = observation.mask[rows[inside_image], cols[inside_image]]

            origin, direction, angular_radius = observation.center_ray()
            relative = self.particles - origin
            ray_depth = relative @ direction
            closest = origin + ray_depth[:, None] * direction
            ray_distance = np.linalg.norm(self.particles - closest, axis=1)
            ray_sigma = np.maximum(np.abs(ray_depth) * math.tan(angular_radius), 0.5)
            ray_likelihood = np.exp(-0.5 * np.square(ray_distance / ray_sigma))
            mask_likelihood = np.where(inside_mask, 1.0, 0.05)
            likelihood = np.maximum(ray_likelihood * mask_likelihood, 1e-12)
            observation_weight = (
                0.5 + 0.5 * observation.confidence
            ) * accepted_view.quality
            log_weights += observation_weight * np.log(likelihood)

        log_weights -= np.max(log_weights)
        self.weights = np.exp(log_weights)
        self._normalize_weights()

    def _normalize_weights(self) -> None:
        total = float(np.sum(self.weights))
        if not np.isfinite(total) or total <= 0.0:
            self.weights.fill(1.0 / self.config.particle_count)
            return
        self.weights /= total

    def _resample_if_needed(self) -> None:
        effective_sample_size = 1.0 / float(np.sum(np.square(self.weights)))
        if effective_sample_size >= self.config.particle_count * self.config.resample_ess_ratio:
            return
        cumulative = np.cumsum(self.weights)
        positions = (
            self.rng.random() + np.arange(self.config.particle_count)
        ) / self.config.particle_count
        indices = np.searchsorted(cumulative, positions, side="right")
        self.particles = self.particles[indices]
        self.particles += self.rng.normal(
            0.0,
            self.config.resample_jitter,
            size=self.particles.shape,
        )
        self.weights.fill(1.0 / self.config.particle_count)

    def _update_state(self) -> None:
        if self.completed:
            return
        if self.lidar_consistent_frames >= self.config.lidar_lock_frames:
            self.state = FusionState.LIDAR_LOCKED
            return
        estimate = self.estimate()
        if estimate is None:
            self.state = FusionState.EMPTY
            return
        horizontal_std = math.sqrt(
            max(float(estimate.covariance[0, 0] + estimate.covariance[1, 1]), 0.0)
        )
        if (
            self.accepted_views >= self.config.stable_min_views
            and horizontal_std <= self.config.stable_max_horizontal_std
            and estimate.confidence >= self.config.stable_min_confidence
        ):
            self.state = FusionState.STABLE_VISION
        elif self.accepted_views >= self.config.stable_min_views:
            self.state = FusionState.TRACKING
        else:
            self.state = FusionState.PENDING

    def _confidence(self, horizontal_std: float) -> float:
        view_score = min(
            self.view_support / max(self.config.stable_min_views, 1),
            1.0,
        )
        spread_score = math.exp(
            -horizontal_std / max(self.config.stable_max_horizontal_std, 1e-6)
        )
        effective_lidar_frames = self.lidar_consistent_frames
        if (
            self.accepted_views > 0
            and effective_lidar_frames < self.config.lidar_lock_frames
        ):
            effective_lidar_frames = 0
        lidar_score = min(
            effective_lidar_frames / max(self.config.lidar_lock_frames, 1),
            1.0,
        )
        confidence = 0.35 * view_score + 0.45 * spread_score + 0.20 * lidar_score
        return float(np.clip(confidence, 0.0, 1.0))


def _view_geometry(
    origin: np.ndarray,
    direction: np.ndarray,
    previous_origin: np.ndarray,
    previous_direction: np.ndarray,
) -> tuple[float, float]:
    """计算两个视角的横向基线和射线夹角"""
    baseline = origin - previous_origin
    mean_direction = direction + previous_direction
    mean_direction_norm = np.linalg.norm(mean_direction)
    if mean_direction_norm < 1e-6:
        return 0.0, 180.0
    mean_direction /= mean_direction_norm
    lateral_baseline = float(
        np.linalg.norm(baseline - np.dot(baseline, mean_direction) * mean_direction)
    )
    angle = math.degrees(
        math.acos(float(np.clip(np.dot(direction, previous_direction), -1.0, 1.0)))
    )
    return lateral_baseline, angle


def _view_quality(
    config: ParticleFilterConfig,
    lateral_baseline: float,
    angle_deg: float,
) -> float:
    """将视角几何转换为连续权重, 夹角不再作为硬门槛"""
    translation_score = min(
        lateral_baseline / max(config.full_quality_translation, 1e-6),
        1.0,
    )
    angle_score = min(
        angle_deg / max(config.full_quality_angle_deg, 1e-6),
        1.0,
    )
    return float(np.clip(0.7 * translation_score + 0.3 * angle_score, 0.1, 1.0))
