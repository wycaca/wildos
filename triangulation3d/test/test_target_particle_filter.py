import math

import numpy as np

from triangulation3d.target_particle_filter import (
    CameraObservation,
    FusionState,
    ParticleFilterConfig,
    TargetParticleFilter,
)


def _look_at_rotation(camera_position, target_position):
    """构造光学坐标系 Z 轴朝向目标的 world_from_camera 旋转"""
    forward = target_position - camera_position
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack((right, down, forward))


def _observation(camera_position, target_position, camera_id):
    intrinsic = np.array([[180.0, 0.0, 160.0], [0.0, 180.0, 120.0], [0.0, 0.0, 1.0]])
    rotation = _look_at_rotation(camera_position, target_position)
    camera_point = rotation.T @ (target_position - camera_position)
    pixel = intrinsic @ camera_point
    col = int(round(pixel[0] / pixel[2]))
    row = int(round(pixel[1] / pixel[2]))
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[max(row - 8, 0):row + 9, max(col - 8, 0):col + 9] = 1
    return CameraObservation(
        mask=mask,
        intrinsic=intrinsic,
        rotation_world_from_camera=rotation,
        translation_world_from_camera=camera_position,
        confidence=0.9,
        camera_id=camera_id,
    )


def test_multiview_filter_converges_near_target():
    target = np.array([8.0, 2.0, 1.0])
    config = ParticleFilterConfig(
        particle_count=4000,
        max_depth=15.0,
        stable_max_horizontal_std=5.0,
        stable_min_confidence=0.45,
    )
    particle_filter = TargetParticleFilter(config, seed=7)

    particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front"),
        _observation(np.array([1.0, -1.0, 1.0]), target, "left"),
    ])
    estimate = particle_filter.update_vision([
        _observation(np.array([2.0, 0.5, 1.0]), target, "front-2"),
    ])

    assert estimate is not None
    assert np.linalg.norm(estimate.position[:2] - target[:2]) < 2.0
    assert estimate.accepted_views >= 2
    assert estimate.state in {FusionState.TRACKING, FusionState.STABLE_VISION}


def test_repeated_view_does_not_increase_support():
    target = np.array([6.0, 1.0, 1.0])
    observation = _observation(np.array([0.0, 0.0, 1.0]), target, "front")
    particle_filter = TargetParticleFilter(ParticleFilterConfig(particle_count=500), seed=3)

    first = particle_filter.update_vision([observation])
    second = particle_filter.update_vision([observation])

    assert first.accepted_views == 1
    assert second.accepted_views == 1
    assert particle_filter.duplicate_views_rejected == 1


def test_forward_motion_does_not_create_triangulation_parallax():
    """沿目标射线直行没有横向基线, 不能增加多视角深度支持"""
    target = np.array([10.0, 0.0, 1.0])
    particle_filter = TargetParticleFilter(ParticleFilterConfig(particle_count=500), seed=4)

    first = particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front-0"),
    ])
    second = particle_filter.update_vision([
        _observation(np.array([1.0, 0.0, 1.0]), target, "front-1"),
    ])

    assert first.accepted_views == 1
    assert second.accepted_views == 1
    assert second.state == FusionState.PENDING


def test_far_target_accepts_lateral_view_without_three_degree_angle():
    """远目标有横向基线时不再被固定三度夹角丢弃"""
    target = np.array([20.0, 0.0, 1.0])
    particle_filter = TargetParticleFilter(
        ParticleFilterConfig(particle_count=1000, max_depth=30.0),
        seed=12,
    )

    particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front-0"),
    ])
    estimate = particle_filter.update_vision([
        _observation(np.array([0.0, 0.15, 1.0]), target, "front-1"),
    ])

    assert estimate.accepted_views == 2
    assert estimate.state in {FusionState.TRACKING, FusionState.STABLE_VISION}
    assert 1.0 < particle_filter.view_support < 2.0


def test_weak_view_updates_particles_without_increasing_independent_views():
    """较小横向位移可低权重更新, 但不能伪装成完整独立视角"""
    target = np.array([12.0, 0.0, 1.0])
    particle_filter = TargetParticleFilter(
        ParticleFilterConfig(particle_count=800, max_depth=20.0),
        seed=13,
    )
    particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front-0"),
    ])
    particles_before = particle_filter.particles.copy()

    estimate = particle_filter.update_vision([
        _observation(np.array([0.0, 0.06, 1.0]), target, "front-weak"),
    ])

    assert estimate.accepted_views == 1
    assert estimate.state == FusionState.PENDING
    assert len(particle_filter.observations) == 2
    assert particle_filter.weak_view_updates == 1
    assert not np.array_equal(particle_filter.particles, particles_before)


def test_lidar_requires_consistent_frames_and_reached_is_terminal():
    config = ParticleFilterConfig(particle_count=500, lidar_lock_frames=2)
    particle_filter = TargetParticleFilter(config, seed=5)

    first = particle_filter.update_lidar(np.array([4.0, 2.0, 1.0]), support=80)
    second = particle_filter.update_lidar(np.array([4.2, 2.1, 1.0]), support=95)

    assert first.state != FusionState.LIDAR_LOCKED
    assert second.state == FusionState.LIDAR_LOCKED

    particle_filter.mark_reached()
    reached = particle_filter.update_lidar(np.array([20.0, 20.0, 1.0]), support=500)

    assert reached.state == FusionState.REACHED
    assert reached.stable
    assert math.dist(reached.position, second.position) < 1e-9


def test_consistent_lidar_far_from_visual_track_cannot_lock():
    particle_filter, target = _filter_with_visual_posterior(seed=15)
    particles_before = particle_filter.particles.copy()
    far_measurement = target + np.array([20.0, 20.0, 0.0])

    first = particle_filter.update_lidar(far_measurement, support=80)
    second = particle_filter.update_lidar(
        far_measurement + np.array([0.1, 0.0, 0.0]),
        support=90,
    )

    assert first.state == FusionState.STABLE_VISION
    assert second.state == FusionState.STABLE_VISION
    assert particle_filter.lidar_consistent_frames == 0
    assert particle_filter.lidar_association_rejected == 2
    assert np.array_equal(particle_filter.particles, particles_before)


def test_associated_consecutive_lidar_measurements_can_lock():
    particle_filter, target = _filter_with_visual_posterior(seed=16)

    first = particle_filter.update_lidar(
        target + np.array([0.2, 0.1, 0.0]),
        support=80,
    )
    second = particle_filter.update_lidar(
        target + np.array([0.3, 0.1, 0.0]),
        support=90,
    )

    assert first.state == FusionState.STABLE_VISION
    assert second.state == FusionState.LIDAR_LOCKED
    assert particle_filter.lidar_association_rejected == 0


def test_lidar_outlier_resets_consecutive_associated_frames():
    particle_filter, target = _filter_with_visual_posterior(seed=17)
    valid = target + np.array([0.2, 0.1, 0.0])

    particle_filter.update_lidar(valid, support=80)
    particle_filter.update_lidar(target + np.array([20.0, 20.0, 0.0]), support=80)
    after_restart = particle_filter.update_lidar(valid, support=80)

    assert particle_filter.lidar_consistent_frames == 1
    assert after_restart.state == FusionState.STABLE_VISION

    locked = particle_filter.update_lidar(valid, support=80)

    assert locked.state == FusionState.LIDAR_LOCKED


def test_reached_does_not_promote_unstable_estimate():
    target = np.array([8.0, 0.0, 1.0])
    particle_filter = TargetParticleFilter(ParticleFilterConfig(particle_count=500), seed=8)
    particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front"),
    ])

    particle_filter.mark_reached()
    reached = particle_filter.estimate()

    assert reached.state == FusionState.REACHED
    assert not reached.stable


def test_inconsistent_view_cannot_replace_locked_target():
    """稳定目标形成后远处错误 Mask 不能增加支持数或移动目标"""
    target = np.array([5.0, 0.0, 1.0])
    wrong_target = np.array([5.0, 12.0, 1.0])
    particle_filter = TargetParticleFilter(ParticleFilterConfig(particle_count=1000), seed=9)
    particle_filter.update_lidar(target, support=80)
    locked = particle_filter.update_lidar(target + np.array([0.1, 0.0, 0.0]), support=90)

    after_outlier = particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), wrong_target, "wrong"),
    ])

    assert after_outlier.accepted_views == 0
    assert np.linalg.norm(after_outlier.position - locked.position) < 1e-9


def test_single_lidar_frame_cannot_move_visual_track():
    """已有视觉轨迹时单帧 LiDAR 候选不能立即改写位置"""
    target = np.array([8.0, 0.0, 1.0])
    config = ParticleFilterConfig(
        particle_count=1000,
        max_depth=15.0,
        lidar_lock_frames=2,
    )
    particle_filter = TargetParticleFilter(config, seed=10)
    visual = particle_filter.update_vision([
        _observation(np.array([0.0, 0.0, 1.0]), target, "front"),
    ])

    after_single_lidar = particle_filter.update_lidar(
        np.array([250.0, 200.0, 1.0]),
        support=100,
    )

    assert np.linalg.norm(after_single_lidar.position - visual.position) < 1e-9
    assert after_single_lidar.confidence == visual.confidence
    assert after_single_lidar.source == visual.source
    assert after_single_lidar.state != FusionState.LIDAR_LOCKED
    assert particle_filter.lidar_association_rejected == 1


def _filter_with_visual_posterior(seed: int):
    target = np.array([8.0, 2.0, 1.0])
    config = ParticleFilterConfig(
        particle_count=500,
        lidar_lock_frames=2,
    )
    particle_filter = TargetParticleFilter(config, seed=seed)
    particle_filter.particles = particle_filter.rng.normal(
        target,
        0.1,
        size=(config.particle_count, 3),
    )
    particle_filter.weights = np.full(
        config.particle_count,
        1.0 / config.particle_count,
    )
    particle_filter.accepted_views = 2
    particle_filter.view_support = 2.0
    particle_filter.state = FusionState.STABLE_VISION
    return particle_filter, target
