import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, MultiArrayDimension, String
from visualization_msgs.msg import Marker

from object_search_msgs.msg import ObjectMaskWithTf
from triangulation3d.target_particle_filter import (
    CameraObservation,
    ParticleFilterConfig,
    TargetEstimate,
    TargetParticleFilter,
)
from visual_navigation.object_target_fusion import (
    ObjectTargetFusion,
    _mask_array,
    _target_ray_marker,
    _target_surface_measurement,
    _target_surface_measurement_with_reason,
    _mean_observation_bearing,
    _projected_mask_support,
    _target_marker,
    _xyz_points,
)
from visual_navigation.utils.object_search_utils import reference_image_stamp
from visual_navigation.utils.performance_stats import EventRate, TimingWindow


def test_mask_array_decodes_object_mask_message():
    """融合节点必须按 ObjectMaskWithTf 中的 B,1,H,W 布局恢复 Mask"""
    expected = np.arange(24, dtype=np.uint8).reshape(3, 1, 2, 4)
    msg = ObjectMaskWithTf()
    msg.object_mask.layout.dim = [
        MultiArrayDimension(label="batch", size=3, stride=24),
        MultiArrayDimension(label="channel", size=1, stride=8),
        MultiArrayDimension(label="height", size=2, stride=8),
        MultiArrayDimension(label="width", size=4, stride=4),
    ]
    msg.object_mask.data = expected.reshape(-1).tolist()

    decoded = _mask_array(msg)

    assert np.array_equal(decoded, expected)


def test_mask_callback_logs_exception_without_terminating_node():
    """单帧处理异常必须被记录, 不能退出融合进程"""

    class Logger:
        def __init__(self):
            self.errors = []

        def info(self, message):
            pass

        def debug(self, message):
            pass

        def error(self, message):
            self.errors.append(message)

    class CallbackHarness:
        _on_object_mask = ObjectTargetFusion._on_object_mask
        _set_mask_stage = ObjectTargetFusion._set_mask_stage

        def __init__(self):
            self._mask_received = 0
            self._mask_errors = 0
            self._mask_stage = "idle"
            self._mask_rate = EventRate()
            self._timings = {"total": TimingWindow()}
            self.logger = Logger()

        def get_logger(self):
            return self.logger

        def _process_object_mask(self, msg):
            self._mask_stage = "update_vision"
            raise RuntimeError("synthetic callback failure")

        def _warn_if_slow(self, elapsed_seconds, stage):
            pass

        def _record_message_age(self, name, stamp):
            pass

        def _record_timeline_event(self, name, stamp, detail=""):
            pass

    harness = CallbackHarness()

    harness._on_object_mask(ObjectMaskWithTf())

    assert harness._mask_received == 1
    assert harness._mask_errors == 1
    assert harness._mask_stage == "idle"
    assert "stage=update_vision" in harness.logger.errors[0]
    assert "RuntimeError: synthetic callback failure" in harness.logger.errors[0]


def test_target_change_clears_old_fusion_state():
    """新目标不能沿用旧目标粒子、点云和可视化"""

    class Clock:
        def now(self):
            return type("Now", (), {"nanoseconds": 20_000_000_000})()

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    class Logger:
        def info(self, message):
            pass

        def warn(self, message):
            pass

    class Harness:
        _on_object_search_target = ObjectTargetFusion._on_object_search_target

        def __init__(self):
            self.current_target = "old target"
            self._target_changed_stamp_sec = None
            self.particle_config = ParticleFilterConfig(particle_count=100)
            self.particle_filter = TargetParticleFilter(self.particle_config)
            self.particle_filter.particles = np.ones((100, 3))
            self.lidar_buffer = [(1.0, object())]
            self._latest_camera_origins = [np.ones(3)]
            self._latest_bearing_world = np.ones(3)
            self._last_logged_state = "TRACKING"
            self._first_event_stamps = {"first_mask": 1.0}
            self.marker_publisher = Publisher()
            self.logger = Logger()

        def get_clock(self):
            return Clock()

        def get_logger(self):
            return self.logger

    node = Harness()

    node._on_object_search_target(String(data="new target"))

    assert node.current_target == "new target"
    assert node._target_changed_stamp_sec == 20.0
    assert node.particle_filter.particles is None
    assert node.lidar_buffer == []
    assert node._latest_camera_origins == []
    assert node._latest_bearing_world is None
    assert node.marker_publisher.messages[-1].action == Marker.DELETEALL


def test_xyz_points_accepts_mixed_pointcloud_field_types():
    """XYZ 为 FLOAT32 时不能被额外的整型 timestamp 字段阻断"""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=12, datatype=PointField.UINT32, count=1),
    ]
    cloud = point_cloud2.create_cloud(
        Header(frame_id="dlio_odom"),
        fields,
        [(1.0, 2.0, 3.0, 10), (float("nan"), 5.0, 6.0, 20)],
    )

    points = _xyz_points(cloud)

    assert points.dtype == np.float64
    assert points.shape == (1, 3)
    assert np.allclose(points[0], [1.0, 2.0, 3.0])


def _estimate(
    stable: bool,
    covariance: np.ndarray,
    state: str | None = None,
    confidence: float = 0.8,
) -> TargetEstimate:
    return TargetEstimate(
        position=np.array([8.0, 2.0, 1.0]),
        covariance=covariance,
        confidence=confidence,
        source=1,
        stable=stable,
        accepted_views=2,
        lidar_support=0,
        state=state or ("STABLE_VISION" if stable else "PENDING"),
    )


def test_pending_estimate_deletes_misleading_position_marker():
    marker = _target_marker(_estimate(False, np.eye(3) * 100.0), "odom", Time())

    assert marker.action == Marker.DELETE


def test_stable_marker_scale_is_bounded():
    covariance = np.diag([100.0, 4.0, 0.25])
    marker = _target_marker(_estimate(True, covariance), "odom", Time())

    assert marker.action == Marker.ADD
    assert marker.pose.position.x == 8.0
    assert 0.3 <= marker.scale.x <= 1.5
    assert 0.3 <= marker.scale.y <= 1.5
    assert 0.3 <= marker.scale.z <= 1.5
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0


def test_two_view_tracking_estimate_publishes_coarse_marker():
    marker = _target_marker(
        _estimate(False, np.eye(3) * 16.0, state="TRACKING"),
        "odom",
        Time(),
    )

    assert marker.action == Marker.ADD
    assert marker.color.r == 1.0
    assert marker.color.g == 0.75


def test_coarse_marker_uses_same_confidence_gate_as_navigation():
    accepted = _target_marker(
        _estimate(
            False,
            np.eye(3) * 16.0,
            state="TRACKING",
            confidence=0.46,
        ),
        "odom",
        Time(),
    )
    rejected = _target_marker(
        _estimate(
            False,
            np.eye(3) * 16.0,
            state="TRACKING",
            confidence=0.44,
        ),
        "odom",
        Time(),
    )

    assert accepted.action == Marker.ADD
    assert rejected.action == Marker.DELETE


def test_visible_target_publishes_green_camera_rays():
    estimate = _estimate(True, np.eye(3))
    marker = _target_ray_marker(
        estimate,
        "odom",
        Time(),
        [np.array([0.0, 0.0, 0.5]), np.array([0.0, 0.2, 0.5])],
    )

    assert marker.action == Marker.ADD
    assert marker.type == Marker.LINE_LIST
    assert len(marker.points) == 4
    assert marker.points[1].x == 8.0
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 0.0


def test_pending_target_deletes_camera_rays():
    marker = _target_ray_marker(
        _estimate(False, np.eye(3), state="PENDING"),
        "odom",
        Time(),
        [np.zeros(3)],
    )

    assert marker.action == Marker.DELETE


def test_reference_image_stamp_uses_middle_camera_time():
    image_msgs = [Image(), Image(), Image()]
    image_msgs[0].header.stamp = Time(sec=10, nanosec=100)
    image_msgs[1].header.stamp = Time(sec=12, nanosec=100)
    image_msgs[2].header.stamp = Time(sec=11, nanosec=100)

    stamp = reference_image_stamp(image_msgs)

    assert stamp.sec == 11
    assert stamp.nanosec == 100


def test_lidar_projection_counts_same_point_once_across_cameras():
    mask = np.zeros((7, 7), dtype=bool)
    mask[2, 2] = True
    observation = CameraObservation(
        mask=mask,
        intrinsic=np.eye(3),
        rotation_world_from_camera=np.eye(3),
        translation_world_from_camera=np.zeros(3),
    )

    visible, supported = _projected_mask_support(
        np.array([[2.0, 2.0, 1.0]]),
        [observation, observation],
        dilation_pixels=0,
    )

    assert np.count_nonzero(visible) == 1
    assert np.count_nonzero(supported) == 1


def test_lidar_projection_uses_small_mask_dilation():
    mask = np.zeros((7, 7), dtype=bool)
    mask[2, 2] = True
    observation = CameraObservation(
        mask=mask,
        intrinsic=np.eye(3),
        rotation_world_from_camera=np.eye(3),
        translation_world_from_camera=np.zeros(3),
    )
    points = np.array([[4.0, 2.0, 1.0]])

    _, without_dilation = _projected_mask_support(
        points,
        [observation],
        dilation_pixels=0,
    )
    _, with_dilation = _projected_mask_support(
        points,
        [observation],
        dilation_pixels=2,
    )

    assert not without_dilation[0]
    assert with_dilation[0]


def test_pending_bearing_uses_camera_mask_center_ray():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 4] = True
    observation = CameraObservation(
        mask=mask,
        intrinsic=np.eye(3),
        rotation_world_from_camera=np.eye(3),
        translation_world_from_camera=np.zeros(3),
    )

    bearing = _mean_observation_bearing([observation])

    assert bearing is not None
    assert np.allclose(bearing, np.array([4.0, 2.0, 1.0]) / np.sqrt(21.0))


def test_lidar_measurement_rejects_ground_points_inside_mask():
    rng = np.random.default_rng(7)
    ground = np.column_stack(
        (
            rng.normal(0.0, 0.35, 45),
            rng.normal(-7.0, 0.35, 45),
            rng.normal(0.0, 0.015, 45),
        )
    )
    target = np.column_stack(
        (
            rng.normal(0.0, 0.18, 24),
            rng.normal(-9.0, 0.18, 24),
            rng.normal(0.55, 0.08, 24),
        )
    )

    measurement = _target_surface_measurement(
        np.vstack((ground, target)),
        minimum_support=30,
    )

    assert measurement is not None
    position, support = measurement
    assert support >= 10
    assert np.linalg.norm(position[:2] - np.array([0.0, -9.0])) < 0.5
    assert position[2] > 0.3


def test_lidar_measurement_prefers_nearest_foreground_cluster():
    """同一 mask 射线中不能因背景更密集而把目标推到墙面"""
    rng = np.random.default_rng(11)
    foreground = rng.normal(
        loc=np.array([4.0, 0.0, 0.7]),
        scale=np.array([0.12, 0.12, 0.08]),
        size=(24, 3),
    )
    background = rng.normal(
        loc=np.array([8.0, 0.0, 1.0]),
        scale=np.array([0.18, 0.18, 0.15]),
        size=(80, 3),
    )
    ground = rng.normal(
        loc=np.array([3.0, 0.0, 0.0]),
        scale=np.array([0.3, 0.3, 0.01]),
        size=(45, 3),
    )

    measurement = _target_surface_measurement(
        np.vstack((ground, foreground, background)),
        minimum_support=30,
        reference_position=np.zeros(3),
    )

    assert measurement is not None
    position, support = measurement
    assert support >= 10
    assert np.linalg.norm(position - np.array([4.0, 0.0, 0.7])) < 0.5


def test_lidar_failure_reports_insufficient_mask_points():
    """Mask 内点数不足需要有独立失败代码"""
    measurement, reason = _target_surface_measurement_with_reason(
        np.zeros((5, 3)),
        minimum_support=30,
    )

    assert measurement is None
    assert reason == "mask_points_insufficient"


def test_lidar_failure_reports_foreground_cluster_shortage():
    """高点存在但不能形成空间簇时需要区分为前景聚类失败"""
    x = np.arange(30, dtype=float) * 2.0
    points = np.column_stack((x, np.zeros_like(x), x * 0.1))

    measurement, reason = _target_surface_measurement_with_reason(
        points,
        minimum_support=18,
    )

    assert measurement is None
    assert reason == "foreground_cluster_insufficient"
