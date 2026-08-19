from __future__ import annotations

from collections import Counter, deque
import faulthandler
import math
import sys
import time
import traceback

import numpy as np
import rclpy
import scipy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from object_search_msgs.msg import ObjectMaskWithTf, TargetEstimate
from triangulation3d.target_particle_filter import (
    CameraObservation,
    ParticleFilterConfig,
    TargetEstimate as CoreTargetEstimate,
    TargetParticleFilter,
)
from visual_navigation.utils.performance_stats import EventRate, TimingWindow
from visual_navigation.object_search_types import (
    coarse_target_evidence_ready,
    normalize_object_search_target,
)


_LIDAR_BUFFER_SIZE = 40
_MAX_TARGET_MARKER_SCALE = 1.5
_MIN_TARGET_HEIGHT_ABOVE_GROUND = 0.12
_TARGET_CLUSTER_RADIUS = 0.75
_DIAGNOSTICS_LOG_PERIOD_SEC = 60.0
_SLOW_CALLBACK_WARNING_MS = 500.0


class ObjectTargetFusion(Node):
    """用多视角 Mask 估计远距离粗目标, LiDAR 仅作为可选精度增强"""

    def __init__(self):
        super().__init__("object_target_fusion")

        self.declare_parameter("object_mask_topic", "/spot1/object_mask")
        self.declare_parameter("lidar_topic", "/spot1/lidar/points_aligned")
        self.declare_parameter("target_estimate_topic", "/spot1/object_target_estimate")
        self.declare_parameter("target_marker_topic", "/spot1/object_target_estimate_viz")
        self.declare_parameter("particle_topic", "/spot1/object_target_particles")
        self.declare_parameter("completion_topic", "/spot1/object_search_completed")
        self.declare_parameter("object_search_target_topic", "/spot1/object_search_target")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("particle_count", 1500)
        self.declare_parameter("max_depth", 100.0)
        self.declare_parameter("stable_min_confidence", 0.6)
        self.declare_parameter("coarse_target_min_views", 2)
        self.declare_parameter("coarse_target_min_confidence", 0.45)
        self.declare_parameter("independent_view_translation", 0.12)
        self.declare_parameter("duplicate_view_translation", 0.03)
        self.declare_parameter("duplicate_view_angle_deg", 0.5)
        self.declare_parameter("full_quality_translation", 0.3)
        self.declare_parameter("full_quality_angle_deg", 3.0)
        self.declare_parameter("lidar_min_points", 18)
        self.declare_parameter("lidar_mask_dilation_pixels", 3)
        self.declare_parameter("max_lidar_age_sec", 0.25)

        self.max_depth = max(float(self.get_parameter("max_depth").value), 2.0)
        self.particle_config = ParticleFilterConfig(
            particle_count=max(int(self.get_parameter("particle_count").value), 100),
            max_depth=self.max_depth,
            stable_min_confidence=float(self.get_parameter("stable_min_confidence").value),
            independent_view_translation=max(
                float(self.get_parameter("independent_view_translation").value),
                0.01,
            ),
            duplicate_view_translation=max(
                float(self.get_parameter("duplicate_view_translation").value),
                0.0,
            ),
            duplicate_view_angle_deg=max(
                float(self.get_parameter("duplicate_view_angle_deg").value),
                0.0,
            ),
            full_quality_translation=max(
                float(self.get_parameter("full_quality_translation").value),
                0.01,
            ),
            full_quality_angle_deg=max(
                float(self.get_parameter("full_quality_angle_deg").value),
                0.1,
            ),
        )
        self.particle_filter = TargetParticleFilter(self.particle_config)
        self.current_target: str | None = None
        self._target_changed_stamp_sec: float | None = None
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.coarse_target_min_views = max(
            int(self.get_parameter("coarse_target_min_views").value),
            2,
        )
        self.coarse_target_min_confidence = max(
            float(self.get_parameter("coarse_target_min_confidence").value),
            0.0,
        )
        self.lidar_min_points = max(int(self.get_parameter("lidar_min_points").value), 1)
        self.lidar_mask_dilation_pixels = max(
            int(self.get_parameter("lidar_mask_dilation_pixels").value),
            0,
        )
        self.max_lidar_age_sec = max(float(self.get_parameter("max_lidar_age_sec").value), 0.0)
        self.lidar_buffer: deque[tuple[float, PointCloud2]] = deque(
            maxlen=_LIDAR_BUFFER_SIZE
        )
        self._lidar_cache_key: tuple[int, int, str] | None = None
        self._lidar_cache_points: np.ndarray | None = None
        self._lidar_cache_world_points: np.ndarray | None = None
        self._lidar_cache_hits = 0
        self._last_logged_state = ""
        self._latest_camera_origins: list[np.ndarray] = []
        self._latest_bearing_world: np.ndarray | None = None
        self._mask_stage = "idle"
        self._mask_received = 0
        self._mask_processed = 0
        self._mask_empty = 0
        self._mask_errors = 0
        self._mask_ignored_reached = 0
        self._lidar_matched = 0
        self._lidar_refined = 0
        self._lidar_failures: Counter[str] = Counter()
        self._lidar_point_counts = {
            name: deque(maxlen=256)
            for name in ("cloud", "visible", "mask", "elevated", "cluster")
        }
        self._first_event_stamps: dict[str, float] = {}
        self._last_slow_warning = 0.0
        self._mask_rate = EventRate()
        self._lidar_rate = EventRate()
        self._timings = {
            name: TimingWindow()
            for name in (
                "decode",
                "vision",
                "lidar",
                "lidar_decode",
                "lidar_transform",
                "lidar_project",
                "publish",
                "total",
            )
        }
        self._message_ages = {
            name: TimingWindow()
            for name in ("mask", "lidar_match")
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.estimate_publisher = self.create_publisher(
            TargetEstimate,
            str(self.get_parameter("target_estimate_topic").value),
            10,
        )
        self.marker_publisher = self.create_publisher(
            Marker,
            str(self.get_parameter("target_marker_topic").value),
            10,
        )
        self.particle_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("particle_topic").value),
            10,
        )
        self.create_subscription(
            ObjectMaskWithTf,
            str(self.get_parameter("object_mask_topic").value),
            self._on_object_mask,
            1,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar_topic").value),
            self._on_lidar,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("completion_topic").value),
            self._on_completed,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("object_search_target_topic").value),
            self._on_object_search_target,
            10,
        )
        self.create_timer(_DIAGNOSTICS_LOG_PERIOD_SEC, self._log_health)

        self.get_logger().info(
            "目标融合已启动, "
            f"Mask话题={self.get_parameter('object_mask_topic').value}, "
            f"雷达话题={self.get_parameter('lidar_topic').value}, "
            f"估计话题={self.get_parameter('target_estimate_topic').value}, "
            f"最大深度={self.max_depth:.1f}m, "
            f"独立视角横向基线={self.particle_config.independent_view_translation:.2f}m, "
            f"重复帧门槛={self.particle_config.duplicate_view_translation:.2f}m/"
            f"{self.particle_config.duplicate_view_angle_deg:.1f}deg, "
            f"雷达匹配时间差上限={self.max_lidar_age_sec:.2f}s, "
            f"Mask扩张={self.lidar_mask_dilation_pixels}px, "
            f"最少投影点={self.lidar_min_points}, "
            f"python={sys.executable}, numpy={np.__version__}, scipy={scipy.__version__}"
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        self.lidar_buffer.append((_stamp_seconds(msg.header.stamp), msg))
        self._lidar_rate.tick()

    def _on_object_search_target(self, msg: String) -> None:
        """新任务必须丢弃旧目标粒子和可能滞留的旧 Mask"""
        target = normalize_object_search_target(msg.data)
        if target is None:
            self.get_logger().warn("目标融合收到空搜索目标, 已忽略")
            return
        if target == self.current_target:
            return

        self.current_target = target
        self._target_changed_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.particle_filter = TargetParticleFilter(self.particle_config)
        self.lidar_buffer.clear()
        self._lidar_cache_key = None
        self._lidar_cache_points = None
        self._lidar_cache_world_points = None
        self._latest_camera_origins.clear()
        self._latest_bearing_world = None
        self._last_logged_state = ""
        self._first_event_stamps.clear()
        if self.marker_publisher.get_subscription_count() > 0:
            self.marker_publisher.publish(Marker(action=Marker.DELETEALL))
        self.get_logger().info(f"目标融合状态已重置, target={target!r}")

    def _on_completed(self, msg: Bool) -> None:
        """只接受 Mux 最终完成通知, 未稳定估计不能提前终止融合"""
        if not msg.data or self.particle_filter.completed:
            return
        estimate = self.particle_filter.estimate()
        if estimate is None or not estimate.stable:
            self.get_logger().warn("收到完成通知但融合目标未稳定, 已忽略")
            return
        self.particle_filter.mark_reached()
        estimate = self.particle_filter.estimate()
        stamp = self.get_clock().now().to_msg()
        self._log_estimate_state(estimate)
        self._publish_estimate(estimate, stamp)
        self._publish_markers(estimate, stamp)
        self.get_logger().info("目标融合收到 Mux 完成通知, 状态=任务完成(REACHED)")

    def _on_object_mask(self, msg: ObjectMaskWithTf) -> None:
        """隔离单帧异常, 保留融合节点并记录完整处理阶段"""
        mask_stamp = _stamp_seconds(msg.header.stamp)
        if (
            getattr(self, "_target_changed_stamp_sec", None) is not None
            and mask_stamp > 0.0
            and mask_stamp < self._target_changed_stamp_sec
        ):
            return
        callback_started = time.perf_counter()
        self._mask_received += 1
        self._mask_rate.tick()
        self._record_message_age("mask", msg.header.stamp)
        self._record_timeline_event("first_mask", msg.header.stamp)
        self._set_mask_stage("received", _mask_message_summary(msg))
        try:
            self._process_object_mask(msg)
        except Exception:
            self._mask_errors += 1
            self.get_logger().error(
                f"目标融合处理 Mask 失败, frame={self._mask_received}, "
                f"stage={self._mask_stage}\n{traceback.format_exc()}"
            )
        finally:
            final_stage = self._mask_stage
            self._mask_stage = "idle"
            elapsed = time.perf_counter() - callback_started
            self._timings["total"].add_seconds(elapsed)
            self._warn_if_slow(elapsed, final_stage)

    def _process_object_mask(self, msg: ObjectMaskWithTf) -> None:
        """每条确认 Mask 都先更新视觉粒子, 有近时刻点云时再追加 LiDAR 测量"""
        if self.particle_filter.completed:
            self._mask_ignored_reached += 1
            return
        self._set_mask_stage("decode_mask")
        stage_started = time.perf_counter()
        try:
            observations = self._camera_observations(msg)
        except ValueError as exc:
            self.get_logger().warn(f"目标 Mask 数据无效, 已跳过: {exc}")
            return
        finally:
            self._timings["decode"].add_seconds(time.perf_counter() - stage_started)
        if not observations:
            self._mask_empty += 1
            return
        self._latest_camera_origins = [
            observation.translation_world_from_camera.copy()
            for observation in observations
        ]
        self._latest_bearing_world = _mean_observation_bearing(observations)

        self._set_mask_stage("update_vision", f"observations={len(observations)}")
        stage_started = time.perf_counter()
        estimate = self.particle_filter.update_vision(observations)
        self._timings["vision"].add_seconds(time.perf_counter() - stage_started)
        self._set_mask_stage("match_lidar", f"buffer={len(self.lidar_buffer)}")
        lidar_match = self._nearest_lidar(msg.header.stamp)
        if lidar_match is not None:
            lidar_msg, lidar_age = lidar_match
            self._lidar_matched += 1
            self._message_ages["lidar_match"].add_seconds(lidar_age)
            self._set_mask_stage(
                "project_lidar",
                f"frame={lidar_msg.header.frame_id}, points={lidar_msg.width * lidar_msg.height}",
            )
            stage_started = time.perf_counter()
            lidar_measurement, failure_reason = self._lidar_measurement(
                observations,
                lidar_msg,
            )
            self._timings["lidar"].add_seconds(time.perf_counter() - stage_started)
            if lidar_measurement is not None:
                position, support = lidar_measurement
                self._set_mask_stage("update_lidar", f"support={support}")
                rejected_before = self.particle_filter.lidar_association_rejected
                estimate = self.particle_filter.update_lidar(position, support)
                if self.particle_filter.lidar_association_rejected > rejected_before:
                    self._lidar_failures["visual_track_mismatch"] += 1
                    self._set_mask_stage("reject_lidar", "visual_track_mismatch")
                else:
                    self._lidar_refined += 1
            elif failure_reason is not None:
                self._lidar_failures[failure_reason] += 1
        else:
            self._lidar_failures["no_time_matched_cloud"] += 1

        if estimate is None:
            return
        self._set_mask_stage(
            "publish",
            f"state={estimate.state}, views={estimate.accepted_views}, "
            f"lidar_support={estimate.lidar_support}",
        )
        stage_started = time.perf_counter()
        self._log_estimate_state(estimate)
        self._publish_estimate(estimate, msg.header.stamp)
        self._publish_markers(estimate, msg.header.stamp)
        self._publish_particles(msg.header.stamp)
        self._timings["publish"].add_seconds(time.perf_counter() - stage_started)
        self._mask_processed += 1

    def _record_message_age(self, name: str, stamp) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - _stamp_seconds(stamp)
        if math.isfinite(age) and age >= 0.0:
            self._message_ages[name].add_seconds(age)

    def _record_timeline_event(self, name: str, stamp, detail: str = "") -> None:
        """关键状态只记录首次发生时间, 避免逐帧重复日志"""
        if name in self._first_event_stamps:
            return
        event_stamp = _stamp_seconds(stamp)
        self._first_event_stamps[name] = event_stamp
        first_stamp = self._first_event_stamps.get("first_mask", event_stamp)
        suffix = f", {detail}" if detail else ""
        self.get_logger().info(
            f"目标定位时间线, event={name}, stamp={event_stamp:.3f}, "
            f"since_first_mask={event_stamp - first_stamp:.2f}s{suffix}"
        )

    def _set_mask_stage(self, stage: str, detail: str = "") -> None:
        """首帧逐阶段记录, 后续由周期统计报告当前阶段"""
        self._mask_stage = stage
        if self._mask_received != 1:
            return
        suffix = f", {detail}" if detail else ""
        self.get_logger().debug(f"首帧目标 Mask 处理阶段={stage}{suffix}")

    def _warn_if_slow(self, elapsed_seconds: float, stage: str) -> None:
        """Report sustained-risk callbacks without logging every slow frame"""
        now = time.monotonic()
        if (
            elapsed_seconds * 1000.0 < _SLOW_CALLBACK_WARNING_MS
            or now - self._last_slow_warning < 30.0
        ):
            return
        self._last_slow_warning = now
        self.get_logger().warn(
            f"目标融合处理耗时偏高, total={elapsed_seconds * 1000.0:.1f}ms, "
            f"threshold={_SLOW_CALLBACK_WARNING_MS:.1f}ms, stage={stage}"
        )

    def _log_health(self) -> None:
        """周期报告融合存活状态和最近回调阶段"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._timings.items()
        }
        age_summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._message_ages.items()
        }
        refine_ratio = 100.0 * self._lidar_refined / max(self._lidar_matched, 1)
        total = summaries["total"]
        self.get_logger().info(
            "目标融合统计, "
            f"Mask=收到{self._mask_received}/完成{self._mask_processed}/"
            f"空数据{self._mask_empty}/错误{self._mask_errors}, "
            f"输入频率=Mask {self._mask_rate.sample(reset=True):.2f}Hz/"
            f"雷达{self._lidar_rate.sample(reset=True):.1f}Hz, "
            f"雷达精修={self._lidar_refined}/{self._lidar_matched}"
            f"({refine_ratio:.1f}%), "
            f"精修失败={_counter_summary(self._lidar_failures)}, "
            "雷达点数平均值="
            f"原始{self._lidar_count_average('cloud'):.0f}/"
            f"相机内{self._lidar_count_average('visible'):.0f}/"
            f"Mask内{self._lidar_count_average('mask'):.0f}/"
            f"去地面{self._lidar_count_average('elevated'):.0f}/"
            f"最终簇{self._lidar_count_average('cluster'):.0f}, "
            f"视角=独立{self.particle_filter.accepted_views}/"
            f"有效权重{self.particle_filter.view_support:.2f}/"
            f"弱更新{self.particle_filter.weak_view_updates}/"
            f"重复丢弃{self.particle_filter.duplicate_views_rejected}, "
            f"雷达视觉关联拒绝{self.particle_filter.lidar_association_rejected}, "
            f"雷达缓存复用{self._lidar_cache_hits}次, "
            "消息年龄="
            f"Mask平均{age_summaries['mask'].average_ms:.0f}/"
            f"95%上限{age_summaries['mask'].p95_ms:.0f}ms, "
            f"匹配点云时间差平均{age_summaries['lidar_match'].average_ms:.0f}/"
            f"95%上限{age_summaries['lidar_match'].p95_ms:.0f}ms, "
            f"融合状态={_estimate_state_name(self.particle_filter.state)}, "
            f"总耗时=平均{total.average_ms:.0f}/95%上限{total.p95_ms:.0f}/"
            f"最大{total.maximum_ms:.0f}ms, "
            "阶段平均耗时="
            f"Mask解析{summaries['decode'].average_ms:.0f}ms/"
            f"视觉融合{summaries['vision'].average_ms:.0f}ms/"
            f"雷达总计{summaries['lidar'].average_ms:.0f}ms/"
            f"点云解码{summaries['lidar_decode'].average_ms:.0f}ms/"
            f"坐标转换{summaries['lidar_transform'].average_ms:.0f}ms/"
            f"雷达投影{summaries['lidar_project'].average_ms:.0f}ms/"
            f"结果发布{summaries['publish'].average_ms:.0f}ms"
        )
        self._lidar_cache_hits = 0

    def _log_estimate_state(self, estimate: CoreTargetEstimate) -> None:
        if estimate.state == self._last_logged_state:
            return
        self._last_logged_state = estimate.state
        self._record_timeline_event(
            estimate.state.lower(),
            self.get_clock().now().to_msg(),
            f"views={estimate.accepted_views}, confidence={estimate.confidence:.2f}",
        )
        self.get_logger().info(
            f"目标融合状态变化, 状态={_estimate_state_name(estimate.state)}, "
            f"位置=({estimate.position[0]:.2f}, {estimate.position[1]:.2f}, "
            f"{estimate.position[2]:.2f}), 置信度={estimate.confidence:.2f}, "
            f"有效视角={estimate.accepted_views}, "
            f"视角权重={self.particle_filter.view_support:.2f}, "
            f"雷达支持点={estimate.lidar_support}"
        )

    def _camera_observations(self, msg: ObjectMaskWithTf) -> list[CameraObservation]:
        masks = _mask_array(msg)
        if masks.shape[0] != len(msg.cam_infos):
            raise ValueError("mask 数量和 CameraInfo 数量不一致")
        if masks.shape[0] != len(msg.cam_transforms.transforms):
            raise ValueError("mask 数量和相机 TF 数量不一致")

        scores = list(msg.camera_scores)
        observations = []
        for camera_idx in range(masks.shape[0]):
            mask = masks[camera_idx, 0].astype(bool)
            if not np.any(mask):
                continue
            camera_info = msg.cam_infos[camera_idx]
            transform = msg.cam_transforms.transforms[camera_idx]
            rotation = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]).as_matrix()
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ])
            score = scores[camera_idx] if camera_idx < len(scores) else 1.0
            observations.append(
                CameraObservation(
                    mask=mask,
                    intrinsic=np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3),
                    rotation_world_from_camera=rotation,
                    translation_world_from_camera=translation,
                    confidence=score,
                    camera_id=transform.child_frame_id,
                )
            )
        return observations

    def _nearest_lidar(self, stamp) -> tuple[PointCloud2, float] | None:
        if not self.lidar_buffer:
            return None
        target_time = _stamp_seconds(stamp)
        age, lidar_msg = min(
            (
                (abs(lidar_time - target_time), lidar_msg)
                for lidar_time, lidar_msg in self.lidar_buffer
            ),
            key=lambda item: item[0],
        )
        return (lidar_msg, age) if age <= self.max_lidar_age_sec else None

    def _lidar_measurement(
        self,
        observations: list[CameraObservation],
        lidar_msg: PointCloud2,
    ) -> tuple[tuple[np.ndarray, int] | None, str | None]:
        """联合三相机投影, 去地面后选择最近的连续前景簇"""
        points, world_points = self._cached_lidar_points(lidar_msg)
        if points.size == 0:
            self._record_lidar_counts()
            return None, "empty_cloud"
        if world_points is None:
            self._record_lidar_counts(cloud=points.shape[0])
            return None, "tf_unavailable"

        project_started = time.perf_counter()
        visible_union, mask_union = _projected_mask_support(
            world_points,
            observations,
            self.lidar_mask_dilation_pixels,
        )
        self._timings["lidar_project"].add_seconds(
            time.perf_counter() - project_started
        )

        supported_points = world_points[mask_union]
        if supported_points.size == 0:
            self._record_lidar_counts(
                cloud=world_points.shape[0],
                visible=np.count_nonzero(visible_union),
            )
            return None, "no_points_in_mask"
        if supported_points.shape[0] < self.lidar_min_points:
            self._record_lidar_counts(
                cloud=world_points.shape[0],
                visible=np.count_nonzero(visible_union),
                mask=supported_points.shape[0],
            )
            return None, "mask_points_insufficient"

        measurement, failure_reason, details = _target_surface_measurement_details(
            supported_points,
            self.lidar_min_points,
            reference_position=np.mean(
                [observation.translation_world_from_camera for observation in observations],
                axis=0,
            ),
        )
        self._record_lidar_counts(
            cloud=world_points.shape[0],
            visible=np.count_nonzero(visible_union),
            mask=supported_points.shape[0],
            elevated=details["elevated"],
            cluster=details["cluster"],
        )
        if measurement is None:
            return None, failure_reason
        return measurement, None

    def _cached_lidar_points(
        self,
        lidar_msg: PointCloud2,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """Decode and transform one cloud once across masks sharing its identity"""
        key = (
            int(lidar_msg.header.stamp.sec),
            int(lidar_msg.header.stamp.nanosec),
            lidar_msg.header.frame_id,
        )
        if key != self._lidar_cache_key:
            decode_started = time.perf_counter()
            self._lidar_cache_points = _xyz_points(lidar_msg)
            self._timings["lidar_decode"].add_seconds(
                time.perf_counter() - decode_started
            )
            self._lidar_cache_key = key
            self._lidar_cache_world_points = None
        elif self._lidar_cache_world_points is not None:
            self._lidar_cache_hits += 1
            return self._lidar_cache_points, self._lidar_cache_world_points

        if self._lidar_cache_points.size == 0:
            self._lidar_cache_world_points = self._lidar_cache_points
            return self._lidar_cache_points, self._lidar_cache_world_points
        transform_started = time.perf_counter()
        world_points = self._points_in_global_frame(
            self._lidar_cache_points,
            lidar_msg,
        )
        self._timings["lidar_transform"].add_seconds(
            time.perf_counter() - transform_started
        )
        if world_points is not None:
            self._lidar_cache_world_points = world_points
        return self._lidar_cache_points, world_points

    def _record_lidar_counts(
        self,
        cloud: int = 0,
        visible: int = 0,
        mask: int = 0,
        elevated: int = 0,
        cluster: int = 0,
    ) -> None:
        """保存精修各阶段点数, 仅由周期日志汇总"""
        values = {
            "cloud": cloud,
            "visible": visible,
            "mask": mask,
            "elevated": elevated,
            "cluster": cluster,
        }
        for name, value in values.items():
            self._lidar_point_counts[name].append(int(value))

    def _lidar_count_average(self, name: str) -> float:
        values = self._lidar_point_counts[name]
        return float(np.mean(values)) if values else 0.0

    def _points_in_global_frame(
        self,
        points: np.ndarray,
        lidar_msg: PointCloud2,
    ) -> np.ndarray | None:
        lidar_frame = lidar_msg.header.frame_id.lstrip("/")
        global_frame = self.global_frame.lstrip("/")
        if lidar_frame == global_frame:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(
                global_frame,
                lidar_frame,
                rclpy.time.Time.from_msg(lidar_msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().debug(f"LiDAR TF 暂不可用, 本帧只做视觉融合: {exc}")
            return None
        rotation = Rotation.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]).as_matrix()
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        return (rotation @ points.T).T + translation

    def _publish_estimate(self, estimate: CoreTargetEstimate, stamp) -> None:
        msg = TargetEstimate()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = stamp
        msg.pose.pose.position.x = float(estimate.position[0])
        msg.pose.pose.position.y = float(estimate.position[1])
        msg.pose.pose.position.z = float(estimate.position[2])
        msg.pose.pose.orientation.w = 1.0
        for row in range(3):
            for col in range(3):
                msg.pose.covariance[row * 6 + col] = float(estimate.covariance[row, col])
        msg.confidence = float(estimate.confidence)
        msg.source = int(estimate.source)
        msg.stable = bool(estimate.stable)
        msg.accepted_views = int(estimate.accepted_views)
        msg.lidar_support = int(estimate.lidar_support)
        msg.state = estimate.state
        if self._latest_bearing_world is not None:
            msg.bearing.x = float(self._latest_bearing_world[0])
            msg.bearing.y = float(self._latest_bearing_world[1])
            msg.bearing.z = float(self._latest_bearing_world[2])
            msg.bearing_valid = True
        self.estimate_publisher.publish(msg)

    def _publish_markers(self, estimate: CoreTargetEstimate, stamp) -> None:
        """在同一 Marker 话题发布目标球和观测射线, 避免增加重复可视化话题"""
        if self.marker_publisher.get_subscription_count() == 0:
            return
        self.marker_publisher.publish(
            _target_marker(
                estimate,
                self.global_frame,
                stamp,
                self.coarse_target_min_views,
                self.coarse_target_min_confidence,
            )
        )
        self.marker_publisher.publish(
            _target_ray_marker(
                estimate,
                self.global_frame,
                stamp,
                self._latest_camera_origins,
                self.coarse_target_min_views,
                self.coarse_target_min_confidence,
            )
        )

    def _publish_particles(self, stamp) -> None:
        if self.particle_publisher.get_subscription_count() == 0:
            return
        particles = self.particle_filter.particles
        if particles is None:
            return
        header = Header(frame_id=self.global_frame, stamp=stamp)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.particle_publisher.publish(point_cloud2.create_cloud(header, fields, particles))


def _estimate_state_name(state: str) -> str:
    """保留协议状态码, 同时给运行日志提供中文含义"""
    return {
        "PENDING": "等待多视角(PENDING)",
        "TRACKING": "多视角跟踪(TRACKING)",
        "STABLE_VISION": "视觉稳定(STABLE_VISION)",
        "LIDAR_LOCKED": "雷达锁定(LIDAR_LOCKED)",
        "REACHED": "任务完成(REACHED)",
    }.get(state, f"未知状态({state})")


def _mask_array(msg: ObjectMaskWithTf) -> np.ndarray:
    dimensions = msg.object_mask.layout.dim
    if len(dimensions) != 4:
        raise ValueError("object_mask 必须是 B,1,H,W")
    shape = tuple(int(dimension.size) for dimension in dimensions)
    offset = int(msg.object_mask.layout.data_offset)
    data = np.asarray(msg.object_mask.data[offset:], dtype=np.uint8)
    if data.size != math.prod(shape):
        raise ValueError("object_mask 数据长度和 shape 不一致")
    return data.reshape(shape)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _mask_message_summary(msg: ObjectMaskWithTf) -> str:
    """只记录消息结构, 避免复制大体积 Mask"""
    shape = "x".join(str(dimension.size) for dimension in msg.object_mask.layout.dim)
    stamp = _stamp_seconds(msg.header.stamp)
    return (
        f"stamp={stamp:.6f}, frame={msg.header.frame_id}, shape={shape or 'unknown'}, "
        f"data={len(msg.object_mask.data)}, cameras={len(msg.cam_infos)}, "
        f"transforms={len(msg.cam_transforms.transforms)}"
    )


def _xyz_points(cloud: PointCloud2) -> np.ndarray:
    """从异构 PointCloud2 中只提取同为浮点类型的 XYZ 字段"""
    points = point_cloud2.read_points(
        cloud,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.column_stack(
        (points["x"], points["y"], points["z"])
    ).astype(np.float64, copy=False)


def _projected_mask_support(
    world_points: np.ndarray,
    observations: list[CameraObservation],
    dilation_pixels: int,
) -> tuple[np.ndarray, np.ndarray]:
    """把三相机投影合并为唯一点集合, 避免重复计算支持点"""
    visible_union = np.zeros(world_points.shape[0], dtype=bool)
    mask_union = np.zeros(world_points.shape[0], dtype=bool)
    for observation in observations:
        camera_points = (
            observation.rotation_world_from_camera.T
            @ (world_points - observation.translation_world_from_camera).T
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
        visible_union |= inside_image
        target_mask = observation.mask
        if dilation_pixels:
            target_mask = binary_dilation(
                target_mask,
                iterations=dilation_pixels,
            )
        inside_mask = np.zeros(world_points.shape[0], dtype=bool)
        inside_mask[inside_image] = target_mask[
            rows[inside_image],
            cols[inside_image],
        ]
        mask_union |= inside_mask
    return visible_union, mask_union


def _mean_observation_bearing(
    observations: list[CameraObservation],
) -> np.ndarray | None:
    """按相机置信度合并当前 Mask 的世界坐标系方向"""
    weighted_direction = np.zeros(3, dtype=np.float64)
    for observation in observations:
        _, direction, _ = observation.center_ray()
        weighted_direction += max(observation.confidence, 1e-3) * direction
    norm = np.linalg.norm(weighted_direction)
    if norm < 1e-6:
        return None
    return weighted_direction / norm


def _target_surface_measurement(
    points: np.ndarray,
    minimum_support: int,
    reference_position: np.ndarray | None = None,
) -> tuple[np.ndarray, int] | None:
    """先移除局部地面, 再从有效高点簇中选择最近可见前景"""
    measurement, _ = _target_surface_measurement_with_reason(
        points,
        minimum_support,
        reference_position,
    )
    return measurement


def _target_surface_measurement_with_reason(
    points: np.ndarray,
    minimum_support: int,
    reference_position: np.ndarray | None = None,
) -> tuple[tuple[np.ndarray, int] | None, str | None]:
    """返回目标表面测量和失败阶段, 供周期诊断汇总"""
    measurement, reason, _ = _target_surface_measurement_details(
        points,
        minimum_support,
        reference_position,
    )
    return measurement, reason


def _target_surface_measurement_details(
    points: np.ndarray,
    minimum_support: int,
    reference_position: np.ndarray | None = None,
) -> tuple[tuple[np.ndarray, int] | None, str | None, dict[str, int]]:
    """提取连续前景簇并返回各过滤阶段点数"""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    details = {"elevated": 0, "cluster": 0}
    if points.shape[0] < minimum_support:
        return None, "mask_points_insufficient", details

    ground_height = float(np.quantile(points[:, 2], 0.2))
    elevated = points[
        points[:, 2] >= ground_height + _MIN_TARGET_HEIGHT_ABOVE_GROUND
    ]
    details["elevated"] = int(elevated.shape[0])
    elevated_minimum = max(6, minimum_support // 3)
    candidates = elevated if elevated.shape[0] >= elevated_minimum else points
    used_ground_fallback = elevated.shape[0] < elevated_minimum

    tree = cKDTree(candidates[:, :2])
    neighborhoods = tree.query_ball_point(
        candidates[:, :2],
        r=_TARGET_CLUSTER_RADIUS,
    )
    required_cluster_size = min(elevated_minimum, candidates.shape[0])
    valid_neighborhoods = [
        indices for indices in neighborhoods if len(indices) >= required_cluster_size
    ]
    if not valid_neighborhoods:
        failure_reason = (
            "ground_filter_insufficient"
            if used_ground_fallback
            else "foreground_cluster_insufficient"
        )
        return None, failure_reason, details
    if reference_position is None:
        selected_indices = max(valid_neighborhoods, key=len)
    else:
        reference = np.asarray(reference_position, dtype=np.float64).reshape(3)

        def median_range(indices) -> float:
            cluster = candidates[np.asarray(indices, dtype=np.int64)]
            return float(np.median(np.linalg.norm(cluster - reference, axis=1)))

        selected_indices = min(valid_neighborhoods, key=median_range)
    cluster = candidates[np.asarray(selected_indices, dtype=np.int64)]
    details["cluster"] = int(cluster.shape[0])
    return (np.median(cluster, axis=0), int(cluster.shape[0])), None, details


def _counter_summary(counter: Counter[str]) -> str:
    if not counter:
        return "无"
    names = {
        "no_time_matched_cloud": "无时间匹配点云",
        "empty_cloud": "空点云",
        "tf_unavailable": "点云TF不可用",
        "no_points_in_mask": "Mask内无投影点",
        "mask_points_insufficient": "Mask内点数不足",
        "ground_filter_insufficient": "地面过滤后不足",
        "foreground_cluster_insufficient": "前景簇不足",
    }
    return "/".join(
        f"{names.get(reason, reason)}{count}"
        for reason, count in sorted(counter.items())
    )


def _target_marker(
    estimate: CoreTargetEstimate,
    frame_id: str,
    stamp,
    coarse_min_views: int = 2,
    coarse_min_confidence: float = 0.45,
) -> Marker:
    """显示两视角粗目标和稳定目标, 单视角深度不确定时删除标记"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "object_target_estimate"
    marker.id = 0
    marker.type = Marker.SPHERE
    if not _target_marker_visible(
        estimate,
        coarse_min_views,
        coarse_min_confidence,
    ):
        marker.action = Marker.DELETE
        return marker

    marker.action = Marker.ADD
    marker.pose.position.x = float(estimate.position[0])
    marker.pose.position.y = float(estimate.position[1])
    marker.pose.position.z = float(estimate.position[2])
    marker.pose.orientation.w = 1.0
    std = np.sqrt(np.maximum(np.diag(estimate.covariance), 0.0))
    marker.scale.x = min(max(float(2.0 * std[0]), 0.3), _MAX_TARGET_MARKER_SCALE)
    marker.scale.y = min(max(float(2.0 * std[1]), 0.3), _MAX_TARGET_MARKER_SCALE)
    marker.scale.z = min(max(float(2.0 * std[2]), 0.3), _MAX_TARGET_MARKER_SCALE)
    marker.color.a = 0.85
    if estimate.stable:
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
    else:
        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.0
    return marker


def _target_ray_marker(
    estimate: CoreTargetEstimate,
    frame_id: str,
    stamp,
    camera_origins: list[np.ndarray],
    coarse_min_views: int = 2,
    coarse_min_confidence: float = 0.45,
) -> Marker:
    """用论文风格绿色射线连接有效相机和当前目标估计"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "object_target_rays"
    marker.id = 1
    marker.type = Marker.LINE_LIST
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.06
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.9
    if (
        not _target_marker_visible(
            estimate,
            coarse_min_views,
            coarse_min_confidence,
        )
        or not camera_origins
    ):
        marker.action = Marker.DELETE
        return marker

    marker.action = Marker.ADD
    target = Point(
        x=float(estimate.position[0]),
        y=float(estimate.position[1]),
        z=float(estimate.position[2]),
    )
    for origin in camera_origins:
        marker.points.append(
            Point(
                x=float(origin[0]),
                y=float(origin[1]),
                z=float(origin[2]),
            )
        )
        marker.points.append(target)
    return marker


def _target_marker_visible(
    estimate: CoreTargetEstimate,
    coarse_min_views: int = 2,
    coarse_min_confidence: float = 0.45,
) -> bool:
    """黄色标记和导航接管使用相同的粗目标证据规则"""
    coarse_ready = coarse_target_evidence_ready(
        estimate.state,
        estimate.accepted_views,
        estimate.confidence,
        coarse_min_views,
        coarse_min_confidence,
    )
    return bool(estimate.stable or coarse_ready)


def main(args=None):
    faulthandler.enable(all_threads=True)
    rclpy.init(args=args)
    node = ObjectTargetFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        node.get_logger().fatal(f"目标融合主循环异常\n{traceback.format_exc()}")
        raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
