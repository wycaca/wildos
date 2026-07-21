from __future__ import annotations

from collections import deque
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
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
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


_LIDAR_BUFFER_SIZE = 40
_MAX_TARGET_MARKER_SCALE = 1.5
_MIN_TARGET_HEIGHT_ABOVE_GROUND = 0.12
_TARGET_CLUSTER_RADIUS = 0.75


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
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("particle_count", 1500)
        self.declare_parameter("max_depth", 100.0)
        self.declare_parameter("stable_min_confidence", 0.6)
        self.declare_parameter("lidar_min_points", 30)
        self.declare_parameter("max_lidar_age_sec", 3.0)
        self.declare_parameter("diagnostics_log_period_sec", 60.0)
        self.declare_parameter("slow_callback_warning_ms", 500.0)

        self.max_depth = max(float(self.get_parameter("max_depth").value), 2.0)
        particle_config = ParticleFilterConfig(
            particle_count=max(int(self.get_parameter("particle_count").value), 100),
            max_depth=self.max_depth,
            stable_min_confidence=float(self.get_parameter("stable_min_confidence").value),
        )
        self.particle_filter = TargetParticleFilter(particle_config)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.lidar_min_points = max(int(self.get_parameter("lidar_min_points").value), 1)
        self.max_lidar_age_sec = max(float(self.get_parameter("max_lidar_age_sec").value), 0.0)
        self.lidar_buffer: deque[tuple[float, PointCloud2]] = deque(
            maxlen=_LIDAR_BUFFER_SIZE
        )
        self._last_logged_state = ""
        self._latest_camera_origins: list[np.ndarray] = []
        self._mask_stage = "idle"
        self._mask_received = 0
        self._mask_processed = 0
        self._mask_empty = 0
        self._mask_errors = 0
        self._mask_ignored_reached = 0
        self._lidar_matched = 0
        self._lidar_refined = 0
        self._last_slow_warning = 0.0
        self._mask_rate = EventRate()
        self._lidar_rate = EventRate()
        self._timings = {
            name: TimingWindow()
            for name in ("decode", "vision", "lidar", "publish", "total")
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
            10,
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
        diagnostics_period = max(
            float(self.get_parameter("diagnostics_log_period_sec").value),
            10.0,
        )
        self.create_timer(diagnostics_period, self._log_health)

        self.get_logger().info(
            "目标融合已启动, "
            f"Mask话题={self.get_parameter('object_mask_topic').value}, "
            f"雷达话题={self.get_parameter('lidar_topic').value}, "
            f"估计话题={self.get_parameter('target_estimate_topic').value}, "
            f"最大深度={self.max_depth:.1f}m, "
            f"python={sys.executable}, numpy={np.__version__}, scipy={scipy.__version__}"
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        self.lidar_buffer.append((_stamp_seconds(msg.header.stamp), msg))
        self._lidar_rate.tick()

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
        callback_started = time.perf_counter()
        self._mask_received += 1
        self._mask_rate.tick()
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

        self._set_mask_stage("update_vision", f"observations={len(observations)}")
        stage_started = time.perf_counter()
        estimate = self.particle_filter.update_vision(observations)
        self._timings["vision"].add_seconds(time.perf_counter() - stage_started)
        self._set_mask_stage("match_lidar", f"buffer={len(self.lidar_buffer)}")
        lidar_msg = self._nearest_lidar(msg.header.stamp)
        if lidar_msg is not None:
            self._lidar_matched += 1
            self._set_mask_stage(
                "project_lidar",
                f"frame={lidar_msg.header.frame_id}, points={lidar_msg.width * lidar_msg.height}",
            )
            stage_started = time.perf_counter()
            lidar_measurement = self._lidar_measurement(observations, lidar_msg)
            self._timings["lidar"].add_seconds(time.perf_counter() - stage_started)
            if lidar_measurement is not None:
                position, support = lidar_measurement
                self._lidar_refined += 1
                self._set_mask_stage("update_lidar", f"support={support}")
                estimate = self.particle_filter.update_lidar(position, support)

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

    def _set_mask_stage(self, stage: str, detail: str = "") -> None:
        """首帧逐阶段记录, 后续由周期统计报告当前阶段"""
        self._mask_stage = stage
        if self._mask_received != 1:
            return
        suffix = f", {detail}" if detail else ""
        self.get_logger().debug(f"首帧目标 Mask 处理阶段={stage}{suffix}")

    def _warn_if_slow(self, elapsed_seconds: float, stage: str) -> None:
        """Report sustained-risk callbacks without logging every slow frame"""
        threshold_ms = float(self.get_parameter("slow_callback_warning_ms").value)
        now = time.monotonic()
        if elapsed_seconds * 1000.0 < threshold_ms or now - self._last_slow_warning < 30.0:
            return
        self._last_slow_warning = now
        self.get_logger().warn(
            f"目标融合处理耗时偏高, total={elapsed_seconds * 1000.0:.1f}ms, "
            f"threshold={threshold_ms:.1f}ms, stage={stage}"
        )

    def _log_health(self) -> None:
        """周期报告融合存活状态和最近回调阶段"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._timings.items()
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
            f"融合状态={_estimate_state_name(self.particle_filter.state)}, "
            f"总耗时=平均{total.average_ms:.0f}/95%上限{total.p95_ms:.0f}/"
            f"最大{total.maximum_ms:.0f}ms, "
            "阶段平均耗时="
            f"Mask解析{summaries['decode'].average_ms:.0f}ms/"
            f"视觉融合{summaries['vision'].average_ms:.0f}ms/"
            f"雷达投影{summaries['lidar'].average_ms:.0f}ms/"
            f"结果发布{summaries['publish'].average_ms:.0f}ms"
        )

    def _log_estimate_state(self, estimate: CoreTargetEstimate) -> None:
        if estimate.state == self._last_logged_state:
            return
        self._last_logged_state = estimate.state
        self.get_logger().info(
            f"目标融合状态变化, 状态={_estimate_state_name(estimate.state)}, "
            f"位置=({estimate.position[0]:.2f}, {estimate.position[1]:.2f}, "
            f"{estimate.position[2]:.2f}), 置信度={estimate.confidence:.2f}, "
            f"有效视角={estimate.accepted_views}, "
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

    def _nearest_lidar(self, stamp) -> PointCloud2 | None:
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
        return lidar_msg if age <= self.max_lidar_age_sec else None

    def _lidar_measurement(
        self,
        observations: list[CameraObservation],
        lidar_msg: PointCloud2,
    ) -> tuple[np.ndarray, int] | None:
        """把点云转到目标全局坐标系, 仅保留投影落入任一目标 Mask 的点"""
        points = _xyz_points(lidar_msg)
        if points.size == 0:
            return None
        world_points = self._points_in_global_frame(points, lidar_msg)
        if world_points is None:
            return None

        mask_points = []
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
            inside_mask = np.zeros(world_points.shape[0], dtype=bool)
            inside_mask[inside_image] = observation.mask[rows[inside_image], cols[inside_image]]
            mask_points.append(world_points[inside_mask])

        supported = [points for points in mask_points if points.size > 0]
        if not supported:
            return None
        supported_points = np.vstack(supported)
        if supported_points.shape[0] < self.lidar_min_points:
            return None

        measurement = _target_surface_measurement(
            supported_points,
            self.lidar_min_points,
            reference_position=np.mean(
                [observation.translation_world_from_camera for observation in observations],
                axis=0,
            ),
        )
        if measurement is None:
            return None
        return measurement

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
        self.estimate_publisher.publish(msg)

    def _publish_markers(self, estimate: CoreTargetEstimate, stamp) -> None:
        """在同一 Marker 话题发布目标球和观测射线, 避免增加重复可视化话题"""
        self.marker_publisher.publish(
            _target_marker(estimate, self.global_frame, stamp)
        )
        self.marker_publisher.publish(
            _target_ray_marker(
                estimate,
                self.global_frame,
                stamp,
                self._latest_camera_origins,
            )
        )

    def _publish_particles(self, stamp) -> None:
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


def _target_surface_measurement(
    points: np.ndarray,
    minimum_support: int,
    reference_position: np.ndarray | None = None,
) -> tuple[np.ndarray, int] | None:
    """先移除局部地面, 再从有效高点簇中选择最近可见前景"""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] < minimum_support:
        return None

    ground_height = float(np.quantile(points[:, 2], 0.2))
    elevated = points[
        points[:, 2] >= ground_height + _MIN_TARGET_HEIGHT_ABOVE_GROUND
    ]
    elevated_minimum = max(6, minimum_support // 3)
    candidates = elevated if elevated.shape[0] >= elevated_minimum else points

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
        return None
    if reference_position is None:
        selected_indices = max(valid_neighborhoods, key=len)
    else:
        reference = np.asarray(reference_position, dtype=np.float64).reshape(3)

        def median_range(indices) -> float:
            cluster = candidates[np.asarray(indices, dtype=np.int64)]
            return float(np.median(np.linalg.norm(cluster - reference, axis=1)))

        selected_indices = min(valid_neighborhoods, key=median_range)
    cluster = candidates[np.asarray(selected_indices, dtype=np.int64)]
    return np.median(cluster, axis=0), int(cluster.shape[0])


def _target_marker(estimate: CoreTargetEstimate, frame_id: str, stamp) -> Marker:
    """显示两视角粗目标和稳定目标, 单视角深度不确定时删除标记"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "object_target_estimate"
    marker.id = 0
    marker.type = Marker.SPHERE
    if not _target_marker_visible(estimate):
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
    if not _target_marker_visible(estimate) or not camera_origins:
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


def _target_marker_visible(estimate: CoreTargetEstimate) -> bool:
    coarse_ready = estimate.state == "TRACKING" and estimate.accepted_views >= 2
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
