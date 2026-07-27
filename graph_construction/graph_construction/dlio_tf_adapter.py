from __future__ import annotations

import copy
import math
from collections import deque
from dataclasses import dataclass
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from graph_construction.performance_stats import EventRate, TimingWindow


_DIAGNOSTICS_LOG_PERIOD_SEC = 30.0


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True)
class OdometryHealthEvaluation:
    """区分参考漂移告警和需要断流的硬异常"""

    metrics: tuple[float, float, float]
    jump_metrics: tuple[float, float]
    warning_healthy: bool
    hard_healthy: bool
    recovery_healthy: bool
    instantaneous_healthy: bool
    finite: bool


@dataclass(frozen=True)
class HeadingConsistencySummary:
    """汇总启动方向差和对齐后的累计变化"""

    initial_raw_error_deg: float
    current_aligned_error_deg: float
    cumulative_change_deg: float
    maximum_aligned_error_deg: float
    samples: int


class HeadingConsistencyMonitor:
    """区分固定启动方向差和运行中的累计 yaw 漂移"""

    def __init__(self) -> None:
        self._initial_raw_error_deg: float | None = None
        self._previous_aligned_error_deg: float | None = None
        self._current_aligned_error_deg = 0.0
        self._cumulative_change_deg = 0.0
        self._maximum_aligned_error_deg = 0.0
        self._samples = 0

    def update(
        self,
        local: Odometry,
        aligned: Odometry,
        reference: Odometry | None,
    ) -> HeadingConsistencySummary | None:
        """使用同一时刻数据更新方向差, 不参与健康门控"""
        if reference is None:
            return self.summary()

        raw_error = _signed_yaw_error_deg(local, reference)
        aligned_error = _signed_yaw_error_deg(aligned, reference)
        if self._initial_raw_error_deg is None:
            self._initial_raw_error_deg = raw_error
        if self._previous_aligned_error_deg is not None:
            self._cumulative_change_deg += _normalize_angle_deg(
                aligned_error - self._previous_aligned_error_deg
            )
        self._previous_aligned_error_deg = aligned_error
        self._current_aligned_error_deg = aligned_error
        self._maximum_aligned_error_deg = max(
            self._maximum_aligned_error_deg,
            abs(aligned_error),
        )
        self._samples += 1
        return self.summary()

    def summary(self) -> HeadingConsistencySummary | None:
        if self._initial_raw_error_deg is None:
            return None
        return HeadingConsistencySummary(
            initial_raw_error_deg=self._initial_raw_error_deg,
            current_aligned_error_deg=self._current_aligned_error_deg,
            cumulative_change_deg=self._cumulative_change_deg,
            maximum_aligned_error_deg=self._maximum_aligned_error_deg,
            samples=self._samples,
        )


class OdometryHealthMonitor:
    """监测参考残差变化, 避免累计漂移永久关闭里程计链路"""

    def __init__(
        self,
        max_position_error: float,
        max_orientation_error_deg: float,
        max_linear_speed: float,
        max_position_jump: float,
        max_orientation_jump_deg: float,
        hard_max_position_error: float,
        hard_max_orientation_error_deg: float,
        recovery_ratio: float,
        jump_reset_sec: float,
    ) -> None:
        self.max_position_error = max(0.0, float(max_position_error))
        self.max_orientation_error_deg = max(
            0.0,
            float(max_orientation_error_deg),
        )
        self.max_linear_speed = max(0.0, float(max_linear_speed))
        self.max_position_jump = max(0.0, float(max_position_jump))
        self.max_orientation_jump_deg = max(
            0.0,
            float(max_orientation_jump_deg),
        )
        self.hard_max_position_error = max(
            self.max_position_error,
            float(hard_max_position_error),
        )
        self.hard_max_orientation_error_deg = max(
            self.max_orientation_error_deg,
            float(hard_max_orientation_error_deg),
        )
        self.recovery_ratio = min(max(float(recovery_ratio), 0.1), 1.0)
        self.jump_reset_ns = int(max(0.0, float(jump_reset_sec)) * 1.0e9)
        self._previous_residual: RigidTransform | None = None
        self._previous_stamp_ns: int | None = None

    def evaluate(
        self,
        aligned: Odometry,
        reference: Odometry | None,
    ) -> OdometryHealthEvaluation:
        """累计参考误差只告警, 突变和灾难性误差才触发硬门控"""
        metrics = odometry_health_metrics(aligned, reference)
        finite = _odometry_values_are_finite(aligned, metrics)
        jump_metrics = self._residual_jump(aligned, reference)
        warning_healthy = (
            finite
            and metrics[0] <= self.max_position_error
            and metrics[1] <= self.max_orientation_error_deg
        )
        instantaneous_healthy = (
            finite
            and jump_metrics[0] <= self.max_position_jump
            and jump_metrics[1] <= self.max_orientation_jump_deg
        )
        hard_healthy = self._within_hard_limits(
            metrics,
            jump_metrics,
            ratio=1.0,
            finite=finite,
        )
        recovery_healthy = self._within_hard_limits(
            metrics,
            jump_metrics,
            ratio=self.recovery_ratio,
            finite=finite,
        )
        return OdometryHealthEvaluation(
            metrics=metrics,
            jump_metrics=jump_metrics,
            warning_healthy=warning_healthy,
            hard_healthy=hard_healthy,
            recovery_healthy=recovery_healthy,
            instantaneous_healthy=instantaneous_healthy,
            finite=finite,
        )

    def _within_hard_limits(
        self,
        metrics: tuple[float, float, float],
        jump_metrics: tuple[float, float],
        ratio: float,
        finite: bool,
    ) -> bool:
        """检查速度、瞬时残差跳变和灾难性累计偏差"""
        return (
            finite
            and metrics[0] <= self.hard_max_position_error * ratio
            and metrics[1] <= self.hard_max_orientation_error_deg * ratio
            and metrics[2] <= self.max_linear_speed * ratio
            and jump_metrics[0] <= self.max_position_jump * ratio
            and jump_metrics[1] <= self.max_orientation_jump_deg * ratio
        )

    def _residual_jump(
        self,
        aligned: Odometry,
        reference: Odometry | None,
    ) -> tuple[float, float]:
        """计算相邻帧参考残差的位姿变化"""
        if reference is None:
            self._previous_residual = None
            self._previous_stamp_ns = None
            return (0.0, 0.0)

        residual = _compose(_inverse(_pose_to_rigid(reference)), _pose_to_rigid(aligned))
        stamp_ns = _stamp_nanoseconds(aligned)
        if self._should_reset_jump_history(stamp_ns):
            jump_metrics = (0.0, 0.0)
        else:
            delta = _compose(_inverse(self._previous_residual), residual)
            jump_metrics = (
                math.sqrt(sum(value * value for value in delta.translation)),
                _rotation_angle_deg(delta.rotation),
            )
        self._previous_residual = residual
        self._previous_stamp_ns = stamp_ns
        return jump_metrics

    def _should_reset_jump_history(self, stamp_ns: int) -> bool:
        """时间回退或长时间断帧后重新建立突变基线"""
        if self._previous_residual is None or self._previous_stamp_ns is None:
            return True
        delta_ns = stamp_ns - self._previous_stamp_ns
        if delta_ns <= 0:
            return True
        return self.jump_reset_ns > 0 and delta_ns > self.jump_reset_ns


class HealthStateFilter:
    """Debounce transient odometry errors and require stable recovery"""

    def __init__(self, unhealthy_confirm_frames: int, healthy_confirm_frames: int) -> None:
        self.unhealthy_confirm_frames = max(unhealthy_confirm_frames, 1)
        self.healthy_confirm_frames = max(healthy_confirm_frames, 1)
        self.state: bool | None = None
        self._bad_count = 0
        self._good_count = 0

    def update(self, healthy: bool, force_unhealthy: bool = False) -> bool | None:
        """Return a new stable state only when a transition is confirmed"""
        if self.state is None:
            self.state = healthy
            return self.state
        if force_unhealthy and self.state:
            self.state = False
            self._bad_count = 0
            self._good_count = 0
            return self.state
        if self.state:
            self._good_count = 0
            self._bad_count = 0 if healthy else self._bad_count + 1
            if self._bad_count < self.unhealthy_confirm_frames:
                return None
            self.state = False
            self._bad_count = 0
            return self.state
        self._bad_count = 0
        self._good_count = self._good_count + 1 if healthy else 0
        if self._good_count < self.healthy_confirm_frames:
            return None
        self.state = True
        self._good_count = 0
        return self.state


def odom_to_transform(
    msg: Odometry,
    odom_frame: str,
    base_frame: str,
) -> TransformStamped:
    """Convert one DLIO odom state into an odom to base transform"""
    transform = TransformStamped()
    transform.header = copy.deepcopy(msg.header)
    transform.header.frame_id = odom_frame
    transform.child_frame_id = base_frame
    transform.transform.translation.x = msg.pose.pose.position.x
    transform.transform.translation.y = msg.pose.pose.position.y
    transform.transform.translation.z = msg.pose.pose.position.z
    transform.transform.rotation = copy.deepcopy(msg.pose.pose.orientation)
    return transform


def alignment_from_odometry(
    reference: Odometry,
    local: Odometry,
) -> RigidTransform:
    """Compute the fixed global to local-frame alignment from matching poses"""
    global_pose = _pose_to_rigid(reference)
    local_pose = _pose_to_rigid(local)
    return _compose(global_pose, _inverse(local_pose))


def align_odometry(
    msg: Odometry,
    alignment: RigidTransform,
    global_frame: str,
    base_frame: str,
) -> Odometry:
    """Express a local DLIO pose in the anchored global frame"""
    output = copy.deepcopy(msg)
    output.header.frame_id = global_frame
    output.child_frame_id = base_frame
    aligned_pose = _compose(alignment, _pose_to_rigid(msg))
    output.pose.pose.position.x = aligned_pose.translation[0]
    output.pose.pose.position.y = aligned_pose.translation[1]
    output.pose.pose.position.z = aligned_pose.translation[2]
    output.pose.pose.orientation = Quaternion(
        x=aligned_pose.rotation[0],
        y=aligned_pose.rotation[1],
        z=aligned_pose.rotation[2],
        w=aligned_pose.rotation[3],
    )
    return output


def alignment_to_transform(
    alignment: RigidTransform,
    stamp,
    global_frame: str,
    local_frame: str,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = copy.deepcopy(stamp)
    transform.header.frame_id = global_frame
    transform.child_frame_id = local_frame
    transform.transform.translation.x = alignment.translation[0]
    transform.transform.translation.y = alignment.translation[1]
    transform.transform.translation.z = alignment.translation[2]
    transform.transform.rotation = Quaternion(
        x=alignment.rotation[0],
        y=alignment.rotation[1],
        z=alignment.rotation[2],
        w=alignment.rotation[3],
    )
    return transform


def relay_extrinsic_transforms(
    msg: TFMessage,
    odom_frame: str,
    base_frame: str,
) -> TFMessage:
    """保留唯一的传感器外参, 丢弃 DLIO 原始位姿 TF"""
    output = TFMessage()
    seen_frames: set[tuple[str, str]] = set()
    for transform in msg.transforms:
        if _is_pose_transform(transform, odom_frame, base_frame):
            continue
        frame_pair = (
            transform.header.frame_id.strip("/"),
            transform.child_frame_id.strip("/"),
        )
        if frame_pair in seen_frames:
            continue
        seen_frames.add(frame_pair)
        output.transforms.append(copy.deepcopy(transform))
    return output


def _is_pose_transform(
    transform: TransformStamped,
    odom_frame: str,
    base_frame: str,
) -> bool:
    return (
        transform.header.frame_id.strip("/") == odom_frame.strip("/")
        and transform.child_frame_id.strip("/") == base_frame.strip("/")
    )


def _pose_to_rigid(msg: Odometry) -> RigidTransform:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return RigidTransform(
        translation=(position.x, position.y, position.z),
        rotation=_normalize_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        ),
    )


def _compose(parent: RigidTransform, child: RigidTransform) -> RigidTransform:
    rotated_translation = _rotate_vector(parent.rotation, child.translation)
    return RigidTransform(
        translation=tuple(
            parent.translation[index] + rotated_translation[index]
            for index in range(3)
        ),
        rotation=_normalize_quaternion(
            _multiply_quaternions(parent.rotation, child.rotation)
        ),
    )


def _inverse(transform: RigidTransform) -> RigidTransform:
    inverse_rotation = (
        -transform.rotation[0],
        -transform.rotation[1],
        -transform.rotation[2],
        transform.rotation[3],
    )
    inverse_translation = _rotate_vector(
        inverse_rotation,
        tuple(-value for value in transform.translation),
    )
    return RigidTransform(inverse_translation, inverse_rotation)


def _multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(
    rotation: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = rotation
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _normalize_quaternion(
    rotation: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in rotation))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in rotation)


def _rotation_angle_deg(
    rotation: tuple[float, float, float, float],
) -> float:
    """返回单位四元数表示的最小旋转角"""
    normalized = _normalize_quaternion(rotation)
    return math.degrees(2.0 * math.acos(min(1.0, abs(normalized[3]))))


def _signed_yaw_error_deg(
    estimated: Odometry,
    reference: Odometry,
) -> float:
    """返回 estimated 相对 reference 的有符号 yaw 差"""
    return _normalize_angle_deg(
        _odometry_yaw_deg(estimated) - _odometry_yaw_deg(reference)
    )


def _odometry_yaw_deg(msg: Odometry) -> float:
    orientation = _normalize_quaternion(
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
    )
    x, y, z, w = orientation
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return math.degrees(yaw)


def _normalize_angle_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _stamp_nanoseconds(msg: Odometry) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def odometry_health_metrics(
    aligned: Odometry,
    reference: Odometry | None,
) -> tuple[float, float, float]:
    """Return position error, orientation error and linear speed"""
    position = aligned.pose.pose.position
    linear = aligned.twist.twist.linear
    speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
    if reference is None:
        return (0.0, 0.0, speed)

    reference_position = reference.pose.pose.position
    position_error = math.sqrt(
        (position.x - reference_position.x) ** 2
        + (position.y - reference_position.y) ** 2
        + (position.z - reference_position.z) ** 2
    )
    aligned_orientation = aligned.pose.pose.orientation
    reference_orientation = reference.pose.pose.orientation
    aligned_rotation = _normalize_quaternion(
        (
            aligned_orientation.x,
            aligned_orientation.y,
            aligned_orientation.z,
            aligned_orientation.w,
        )
    )
    reference_rotation = _normalize_quaternion(
        (
            reference_orientation.x,
            reference_orientation.y,
            reference_orientation.z,
            reference_orientation.w,
        )
    )
    dot = abs(
        sum(
            aligned_rotation[index] * reference_rotation[index]
            for index in range(4)
        )
    )
    orientation_error = math.degrees(2.0 * math.acos(min(1.0, dot)))
    return (position_error, orientation_error, speed)


def _odometry_values_are_finite(
    aligned: Odometry,
    metrics: tuple[float, float, float],
) -> bool:
    values = (
        aligned.pose.pose.position.x,
        aligned.pose.pose.position.y,
        aligned.pose.pose.position.z,
        aligned.pose.pose.orientation.x,
        aligned.pose.pose.orientation.y,
        aligned.pose.pose.orientation.z,
        aligned.pose.pose.orientation.w,
        *metrics,
    )
    return all(math.isfinite(value) for value in values)


class DlioTfAdapter(Node):
    """Publish globally anchored DLIO TF and odom"""

    def __init__(self) -> None:
        super().__init__("dlio_tf_adapter")
        self.declare_parameter(
            "input_odom_topic",
            "/spot1/dlio/odom_node/odom",
        )
        self.declare_parameter(
            "input_raw_tf_topic",
            "/spot1/dlio/odom_node/tf_raw",
        )
        self.declare_parameter("reference_odom_topic", "")
        self.declare_parameter(
            "output_aligned_odom_topic",
            "/spot1/dlio/aligned_odom",
        )
        self.declare_parameter("output_tf_topic", "/spot1/tf")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("local_frame", "dlio_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("reference_max_time_delta", 0.1)
        self.declare_parameter("alignment_delay", 0.0)
        self.declare_parameter("health_topic", "/spot1/dlio/odom_node/healthy")
        self.declare_parameter("max_position_error", 0.5)
        self.declare_parameter("max_orientation_error_deg", 10.0)
        self.declare_parameter("max_linear_speed", 5.0)
        self.declare_parameter("max_position_jump", 0.5)
        self.declare_parameter("max_orientation_jump_deg", 20.0)
        self.declare_parameter("hard_max_position_error", 5.0)
        self.declare_parameter("hard_max_orientation_error_deg", 60.0)
        self.declare_parameter("health_jump_reset_sec", 0.5)
        self.declare_parameter("unhealthy_confirm_frames", 3)
        self.declare_parameter("healthy_confirm_frames", 5)
        self.declare_parameter("health_recovery_ratio", 0.8)
        self.input_odom_topic = str(
            self.get_parameter("input_odom_topic").value
        )
        self.input_raw_tf_topic = str(
            self.get_parameter("input_raw_tf_topic").value
        )
        self.reference_odom_topic = str(
            self.get_parameter("reference_odom_topic").value
        )
        self.output_aligned_odom_topic = str(
            self.get_parameter("output_aligned_odom_topic").value
        )
        self.output_tf_topic = str(self.get_parameter("output_tf_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.local_frame = str(self.get_parameter("local_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.reference_max_time_delta_ns = int(
            float(self.get_parameter("reference_max_time_delta").value) * 1.0e9
        )
        self.alignment_delay_ns = int(
            float(self.get_parameter("alignment_delay").value) * 1.0e9
        )
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.max_position_error = float(
            self.get_parameter("max_position_error").value
        )
        self.max_orientation_error_deg = float(
            self.get_parameter("max_orientation_error_deg").value
        )
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_position_jump = float(
            self.get_parameter("max_position_jump").value
        )
        self.max_orientation_jump_deg = float(
            self.get_parameter("max_orientation_jump_deg").value
        )
        self.hard_max_position_error = float(
            self.get_parameter("hard_max_position_error").value
        )
        self.hard_max_orientation_error_deg = float(
            self.get_parameter("hard_max_orientation_error_deg").value
        )
        self.health_jump_reset_sec = float(
            self.get_parameter("health_jump_reset_sec").value
        )
        self.health_recovery_ratio = min(
            max(float(self.get_parameter("health_recovery_ratio").value), 0.1),
            1.0,
        )
        self.health_filter = HealthStateFilter(
            int(self.get_parameter("unhealthy_confirm_frames").value),
            int(self.get_parameter("healthy_confirm_frames").value),
        )
        self.health_monitor = OdometryHealthMonitor(
            max_position_error=self.max_position_error,
            max_orientation_error_deg=self.max_orientation_error_deg,
            max_linear_speed=self.max_linear_speed,
            max_position_jump=self.max_position_jump,
            max_orientation_jump_deg=self.max_orientation_jump_deg,
            hard_max_position_error=self.hard_max_position_error,
            hard_max_orientation_error_deg=self.hard_max_orientation_error_deg,
            recovery_ratio=self.health_recovery_ratio,
            jump_reset_sec=self.health_jump_reset_sec,
        )
        self.reference_messages: deque[Odometry] = deque(maxlen=500)
        self.alignment: RigidTransform | None = None
        self.first_local_stamp_ns: int | None = None
        self._logged_pose = False
        self._logged_extrinsics = False
        self._logged_waiting = False
        self._health_status: bool | None = None
        self._health_episodes = 0
        self._raw_unhealthy_samples = 0
        self._reference_warning_samples = 0
        self._last_reference_warning = 0.0
        self._max_health_metrics = [0.0, 0.0, 0.0]
        self._max_jump_metrics = [0.0, 0.0]
        self._callback_timing = TimingWindow()
        self._input_rate = EventRate()
        self._heading_monitor = HeadingConsistencyMonitor()
        self.extrinsics_ready = not bool(self.reference_odom_topic)

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_publisher = self.create_publisher(
            TFMessage,
            self.output_tf_topic,
            tf_qos,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.output_aligned_odom_topic,
            10,
        )
        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.health_publisher = self.create_publisher(
            Bool,
            self.health_topic,
            health_qos,
        )
        self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self._on_odom,
            10,
        )
        self.create_subscription(
            TFMessage,
            self.input_raw_tf_topic,
            self._on_raw_tf,
            tf_qos,
        )
        if self.reference_odom_topic:
            self.create_subscription(
                Odometry,
                self.reference_odom_topic,
                self._on_reference_odom,
                50,
            )
        else:
            self.alignment = RigidTransform(
                translation=(0.0, 0.0, 0.0),
                rotation=(0.0, 0.0, 0.0, 1.0),
            )
        self.create_timer(_DIAGNOSTICS_LOG_PERIOD_SEC, self._report_diagnostics)
        self.get_logger().info(
            f"DLIO TF adapter 已启动, odom={self.input_odom_topic}, "
            f"reference={self.reference_odom_topic or '<identity>'}, "
            f"aligned_odom={self.output_aligned_odom_topic}, "
            f"frames={self.global_frame}->{self.local_frame}"
            f"->{self.base_frame}"
        )

    def _on_reference_odom(self, msg: Odometry) -> None:
        self.reference_messages.append(msg)

    def _on_odom(self, msg: Odometry) -> None:
        callback_started = time.perf_counter()
        self._input_rate.tick()
        if self.alignment is None:
            if not self.extrinsics_ready:
                if not self._logged_waiting:
                    self.get_logger().info(
                        "等待 DLIO IMU 标定完成并发布传感器外参"
                    )
                    self._logged_waiting = True
                self._callback_timing.add_seconds(time.perf_counter() - callback_started)
                return
            local_stamp_ns = _stamp_nanoseconds(msg)
            if self.first_local_stamp_ns is None:
                self.first_local_stamp_ns = local_stamp_ns
            elapsed_ns = local_stamp_ns - self.first_local_stamp_ns
            if elapsed_ns < self.alignment_delay_ns:
                if not self._logged_waiting:
                    delay_sec = self.alignment_delay_ns / 1.0e9
                    self.get_logger().info(
                        f"等待 DLIO IMU 标定稳定后再对齐, delay={delay_sec:.1f}s"
                    )
                    self._logged_waiting = True
                self._callback_timing.add_seconds(time.perf_counter() - callback_started)
                return
            reference = self._nearest_reference(msg)
            if reference is None:
                if not self._logged_waiting:
                    self.get_logger().warn(
                        "等待与 DLIO 同时刻的参考 odom 以初始化全局坐标"
                    )
                    self._logged_waiting = True
                self._callback_timing.add_seconds(time.perf_counter() - callback_started)
                return
            self.alignment = alignment_from_odometry(reference, msg)
            stamp_delta_ns = abs(
                _stamp_nanoseconds(reference) - _stamp_nanoseconds(msg)
            )
            delta_ms = stamp_delta_ns / 1.0e6
            translation = self.alignment.translation
            initial_heading_error = _signed_yaw_error_deg(msg, reference)
            self.get_logger().info(
                "已锁定 DLIO 到全局坐标的启动对齐, "
                f"delta={delta_ms:.1f}ms, translation=({translation[0]:.3f}, "
                f"{translation[1]:.3f}, {translation[2]:.3f}), "
                f"原始方向差=DLIO-参考{initial_heading_error:.2f}度"
            )

        aligned_odom = align_odometry(
            msg,
            self.alignment,
            self.global_frame,
            self.base_frame,
        )
        reference = self._nearest_reference(msg)
        self._heading_monitor.update(msg, aligned_odom, reference)
        evaluation = self.health_monitor.evaluate(aligned_odom, reference)
        metrics = evaluation.metrics
        self._max_health_metrics = [
            max(previous, current)
            for previous, current in zip(self._max_health_metrics, metrics)
        ]
        self._max_jump_metrics = [
            max(previous, current)
            for previous, current in zip(
                self._max_jump_metrics,
                evaluation.jump_metrics,
            )
        ]
        if not evaluation.hard_healthy:
            self._raw_unhealthy_samples += 1
        if not evaluation.warning_healthy:
            self._reference_warning_samples += 1
            self._warn_reference_drift(metrics)
        candidate = (
            evaluation.recovery_healthy
            if self.health_filter.state is False
            else evaluation.hard_healthy
        )
        transition = self.health_filter.update(
            candidate,
            force_unhealthy=not evaluation.instantaneous_healthy,
        )
        if transition is not None:
            self._publish_health(transition)
            if not transition:
                self._health_episodes += 1
                self.get_logger().warn(
                    "DLIO 位姿异常已确认, 已暂停 odom、TF 和点云输出, "
                    f"position_error={metrics[0]:.3f}m, "
                    f"orientation_error={metrics[1]:.2f}deg, "
                    f"speed={metrics[2]:.3f}m/s, "
                    f"position_jump={evaluation.jump_metrics[0]:.3f}m, "
                    f"orientation_jump={evaluation.jump_metrics[1]:.2f}deg"
                )
            elif self._health_episodes:
                self.get_logger().info(
                    "DLIO 位姿持续稳定, 已恢复 odom、TF 和点云输出"
                )
        if self.health_filter.state is False:
            self._callback_timing.add_seconds(time.perf_counter() - callback_started)
            return
        alignment_tf = alignment_to_transform(
            self.alignment,
            msg.header.stamp,
            self.global_frame,
            self.local_frame,
        )
        local_pose_tf = odom_to_transform(
            msg,
            self.local_frame,
            self.base_frame,
        )
        self.odom_publisher.publish(aligned_odom)
        self.tf_publisher.publish(
            TFMessage(transforms=[alignment_tf, local_pose_tf])
        )
        if not self._logged_pose:
            position = aligned_odom.pose.pose.position
            self.get_logger().info(
                "已发布第一帧全局对齐 odom 和 TF, "
                f"position=({position.x:.3f}, {position.y:.3f}, "
                f"{position.z:.3f})"
            )
            self._logged_pose = True
        self._callback_timing.add_seconds(time.perf_counter() - callback_started)

    def _warn_reference_drift(
        self,
        metrics: tuple[float, float, float],
    ) -> None:
        """参考累计漂移只做低频告警, 不直接中断 DLIO 输出"""
        now = time.monotonic()
        if now - self._last_reference_warning < 30.0:
            return
        self._last_reference_warning = now
        self.get_logger().warn(
            "DLIO 与参考里程计存在累计漂移, 继续输出并监测瞬时稳定性, "
            f"position_error={metrics[0]:.3f}m, "
            f"orientation_error={metrics[1]:.2f}deg"
        )

    def _publish_health(self, healthy: bool) -> bool:
        if self._health_status == healthy:
            return False
        self.health_publisher.publish(Bool(data=healthy))
        self._health_status = healthy
        return True

    def _report_diagnostics(self) -> None:
        """Report bounded health and callback metrics at low frequency"""
        timing = self._callback_timing.summary(reset=True)
        input_rate = self._input_rate.sample(reset=True)
        maxima = self._max_health_metrics
        jump_maxima = self._max_jump_metrics
        heading = self._heading_monitor.summary()
        heading_text = "方向差=无同时刻参考"
        if heading is not None:
            heading_text = (
                f"方向差=启动原始{heading.initial_raw_error_deg:.2f}度/"
                f"当前对齐后{heading.current_aligned_error_deg:.2f}度/"
                f"累计变化{heading.cumulative_change_deg:.2f}度/"
                f"最大对齐后{heading.maximum_aligned_error_deg:.2f}度"
            )
        self.get_logger().info(
            "DLIO 位姿状态, "
            f"输入频率={input_rate:.1f}Hz, "
            f"状态={'正常' if self.health_filter.state else '暂停'}, "
            f"累计异常={self._health_episodes}次, "
            f"本周期硬异常帧={self._raw_unhealthy_samples}, "
            f"参考漂移告警帧={self._reference_warning_samples}, "
            f"最大误差=位置{maxima[0]:.3f}m/角度{maxima[1]:.2f}度/"
            f"速度{maxima[2]:.3f}m/s, "
            f"最大瞬时跳变=位置{jump_maxima[0]:.3f}m/"
            f"角度{jump_maxima[1]:.2f}度, "
            f"{heading_text}, "
            f"回调耗时=平均{timing.average_ms:.1f}/95%上限{timing.p95_ms:.1f}/"
            f"最大{timing.maximum_ms:.1f}ms"
        )
        self._raw_unhealthy_samples = 0
        self._reference_warning_samples = 0
        self._max_health_metrics = [0.0, 0.0, 0.0]
        self._max_jump_metrics = [0.0, 0.0]

    def _nearest_reference(self, msg: Odometry) -> Odometry | None:
        if not self.reference_messages:
            return None
        local_stamp = _stamp_nanoseconds(msg)
        reference = min(
            self.reference_messages,
            key=lambda candidate: abs(
                _stamp_nanoseconds(candidate) - local_stamp
            ),
        )
        time_delta_ns = abs(_stamp_nanoseconds(reference) - local_stamp)
        if time_delta_ns > self.reference_max_time_delta_ns:
            return None
        return reference

    def _on_raw_tf(self, msg: TFMessage) -> None:
        extrinsics = relay_extrinsic_transforms(
            msg,
            self.local_frame,
            self.base_frame,
        )
        if not extrinsics.transforms:
            return
        self.extrinsics_ready = True
        self.tf_publisher.publish(extrinsics)
        if not self._logged_extrinsics:
            frames = [
                f"{transform.header.frame_id}->{transform.child_frame_id}"
                for transform in extrinsics.transforms
            ]
            self.get_logger().info(f"已转发第一帧 DLIO 传感器外参, frames={frames}")
            self._logged_extrinsics = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DlioTfAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
