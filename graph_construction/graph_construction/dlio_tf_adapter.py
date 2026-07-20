from __future__ import annotations

import copy
import math
from collections import deque
from dataclasses import dataclass

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


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


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
    """Keep DLIO sensor extrinsics while dropping its odom to base transform"""
    output = TFMessage()
    output.transforms = [
        copy.deepcopy(transform)
        for transform in msg.transforms
        if not _is_pose_transform(transform, odom_frame, base_frame)
    ]
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


def is_odometry_healthy(
    aligned: Odometry,
    reference: Odometry | None,
    max_position_error: float,
    max_orientation_error_deg: float,
    max_linear_speed: float,
) -> tuple[bool, tuple[float, float, float]]:
    metrics = odometry_health_metrics(aligned, reference)
    values = [
        aligned.pose.pose.position.x,
        aligned.pose.pose.position.y,
        aligned.pose.pose.position.z,
        aligned.pose.pose.orientation.x,
        aligned.pose.pose.orientation.y,
        aligned.pose.pose.orientation.z,
        aligned.pose.pose.orientation.w,
        *metrics,
    ]
    healthy = (
        all(math.isfinite(value) for value in values)
        and metrics[0] <= max_position_error
        and metrics[1] <= max_orientation_error_deg
        and metrics[2] <= max_linear_speed
    )
    return healthy, metrics


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
        self.reference_messages: deque[Odometry] = deque(maxlen=500)
        self.alignment: RigidTransform | None = None
        self.first_local_stamp_ns: int | None = None
        self._logged_pose = False
        self._logged_extrinsics = False
        self._logged_waiting = False
        self._health_status: bool | None = None
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
        if self.alignment is None:
            if not self.extrinsics_ready:
                if not self._logged_waiting:
                    self.get_logger().info(
                        "等待 DLIO IMU 标定完成并发布传感器外参"
                    )
                    self._logged_waiting = True
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
                return
            reference = self._nearest_reference(msg)
            if reference is None:
                if not self._logged_waiting:
                    self.get_logger().warn(
                        "等待与 DLIO 同时刻的参考 odom 以初始化全局坐标"
                    )
                    self._logged_waiting = True
                return
            self.alignment = alignment_from_odometry(reference, msg)
            stamp_delta_ns = abs(
                _stamp_nanoseconds(reference) - _stamp_nanoseconds(msg)
            )
            delta_ms = stamp_delta_ns / 1.0e6
            translation = self.alignment.translation
            self.get_logger().info(
                "已锁定 DLIO 到全局坐标的启动对齐, "
                f"delta={delta_ms:.1f}ms, translation=({translation[0]:.3f}, "
                f"{translation[1]:.3f}, {translation[2]:.3f})"
            )

        aligned_odom = align_odometry(
            msg,
            self.alignment,
            self.global_frame,
            self.base_frame,
        )
        reference = self._nearest_reference(msg)
        healthy, metrics = is_odometry_healthy(
            aligned_odom,
            reference,
            self.max_position_error,
            self.max_orientation_error_deg,
            self.max_linear_speed,
        )
        if not healthy:
            if self._publish_health(False):
                self.get_logger().error(
                    "DLIO 位姿发散, 已暂停 odom、TF 和点云输出, "
                    f"position_error={metrics[0]:.3f}m, "
                    f"orientation_error={metrics[1]:.2f}deg, "
                    f"speed={metrics[2]:.3f}m/s"
                )
            return
        recovered = self._health_status is False
        self._publish_health(True)
        if recovered:
            self.get_logger().info("DLIO 位姿已恢复, 继续发布 odom、TF 和点云")
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

    def _publish_health(self, healthy: bool) -> bool:
        if self._health_status == healthy:
            return False
        self.health_publisher.publish(Bool(data=healthy))
        self._health_status = healthy
        return True

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
