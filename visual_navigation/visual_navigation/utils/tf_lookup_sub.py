from dataclasses import dataclass
from typing import List, Dict
from abc import ABC, abstractmethod

from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.duration import Duration

from builtin_interfaces.msg import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from omegaconf import OmegaConf
import numpy as np
from scipy.spatial.transform import Rotation as R

from visual_navigation.utils.buffer import MessageBuffer

@dataclass
class TFEdge:
    source_frame: str
    target_frame: str

class TFLookupSubscriber(Node, ABC):

    default_tflookup_config = {
        "buffer_size": 1,       # number of messages
        "cache_time": 10,       # seconds
        "timer_duration": 0.5,  # seconds
        "lookup_timeout": 0,     # seconds
        "qos_history_depth": 100,  # depth for QoS profile
        "wait_for_oldest": False,  # whether to wait when buffer is full
        "clear_buffer_on_process": False,  # whether to clear buffer after processing
        "spin_thread": False,     # whether to spin tf listener in a separate thread
        "allow_latest_tf_on_past_extrapolation": True,  # fallback for sim time startup jitter
    }

    def __init__(
        self, node_name: str, config: OmegaConf=OmegaConf.create()
    ):
        super().__init__(node_name)
        config = OmegaConf.merge(OmegaConf.create(self.default_tflookup_config), config)

        self.msg_buffer = MessageBuffer(max_size=config.buffer_size, wait_for_oldest=config.wait_for_oldest)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=config.cache_time))
        self.tf_listener = TransformListener(
            self.tf_buffer, self,
            qos = QoSProfile(
                depth=config.qos_history_depth,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
            spin_thread=config.spin_thread
        )

        self.oldest_time_processed = None
        self.timer = None
        self.timer_duration = config.timer_duration
        self.lookup_timeout = config.lookup_timeout
        self.clear_buffer_on_process = config.clear_buffer_on_process
        self.allow_latest_tf_on_past_extrapolation = self._config_bool(
            config.allow_latest_tf_on_past_extrapolation
        )

        self._required_transforms: Dict[str, TFEdge] = {}
        self._tf_found_count = 0
        self._tf_missing_count = 0
        self._tf_latest_fallback_count = 0

    @property
    def required_transforms(self):
        return self._required_transforms

    @required_transforms.setter
    def required_transforms(self, transforms: Dict[str, TFEdge]):
        if not self._required_transforms:
            self._required_transforms = transforms
        else:
            self.get_logger().warn(f"Required transforms already set. {self.__class__.__name__} tried to set it again.")

    def start_timer(self):
        if self.timer is None:
            self.timer = self.create_timer(self.timer_duration, self.check_tf_exists)
        else:
            self.get_logger().warn(f"Timer already initialized. {self.__class__.__name__} tried to initialize it again.")

    def check_tf_exists(self):
        if self.msg_buffer.buffer:
            found_one_valid_ts = False
            found_invalid_after_valid = False
            valid_tfs = None
            valid_msg = None
            valid_ts = None

            for old_msg, old_msg_stamp, old_msg_tm in self.msg_buffer.buffer:
                tfs = {}
                for edge_name, edge in self._required_transforms.items():
                    try:
                        tf_oldest_msg = self._lookup_transform_with_fallback(edge, old_msg_stamp)
                        tfs[edge_name] = tf_oldest_msg

                    except Exception as e:
                        self._log_tf_missing(edge, old_msg_stamp, e)
                        if not found_one_valid_ts:
                            if self._is_past_extrapolation(e):
                                self.get_logger().warn(
                                    f"Dropping stale message at time {old_msg_tm}, TF buffer cannot serve older data"
                                )
                                self.msg_buffer.pop_oldest_msg()
                            return
                        else:
                            found_invalid_after_valid = True
                            break
                if found_invalid_after_valid:
                    break
                found_one_valid_ts = True
                valid_tfs = tfs.copy()
                valid_msg = old_msg
                valid_ts = old_msg_tm
                break
            
            if self.oldest_time_processed is None or self.oldest_time_processed < valid_ts:
                self.oldest_time_processed = valid_ts
                self._log_tf_found(valid_ts)
                self.do_processing(valid_msg, valid_tfs)
                if self.clear_buffer_on_process:
                    self.msg_buffer.clear()
            else:
                self.get_logger().debug(f"Already processed TF for time {valid_ts}, skipping processing.")
                self.msg_buffer.pop_oldest_msg()
        else:
            self.get_logger().debug("Message buffer is empty, waiting for messages...")

    def fetch_cam_intrinsics_extrinsics(self, cam_info, tf_world_from_cam):
        """
        Fetch camera intrinsics and extrinsics using the CameraInfo message.
        """
        K = np.array(cam_info.k).reshape(3, 3)

        R_wc = R.from_quat([
            tf_world_from_cam.transform.rotation.x,
            tf_world_from_cam.transform.rotation.y,
            tf_world_from_cam.transform.rotation.z,
            tf_world_from_cam.transform.rotation.w
        ]).as_matrix()
        t_wc = np.array([
            tf_world_from_cam.transform.translation.x,
            tf_world_from_cam.transform.translation.y,
            tf_world_from_cam.transform.translation.z
        ]).reshape(3, 1)

        frame_id = cam_info.header.frame_id
        if frame_id.startswith("/"):
            frame_id = frame_id[1:]
        return {
            "K": K,
            "height": cam_info.height,
            "width": cam_info.width,
            "R_wc": R_wc,
            "t_wc": t_wc,
            "frame_id": frame_id,
        }

    @staticmethod
    def _is_past_extrapolation(error: Exception) -> bool:
        msg = str(error)
        return (
            "extrapolation into the past" in msg
            or "only time" in msg and "is in the buffer" in msg
        )

    @staticmethod
    def _config_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        normalized = str(value).strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
        raise ValueError(f"Invalid boolean config value: {value}")

    def _lookup_transform_with_fallback(self, edge: TFEdge, stamp: Time):
        """仿真启动期图像时间略早于 TF buffer 时, 回退使用 latest TF"""
        try:
            return self.tf_buffer.lookup_transform(
                edge.target_frame,
                edge.source_frame,
                stamp,
                timeout=Duration(seconds=self.lookup_timeout)
            )
        except Exception as exc:
            if not self.allow_latest_tf_on_past_extrapolation or not self._is_past_extrapolation(exc):
                raise
            try:
                latest_tf = self.tf_buffer.lookup_transform(
                    edge.target_frame,
                    edge.source_frame,
                    Time(),
                    timeout=Duration(seconds=self.lookup_timeout)
                )
            except Exception:
                raise exc
            self._log_latest_tf_fallback(edge, stamp, exc)
            return latest_tf

    def _log_latest_tf_fallback(self, edge: TFEdge, stamp: Time, error: Exception) -> None:
        self._tf_latest_fallback_count += 1
        if self._tf_latest_fallback_count == 1:
            self.get_logger().warn(
                f"TF 时间略早于缓存, 已回退 latest TF, source={edge.source_frame}, time={stamp}, error={error}"
            )
        elif self._tf_latest_fallback_count % 100 == 0:
            self.get_logger().debug(
                f"TF latest 回退次数={self._tf_latest_fallback_count}, source={edge.source_frame}, time={stamp}"
            )

    def _log_tf_found(self, valid_ts) -> None:
        self._tf_found_count += 1
        if self._tf_found_count == 1:
            self.get_logger().info(f"首次找到相机 TF, time={valid_ts}")
        elif self._tf_found_count % 100 == 0:
            self.get_logger().debug(f"相机 TF 已匹配次数={self._tf_found_count}, latest_time={valid_ts}")

    def _log_tf_missing(self, edge: TFEdge, stamp: Time, error: Exception) -> None:
        self._tf_missing_count += 1
        if self._tf_missing_count == 1:
            self.get_logger().warn(f"暂时未找到 TF, source={edge.source_frame}, time={stamp}, error={error}")
        elif self._tf_missing_count % 100 == 0:
            self.get_logger().debug(
                f"TF 未匹配次数={self._tf_missing_count}, source={edge.source_frame}, time={stamp}, error={error}"
            )

    @abstractmethod
    def do_processing(self, msg: Dict, tfs: List):
        raise NotImplementedError("This method should be overridden by subclasses.")
