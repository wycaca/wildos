import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from copy import deepcopy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image as ImageMsg, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from graphnav_msgs.msg import NavigationGraph, KeyValue
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from object_search_msgs.msg import ObjectMaskWithTf

from pathlib import Path
from threading import Lock
from omegaconf import OmegaConf
import numpy as np
import torch
import time
from torchvision import transforms

from visual_navigation.utils.tf_lookup_sub import TFEdge, TFLookupSubscriber
from visual_navigation.wildos.goalagnostic_scoring import GoalAgnosticScoring
from visual_navigation.wildos.current_frontier_scores import CurrentFrontierScores
from visual_navigation.geofrontier_nav.geofrontier_to_image import GeoFrontierToImage
from visual_navigation.wildos.viz import VisualizeGoalAgnosticGeoFrontierScoring
from explorfm import ExploRFMInference
from visual_navigation.utils.object_search_utils import (
    get_objectmask_msg,
    localize_query,
    reference_image_stamp,
)
from visual_navigation.object_detection_filter import analyze_object_detection_mask
from visual_navigation.object_detection_confirmation import DetectionConfirmationWindow
from visual_navigation.object_reached_evidence import VisualReachedEvidence
from visual_navigation.object_search_types import normalize_object_search_target
from visual_navigation.utils.paths import repository_root
from visual_navigation.utils.performance_stats import EventRate, TimingWindow
from visual_navigation.utils.publish_gate import PeriodicPublishGate
from visual_navigation.utils.wildos_input_cache import WildOSInputCache

HOME_DIR = repository_root()
CAMERA_MAPPING = {
    0: "front",
    1: "left",
    2: "right"
}
CAMERA_LOG_NAMES = {
    0: "前相机",
    1: "左相机",
    2: "右相机",
}
_CALLBACK_LOG_INTERVAL = 100
_DIAGNOSTICS_LOG_PERIOD_SEC = 30.0
_SLOW_PROCESSING_WARNING_MS = 1500.0


class WildOS_Nav(TFLookupSubscriber):
    default_config = {
        # 模型参数
        "frontier_ckpt": "frontier_head.ckpt",
        "traversability_ckpt": "trav_head.ckpt",
        "model_version": "c-radio_v3-b",
        "adaptor_version": None,
        "use_naclip": True,
        "use_summary_for_spatial": True,
        "radio_dim": 768,
        "static_scale_factor": 0.75,
        "model_precision": "FP16",

        # 导航参数
        "num_cameras": 3,
        "cams_inverted": True,
        "camera_image_flip_x": True,
        "num_angular_bins": 16,
        "reach_in_2D": True,

        # 导航图参数
        "frontiers_range": 12.0,
        "traversability_class": "default",
        "heading_sim_thresh": 0.866,  # 相机朝向和 frontier 朝向的余弦相似度阈值
        "default_max_score": 0.5,
        "std_for_default_scores": 30.0,  # 角度
        "std_for_frontier_heading": 30.0,  # 角度

        # 像素评分参数
        "frontier_threshold": 0.6,
        "frontier_opening_kernel_size": 0,
        "traversability_threshold": 0.8,
        "frontier_w": 2.0,
        "goal_w": 3.0,
        "reachability_w": 2.0,
        "scoring_method": "ADDITIVE",
        "reach_scale": 0.25,
        "compute_paths": False,

        # ROS2 frame 和 topic
        "parent_frame": "spot1/odom",
        "cam_frame": "{}_color_optical_frame",
        "camera_img_topic": "/spot1/realsense/{}/color/image_raw/compressed",
        "camera_info_topic": "/spot1/realsense/{}/color/camera_info",
        "odometry_topic": "/spot1/odom",
        "navigation_graph_topic": "/spot1/nav_graph",

        # ROS2 发布 topic
        "scored_navgraph_topic": "/spot1/scored_nav_graph",
        "model_viz_topic": "model_visualization",
        "valid_geofrontiers_topic": "within_range_geofrontiers",
        "score_ring_topic": "/spot1/score_rings",
        "object_mask_topic": "/spot1/object_mask",
        "object_reached_topic": "/spot1/object_search_reached",
        "object_completed_topic": "/spot1/object_search_completed",
        "object_search_target_topic": "/spot1/object_search_target",
        "visualization_publish_period_sec": 2.0,
        "visualization_require_subscribers": True,
        "processing_rate_hz": None,

        # ROS2 订阅参数
        "qos_history_depth": 1,
        "syncsub_queue_size": 1,
        "syncsub_slop": 0.2,
        "odometry_cache_size": 100,
        "odometry_match_max_delta_sec": 0.2,
        "navigation_graph_max_age_sec": 1.0,
        "navigation_graph_future_tolerance_sec": 0.1,

        # 目标搜索参数
        "object_search_config": {
            # "text_queries": ["NASA logo"],
            # "text_queries": ["orange flag"],
            # "text_queries": ["golf cart"],
            # "text_queries": ["garbage container"],
            # "text_queries": ["blue bucket"],
            "text_queries": ["chair"],
            "pixel_level_seg": False,
            "mask_threshold": 0.09,
            "detection_min_peak_score": 0.12,
            "detection_confirm_min_peak_score": 0.110,
            "detection_min_component_pixels": 300,
            "detection_min_component_fraction": 0.0005,
            "detection_confirm_frames": 2,
            "detection_confirm_window_frames": 3,
            "detection_debug_interval": 20,
            "obj_frontier_score": 0.9,
            "obj_trav_score": 0.9,
            "reached_mask_fraction": 0.01,
            "reached_min_pixel_count": 1200,
            "reached_confirm_frames": 2
        },

        # TF 查询配置
        "tf_lookup_config": {
            "buffer_size": 1,       # 消息数量
            "cache_time": 10,       # 秒
            "timer_duration": 0.05,  # 秒
            "lookup_timeout": 0,     # 秒
            "qos_history_depth": 1,  # QoS 队列深度
            "wait_for_oldest": False,  # buffer 满时是否等待
            "clear_buffer_on_process": True,  # 处理后是否清空 buffer
            "spin_thread": False,     # 是否单独线程 spin TF listener
        },

    }

    def __init__(self, config: OmegaConf=OmegaConf.create(), do_object_search=False):
        config = OmegaConf.merge(OmegaConf.create(self.default_config), config)
        processing_rate_hz = config.get("processing_rate_hz")
        if processing_rate_hz is not None:
            processing_rate_hz = float(processing_rate_hz)
            if processing_rate_hz <= 0.0:
                raise ValueError("processing_rate_hz must be greater than 0")
            config.tf_lookup_config.timer_duration = 1.0 / processing_rate_hz

        # 先创建 ROS 和 TF endpoints, 让中间件在模型加载期间完成发现
        super().__init__(
            node_name='wildos',
            config=config.tf_lookup_config
        )
        self.get_logger().info(
            f"WildOS 模型初始化开始, object_search={do_object_search}"
        )
        if processing_rate_hz is not None:
            self.get_logger().info(
                f"WildOS 视觉处理限频, rate={processing_rate_hz:.1f}Hz"
            )
        np.random.seed(42)
        self.init_model(config, do_object_search)
        self.get_logger().info("WildOS 模型初始化完成")

        # 用于转换 ROS 和 OpenCV 图像
        self.br = CvBridge()

        # 导航参数和初始化
        self.num_cameras = config.num_cameras
        self.cam_inverted = config.cams_inverted
        self.camera_image_flip_x = self._config_bool(config.get("camera_image_flip_x", False))
        assert self.num_cameras in [1, 3], "Only 1 or 3 cameras are supported."
        self.num_angular_bins = config.num_angular_bins
        self.reach_in_2D = config.reach_in_2D
        
        # 保存当前 frontier node 及其评分
        self.frontier_uuid_to_scores = {}
        self.object_detection_debug_interval = 20
        self._object_missing_log_count = 0
        self.object_detection_confirmation = None
        self._object_detection_ready = False
        self.object_reached_evidence = None
        self._visual_reached_active = False
        self.object_search_completed = False
        if self.object_search_mode:
            self.object_detection_debug_interval = max(
                int(config.object_search_config.get("detection_debug_interval", 20)),
                1,
            )
            self.object_detection_min_peak_score = float(
                config.object_search_config.get(
                    "detection_min_peak_score",
                    self.mask_threshold + 0.03,
                )
            )
            self.object_detection_confirm_min_peak_score = float(
                config.object_search_config.get(
                    "detection_confirm_min_peak_score",
                    self.object_detection_min_peak_score,
                )
            )
            self.object_detection_min_component_pixels = max(
                int(config.object_search_config.get("detection_min_component_pixels", 300)),
                1,
            )
            self.object_detection_min_component_fraction = max(
                float(
                    config.object_search_config.get(
                        "detection_min_component_fraction",
                        0.0005,
                    )
                ),
                0.0,
            )
            self.object_detection_confirm_frames = max(
                int(config.object_search_config.get("detection_confirm_frames", 2)),
                1,
            )
            self.object_detection_confirm_window_frames = max(
                int(
                    config.object_search_config.get(
                        "detection_confirm_window_frames",
                        self.object_detection_confirm_frames + 1,
                    )
                ),
                self.object_detection_confirm_frames,
            )
            self.object_detection_confirmation = DetectionConfirmationWindow(
                self.object_detection_confirm_frames,
                self.object_detection_confirm_window_frames,
            )
            self.object_reached_mask_fraction = float(
                config.object_search_config.get("reached_mask_fraction", 0.01)
            )
            self.object_reached_min_pixel_count = int(
                config.object_search_config.get("reached_min_pixel_count", 1200)
            )
            self.object_reached_confirm_frames = max(
                int(config.object_search_config.get("reached_confirm_frames", 2)),
                1,
            )
            self.object_reached_evidence = VisualReachedEvidence(
                self.object_reached_min_pixel_count,
                self.object_reached_mask_fraction,
                self.object_reached_confirm_frames,
            )
            self.get_logger().info(
                f"目标搜索已启用, 查询目标={list(self.text_queries)}, "
                f"Mask阈值={self.mask_threshold:.3f}, "
                f"进入/确认峰值门槛={self.object_detection_min_peak_score:.3f}/"
                f"{self.object_detection_confirm_min_peak_score:.3f}, "
                f"最小区域像素={self.object_detection_min_component_pixels}"
            )

        # 将导航图 frontier 投影到图像
        self.geofrontier_to_image = GeoFrontierToImage(
            camera_mapping=CAMERA_MAPPING,
            frontiers_range=config.frontiers_range,
            traversability_class=config.traversability_class,
            cams_inverted=self.cam_inverted,
            heading_sim_thresh=config.heading_sim_thresh,
            reach_in_2D=False,
            image_flip_x=self.camera_image_flip_x
        )
        self.traversability_class = config.traversability_class
        self.default_max_score = config.default_max_score
        self.std_for_default_scores = config.std_for_default_scores
        self.std_for_frontier_heading = config.std_for_frontier_heading

        # 初始化像素评分参数
        frontier_threshold = config.frontier_threshold
        traversability_threshold = config.traversability_threshold
        pixel_scoring_params = {
            "frontier": config.frontier_w,
            "goal": config.goal_w,
            "reachability": config.reachability_w,
            "method": config.scoring_method,
        }
        reach_scale = config.reach_scale
        self.scorer = GoalAgnosticScoring(
            num_angular_bins=config.num_angular_bins,
            pixel_scoring_params=pixel_scoring_params,
            frontier_threshold=frontier_threshold,
            frontier_opening_kernel_size=config.frontier_opening_kernel_size,
            traversability_threshold=traversability_threshold,
            reach_in_2D=self.reach_in_2D,
            cam_inverted=self.cam_inverted,
            reach_scale=reach_scale
        )
        self.compute_paths = config.compute_paths
        self.visualization_require_subscribers = self._config_bool(
            config.get("visualization_require_subscribers", True)
        )
        self._visualization_gate = PeriodicPublishGate(
            config.get("visualization_publish_period_sec", 2.0)
        )
        self._last_slow_warning = 0.0
        self._processing_rate = EventRate()
        self._processing_timings = {
            name: TimingWindow()
            for name in ("decode", "project", "inference", "object", "score", "publish", "total")
        }
        self._latency_timings = {
            name: TimingWindow()
            for name in ("sync_wait", "camera_spread", "source_age", "tf_wait", "mask_age")
        }
        self._input_delta_timings = {
            name: TimingWindow()
            for name in ("camera_spread", "odom_delta", "nav_graph_age")
        }
        self._input_rates = {
            name: EventRate()
            for name in (
                "image_front",
                "image_left",
                "image_right",
                "info_front",
                "info_left",
                "info_right",
                "odom",
                "nav_graph",
                "camera_sync",
                "matched",
            )
        }
        self._input_cache = WildOSInputCache(
            num_cameras=self.num_cameras,
            odom_cache_size=int(config.get("odometry_cache_size", 100)),
            odom_max_delta_seconds=float(
                config.get("odometry_match_max_delta_sec", 0.2)
            ),
            nav_graph_max_age_seconds=float(
                config.get("navigation_graph_max_age_sec", 1.0)
            ),
            nav_graph_future_tolerance_seconds=float(
                config.get("navigation_graph_future_tolerance_sec", 0.1)
            ),
        )
        self._input_rejections: dict[str, int] = {}
        self._input_diagnostics_lock = Lock()
        self._input_started_at = time.monotonic()
        self._last_camera_sync_time = None
        self._last_input_match_time = None
        self._image_arrival_times = {
            camera_idx: {}
            for camera_idx in range(self.num_cameras)
        }

        # 可视化
        self.geofrontier_viz_colors = np.array([
            [0.528, 0.471, 0.701],
            [0.772, 0.432, 0.102],
            [0.572, 0.586, 0.0],
        ])
        self.viz = VisualizeGoalAgnosticGeoFrontierScoring(
            angular_bins=self.scorer.angles,
            camera_mapping=CAMERA_MAPPING,
            num_cameras=self.num_cameras
        )

        self.clbk_cntr = 0

        # frame 和 topic 名称
        self.global_frame = config.parent_frame
        self.cam_tf_frame = config.cam_frame
        self.using_compressed_imgs = "compressed" in config.camera_img_topic

        # TF 查询
        self.required_transforms = {
            f"world_from_cam{idx}": TFEdge(
                source_frame=self.cam_tf_frame.format(CAMERA_MAPPING[idx]),
                target_frame=self.global_frame
            )
            for idx in range(self.num_cameras)
        }

        # 订阅和发布
        self.init_publishers(config)
        self.init_subscribers(config)
        self.start_timer()
        self.create_timer(_DIAGNOSTICS_LOG_PERIOD_SEC, self._log_performance)
        self.create_timer(
            _DIAGNOSTICS_LOG_PERIOD_SEC,
            self._log_input_diagnostics,
        )

    def init_model(self, config, do_object_search):
        # VLM 初始化
        self.device = "cuda"

        if do_object_search and config.adaptor_version is None:
            config.adaptor_version = "siglip2"
        model_path = self._resolve_wildos_model_path(config.model_version)
        adaptor_ckpt_path = self._resolve_wildos_adaptor_path(config.adaptor_version)
        print(
            f"WildOS 加载视觉模型, model={model_path}, adaptor={config.adaptor_version}, adaptor_path={adaptor_ckpt_path}",
            flush=True,
        )

        # RADIO 模型
        self.model = ExploRFMInference(
            frontier_ckpt=HOME_DIR / "ckpts" / config.frontier_ckpt,
            traversability_ckpt=HOME_DIR / "ckpts" / config.traversability_ckpt,
            model_version=model_path,
            adaptor_version=config.adaptor_version,
            adaptor_ckpt_path=adaptor_ckpt_path,
            use_naclip=config.use_naclip,
            use_summary_for_spatial=config.use_summary_for_spatial,
            radio_dim=config.radio_dim,
            static_scale_factor=config.static_scale_factor,
            model_precision=config.model_precision,
        )
        self.transforms = transforms.Compose([
            transforms.ToTensor(),
        ])
        self.object_search_mode = False
        print("WildOS 模型加载完成", flush=True)
        if do_object_search:
            self.object_search_mode = True
            self.text_queries = config.object_search_config.text_queries
            self.pixel_level_seg = config.object_search_config.pixel_level_seg
            self.mask_threshold = float(config.object_search_config.mask_threshold)
            self.obj_frontier_score = float(config.object_search_config.obj_frontier_score)
            self.obj_trav_score = float(config.object_search_config.obj_trav_score)

            assert len(self.text_queries) == 1, "Only single object search is supported in this version."

            print(f"WildOS 计算目标搜索文本特征, text_queries={list(self.text_queries)}", flush=True)
            self.text_feats = self.model.forward_on_text(self.text_queries)
            print("WildOS 目标搜索文本特征计算完成", flush=True)

    def _resolve_wildos_model_path(self, model_version):
        """优先使用本地 RADIO 权重, 避免启动时回退到网络下载"""
        model_version = str(model_version)
        model_path = Path(model_version).expanduser()
        if model_path.is_absolute() or model_path.exists():
            return model_path

        local_model_path = HOME_DIR / "ckpts" / f"{model_version}_half.pth.tar"
        if local_model_path.exists():
            return local_model_path

        return model_version

    def _resolve_wildos_adaptor_path(self, adaptor_version):
        """SigLIP2 需要指向完整 HuggingFace cache 目录"""
        if adaptor_version is None:
            return None

        adaptor_path = HOME_DIR / "ckpts" / str(adaptor_version)
        if adaptor_path.exists():
            return adaptor_path

        return HOME_DIR / "ckpts"

    @staticmethod
    def _config_bool(value):
        """解析 launch dotlist 传入的布尔字符串"""
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"true", "1", "yes", "on"}

    def init_publishers(self, config: OmegaConf):
        self.scored_navgraph_pub = self.create_publisher(
            NavigationGraph,
            config.scored_navgraph_topic,
            10
        )
        self.model_viz_pub = self.create_publisher(
            ImageMsg,
            config.model_viz_topic,
            10
        )
        self.withinrange_geofront_pub = self.create_publisher(
            MarkerArray,
            config.valid_geofrontiers_topic,
            10
        )
        self.score_rings_pub = self.create_publisher(
            MarkerArray,
            config.score_ring_topic,
            10
        )
        if self.object_search_mode:
            self.object_mask_publisher = self.create_publisher(
                ObjectMaskWithTf,
                config.object_mask_topic,
                1
            )
            self.object_reached_publisher = self.create_publisher(
                Bool,
                config.object_reached_topic,
                10
            )

    def init_subscribers(self, config):
        """只同步相机图像, 其他输入使用独立缓存匹配"""
        cameraimg_topic_str = config.camera_img_topic
        camerainfo_topic_str = config.camera_info_topic
        img_msg_type = CompressedImage if self.using_compressed_imgs else ImageMsg

        self.camera_subs = {}
        for i in range(self.num_cameras):
            self.camera_subs[i] = Subscriber(
                self,
                img_msg_type,
                cameraimg_topic_str.format(CAMERA_MAPPING[i]),
                qos_profile=config.qos_history_depth,
            )
            self.camera_subs[i].registerCallback(
                self._record_image_arrival,
                i,
            )
        self.camera_info_subs = [
            self.create_subscription(
                CameraInfo,
                camerainfo_topic_str.format(CAMERA_MAPPING[i]),
                lambda msg, camera_idx=i: self._cache_camera_info(
                    msg,
                    camera_idx,
                ),
                config.qos_history_depth,
            )
            for i in range(self.num_cameras)
        ]
        self.odom_sub = self.create_subscription(
            Odometry,
            config.odometry_topic,
            self._cache_odom,
            config.qos_history_depth,
        )
        self.navgraph_sub = self.create_subscription(
            NavigationGraph,
            config.navigation_graph_topic,
            self._cache_nav_graph,
            config.qos_history_depth,
        )
        if self.object_search_mode:
            self.object_target_sub = self.create_subscription(
                String,
                config.object_search_target_topic,
                self._on_object_search_target,
                10,
            )
            self.object_completed_sub = self.create_subscription(
                Bool,
                config.object_completed_topic,
                self._on_object_search_completed,
                10,
            )

        self.ts = ApproximateTimeSynchronizer(
            list(self.camera_subs.values()),
            queue_size=config.syncsub_queue_size,
            slop=config.syncsub_slop,
        )
        self.ts.registerCallback(self.listener_callback)

    def listener_callback(self, *image_msgs):
        """为三相机同步组匹配内参、最近 odom 和最新导航图"""
        synchronized_at = time.perf_counter()
        self._input_rates["camera_sync"].tick()
        self._last_camera_sync_time = time.monotonic()
        measurement_stamp = reference_image_stamp(image_msgs)
        matched_inputs, rejection_reason = self._input_cache.match(
            measurement_stamp
        )
        if matched_inputs is None:
            self._record_input_rejection(rejection_reason)
            return

        odom_msg = matched_inputs.odom
        navgraph_msg = matched_inputs.nav_graph
        cam_info_msgs = matched_inputs.camera_infos
        if odom_msg.header.frame_id != self.global_frame:
            self._record_input_rejection("odom_frame_mismatch")
            return

        self._input_rates["matched"].tick()
        self._last_input_match_time = time.monotonic()
        self.clbk_cntr += 1
        if self.clbk_cntr == 1:
            self.get_logger().info(
                "WildOS 已匹配第一组相机、里程计和导航图输入"
            )
        elif self.clbk_cntr % _CALLBACK_LOG_INTERVAL == 0:
            self.get_logger().debug(
                f"WildOS 已处理完整输入次数={self.clbk_cntr}"
            )

        image_stamp_seconds = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for msg in image_msgs
        ]
        camera_spread = max(image_stamp_seconds) - min(image_stamp_seconds)
        self._latency_timings["camera_spread"].add_seconds(camera_spread)
        self._input_delta_timings["camera_spread"].add_seconds(camera_spread)
        self._input_delta_timings["odom_delta"].add_seconds(
            matched_inputs.odom_delta_seconds
        )
        self._input_delta_timings["nav_graph_age"].add_seconds(
            matched_inputs.nav_graph_age_seconds
        )
        arrival_times = [
            self._image_arrival_times[camera_idx].pop(
                self._stamp_nanoseconds(image_msg.header.stamp),
                None,
            )
            for camera_idx, image_msg in enumerate(image_msgs)
        ]
        available_arrivals = [arrival for arrival in arrival_times if arrival is not None]
        if available_arrivals:
            self._latency_timings["sync_wait"].add_seconds(
                synchronized_at - min(available_arrivals)
            )
        source_age = self._stamp_age_seconds(measurement_stamp)
        if source_age is not None:
            self._latency_timings["source_age"].add_seconds(source_age)
        measurement_header = Header(
            stamp=measurement_stamp,
            frame_id=self.global_frame,
        )
        self.msg_buffer.add_msg(
            msg={
                "odom": odom_msg,
                "navgraph": navgraph_msg,
                "image_msgs": tuple(image_msgs),
                "camera_info_msgs": cam_info_msgs,
                "measurement_header": measurement_header,
                "queued_at": synchronized_at,
            },
            stamp=measurement_stamp,
        )

    def do_processing(self, msg, tf_data):
        self.get_logger().debug("WildOS 开始执行视觉评分")
        processing_started = time.perf_counter()
        queued_at = msg.get("queued_at")
        if queued_at is not None:
            self._latency_timings["tf_wait"].add_seconds(
                processing_started - queued_at
            )

        # 提取消息
        odom_msg = msg["odom"]
        navgraph_msg = msg["navgraph"]
        image_msgs = msg["image_msgs"]
        cam_info_msgs = msg["camera_info_msgs"]
        measurement_header = msg["measurement_header"]

        # 提取相机图像和内参
        stage_started = time.perf_counter()
        rgb_imgs = []
        for i in range(self.num_cameras):
            if self.using_compressed_imgs:
                convert_func = self.br.compressed_imgmsg_to_cv2
            else:
                convert_func = self.br.imgmsg_to_cv2

            if self.cam_inverted:
                rgb_imgs.append(
                    np.rot90(
                        convert_func(
                            image_msgs[i],
                            desired_encoding='rgb8',
                        ),
                        k=2,
                    )
                )
            else:
                rgb_imgs.append(
                    convert_func(image_msgs[i], desired_encoding='rgb8')
                )
        self._processing_timings["decode"].add_seconds(time.perf_counter() - stage_started)

        # 从 navgraph_msg 提取 geofrontier
        stage_started = time.perf_counter()
        all_cam_data = []
        for i, cam_info_msg in enumerate(cam_info_msgs):
            cam_data = self.fetch_cam_intrinsics_extrinsics(cam_info_msg, tf_data[f"world_from_cam{i}"])
            all_cam_data.append(cam_data)

        try:
            geofrontiers = self.geofrontier_to_image.extract_geofrontiers(
                current_odom_msg=odom_msg,
                navgraph=navgraph_msg,
                all_cam_data=all_cam_data
            )
        except Exception as e:
            self.get_logger().error(f"提取几何Frontier失败, 原因={e}")
            self._finish_processing(processing_started)
            return
        self._processing_timings["project"].add_seconds(time.perf_counter() - stage_started)

        # 模型前向推理
        stage_started = time.perf_counter()
        rgb_tensors = [self.transforms(img.copy()) for img in rgb_imgs]
        batch_tensor = torch.stack(rgb_tensors)
        batch_img_traversability, batch_img_frontiers, spatial_feats = self.model.forward(batch_tensor)

        batch_img_frontiers = batch_img_frontiers.cpu().numpy().astype(np.float32)
        batch_img_traversability = batch_img_traversability.cpu().numpy().astype(np.float32)
        self._processing_timings["inference"].add_seconds(time.perf_counter() - stage_started)

        stage_started = time.perf_counter()
        object_detected = False
        if self.object_search_mode:
            if self.object_search_completed:
                # Mux 确认任务完成后保留 scored graph 更新, 停止目标证据链
                binary_mask = np.zeros_like(batch_img_frontiers, dtype=np.uint8)
            else:
                if self.model.model_precision.is_fp16():
                    spatial_feats = spatial_feats.half()

                text_sim_spatial, binary_mask = localize_query(
                    text_feats=self.text_feats,
                    spatial_feats=spatial_feats,
                    orig_img_shape=rgb_imgs[0].shape[:2],
                    pixel_level_seg=self.pixel_level_seg,
                    mask_threshold=self.mask_threshold
                )
                active_peak_threshold = (
                    self.object_detection_confirmation.active_threshold(
                        self.object_detection_min_peak_score,
                        self.object_detection_confirm_min_peak_score,
                    )
                )
                binary_mask, detection_components, detection_rejections = (
                    analyze_object_detection_mask(
                        text_sim_spatial,
                        binary_mask,
                        min_peak_score=active_peak_threshold,
                        min_component_pixels=self.object_detection_min_component_pixels,
                        min_component_fraction=self.object_detection_min_component_fraction,
                    )
                )
                has_detection_evidence = bool(detection_components)
                if has_detection_evidence:
                    self._object_missing_log_count = 0
                was_confirmation_ready = self._object_detection_ready
                object_detected = self.object_detection_confirmation.update(
                    has_detection_evidence
                )
                confirmation_ready = self.object_detection_confirmation.ready
                if confirmation_ready and not was_confirmation_ready:
                    self._log_object_detection_confirmed(detection_components)
                elif was_confirmation_ready and not confirmation_ready:
                    self._log_object_detection_lost()
                self._object_detection_ready = confirmation_ready

                if object_detected:
                    self._publish_object_reached(binary_mask)
                    tf_list = [
                        tf_data[f"world_from_cam{i}"]
                        for i in range(self.num_cameras)
                    ]
                    camera_scores = [0.0] * self.num_cameras
                    for component in detection_components:
                        camera_scores[component.camera_idx] = component.peak_score
                    self.object_mask_publisher.publish(
                        get_objectmask_msg(
                            binary_mask,
                            self.cam_inverted,
                            tf_list,
                            cam_info_msgs,
                            camera_scores=camera_scores,
                            measurement_header=measurement_header,
                        )
                    )
                    mask_age = self._stamp_age_seconds(measurement_header.stamp)
                    if mask_age is not None:
                        self._latency_timings["mask_age"].add_seconds(mask_age)
                    batch_img_frontiers = np.maximum(
                        batch_img_frontiers,
                        self.obj_frontier_score * binary_mask,
                    )
                    batch_img_traversability = np.maximum(
                        batch_img_traversability,
                        self.obj_trav_score * binary_mask,
                    )
                else:
                    self._publish_object_reached(None)
                    if has_detection_evidence:
                        self._log_object_detection_pending(detection_components)
                    else:
                        self._log_object_missing(
                            detection_rejections,
                            confirmation_ready,
                        )
                    binary_mask.fill(0)
        self._processing_timings["object"].add_seconds(time.perf_counter() - stage_started)

        # 给几何 frontier 评分
        stage_started = time.perf_counter()
        nav_data = []
        for i in range(self.num_cameras):
            cam_data = all_cam_data[i]
            if not geofrontiers[i]:
                # 当前相机没有几何 frontier
                nav_data.append({
                    "image": rgb_imgs[i],
                    "traversability": batch_img_traversability[i][0],
                    "img_frontiers": batch_img_frontiers[i][0],
                    "object_mask": binary_mask[i][0] if self.object_search_mode else None,
                })
                continue

            scores, paths, score_maps = self.scorer.score_geofrontiers(
                geometric_frontiers=geofrontiers[i]["frontier_pixel_coords"],
                img_frontiers=batch_img_frontiers[i][0],
                traversability=batch_img_traversability[i][0],
                cam_data=cam_data,
                compute_paths=self.compute_paths
            )
            nav_data.append({
                "image": rgb_imgs[i],
                "traversability": batch_img_traversability[i][0],
                "img_frontiers": batch_img_frontiers[i][0],
                "object_mask": binary_mask[i][0] if self.object_search_mode else None,
                "score_map": score_maps,
                "geo_frontiers": geofrontiers[i]["frontier_pixel_coords"],
                "scores": scores,
                "paths": paths,
            })
        self._processing_timings["score"].add_seconds(time.perf_counter() - stage_started)

        # 发布评分后的 navgraph
        stage_started = time.perf_counter()
        updated_navgraph, removed_uuids, updated_uuids = self.update_navgraph_with_scores(
            navgraph_msg, geofrontiers, nav_data
        )
        self.scored_navgraph_pub.publish(updated_navgraph)
        if self._should_publish_visualization():
            if self._visualization_publisher_enabled(self.withinrange_geofront_pub):
                self.viz.delete_markers(self.withinrange_geofront_pub)
                self.withinrange_geofront_pub.publish(
                    self.viz.viz_valid_geofrontiers(
                        geofrontiers,
                        all_cam_data,
                        odom_msg.header,
                        self.geofrontier_viz_colors,
                    )
                )
            if self._visualization_publisher_enabled(self.model_viz_pub):
                # 附加源图像时间, 方便 RViz 跟踪 debug 图像流
                model_viz_msg = self.br.cv2_to_imgmsg(
                    self.viz.visualize_model_det(nav_data, all_cam_data),
                    encoding="rgb8",
                )
                model_viz_msg.header = image_msgs[0].header
                self.model_viz_pub.publish(model_viz_msg)
            if self._visualization_publisher_enabled(self.score_rings_pub):
                self.score_rings_pub.publish(
                    self.viz.visualize_all_heading_scores(
                        self.frontier_uuid_to_scores,
                        removed_uuids,
                        updated_uuids,
                        self.global_frame,
                        self.get_clock().now().to_msg(),
                    )
                )
        self._processing_timings["publish"].add_seconds(time.perf_counter() - stage_started)
        self._finish_processing(processing_started)

    def _should_publish_visualization(self) -> bool:
        """仅在有调试订阅者且达到周期时构建可视化消息"""
        publishers = (
            self.withinrange_geofront_pub,
            self.model_viz_pub,
            self.score_rings_pub,
        )
        if not any(self._visualization_publisher_enabled(pub) for pub in publishers):
            return False
        return self._visualization_gate.ready()

    def _visualization_publisher_enabled(self, publisher) -> bool:
        """按配置跳过没有订阅者的调试 topic"""
        return (
            not self.visualization_require_subscribers
            or publisher.get_subscription_count() > 0
        )

    def _finish_processing(self, processing_started: float) -> None:
        """Record total processing time and throttle slow-frame warnings"""
        elapsed = time.perf_counter() - processing_started
        self._processing_timings["total"].add_seconds(elapsed)
        self._processing_rate.tick()
        now = time.monotonic()
        if (
            elapsed * 1000.0 >= _SLOW_PROCESSING_WARNING_MS
            and now - self._last_slow_warning >= 30.0
        ):
            self._last_slow_warning = now
            self.get_logger().warn(
                f"WildOS 视觉处理耗时偏高, total={elapsed * 1000.0:.1f}ms, "
                f"threshold={_SLOW_PROCESSING_WARNING_MS:.1f}ms"
            )

    def _log_performance(self) -> None:
        """Report visual pipeline timing at low frequency"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._processing_timings.items()
        }
        if not summaries["total"].count:
            return
        total = summaries["total"]
        latency = {
            name: timing.summary(reset=True)
            for name, timing in self._latency_timings.items()
        }
        self.get_logger().info(
            "WildOS 视觉性能, "
            f"频率={self._processing_rate.sample(reset=True):.2f}Hz, "
            f"总耗时=平均{total.average_ms:.0f}/95%上限{total.p95_ms:.0f}/"
            f"最大{total.maximum_ms:.0f}ms, "
            "阶段平均耗时="
            f"图像解码{summaries['decode'].average_ms:.0f}ms/"
            f"几何投影{summaries['project'].average_ms:.0f}ms/"
            f"模型推理{summaries['inference'].average_ms:.0f}ms/"
            f"目标检测{summaries['object'].average_ms:.0f}ms/"
            f"边界评分{summaries['score'].average_ms:.0f}ms/"
            f"结果发布{summaries['publish'].average_ms:.0f}ms, "
            "链路延迟平均值="
            f"图像同步等待{latency['sync_wait'].average_ms:.0f}ms/"
            f"三相机时间差{latency['camera_spread'].average_ms:.0f}ms/"
            f"同步时图像年龄{latency['source_age'].average_ms:.0f}ms/"
            f"TF等待{latency['tf_wait'].average_ms:.0f}ms/"
            f"Mask发布年龄{latency['mask_age'].average_ms:.0f}ms"
        )

    def _record_image_arrival(self, msg, camera_idx: int) -> None:
        """记录图像进入同步器的时间, 仅保留最近少量帧"""
        self._input_rates[f"image_{CAMERA_MAPPING[camera_idx]}"].tick()
        arrivals = self._image_arrival_times[camera_idx]
        arrivals[self._stamp_nanoseconds(msg.header.stamp)] = time.perf_counter()
        while len(arrivals) > 16:
            arrivals.pop(next(iter(arrivals)))

    def _cache_camera_info(self, msg: CameraInfo, camera_idx: int) -> None:
        """独立缓存 CameraInfo, 不参与逐帧同步"""
        self._input_rates[f"info_{CAMERA_MAPPING[camera_idx]}"].tick()
        self._input_cache.add_camera_info(camera_idx, msg)

    def _cache_odom(self, msg: Odometry) -> None:
        """缓存 odom, 供相机测量按时间选择最近帧"""
        self._input_rates["odom"].tick()
        self._input_cache.add_odom(msg)

    def _cache_nav_graph(self, msg: NavigationGraph) -> None:
        """缓存时间最新且 current node 有效的导航图"""
        self._input_rates["nav_graph"].tick()
        if not self._input_cache.update_nav_graph(msg):
            self._record_input_rejection("nav_graph_invalid")

    def _record_input_rejection(self, reason: str) -> None:
        """线程安全累计输入匹配拒绝原因"""
        with self._input_diagnostics_lock:
            self._input_rejections[reason] = (
                self._input_rejections.get(reason, 0) + 1
            )

    def _log_input_diagnostics(self) -> None:
        """低频输出输入频率、时间差和同步停滞原因"""
        now = time.monotonic()
        cache = self._input_cache.snapshot()
        rates = {
            name: rate.sample(reset=True)
            for name, rate in self._input_rates.items()
        }
        deltas = {
            name: timing.summary(reset=True)
            for name, timing in self._input_delta_timings.items()
        }
        camera_sync_idle = now - (
            self._last_camera_sync_time or self._input_started_at
        )
        matched_idle = now - (
            self._last_input_match_time or self._input_started_at
        )
        with self._input_diagnostics_lock:
            rejection_summary = "/".join(
                f"{name}:{count}"
                for name, count in sorted(self._input_rejections.items())
                if count > 0
            ) or "无"
            self._input_rejections.clear()
        self.get_logger().info(
            "WildOS 输入诊断, 频率="
            f"图像前/左/右={rates['image_front']:.1f}/"
            f"{rates['image_left']:.1f}/{rates['image_right']:.1f}Hz, "
            f"内参前/左/右={rates['info_front']:.1f}/"
            f"{rates['info_left']:.1f}/{rates['info_right']:.1f}Hz, "
            f"odom={rates['odom']:.1f}Hz, 导航图={rates['nav_graph']:.1f}Hz, "
            f"相机同步={rates['camera_sync']:.1f}Hz, "
            f"完整匹配={rates['matched']:.1f}Hz, "
            "时间差95%上限="
            f"三相机{deltas['camera_spread'].p95_ms:.0f}ms/"
            f"odom{deltas['odom_delta'].p95_ms:.0f}ms/"
            f"导航图年龄{deltas['nav_graph_age'].p95_ms:.0f}ms, "
            f"距上次相机同步={camera_sync_idle:.1f}s, "
            f"距上次完整匹配={matched_idle:.1f}s, "
            f"缓存=odom:{cache.odom_count}/内参:{cache.camera_info_count}/"
            f"导航图:{'有' if cache.has_nav_graph else '无'}, "
            f"拒绝={rejection_summary}"
        )

    @staticmethod
    def _stamp_nanoseconds(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _stamp_age_seconds(self, stamp) -> float | None:
        """返回当前 ROS 时间相对消息时间的非负年龄"""
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        age = now_seconds - stamp_seconds
        if not np.isfinite(age) or age < 0.0:
            return None
        return age

    def _log_object_missing(self, detection_rejections, confirmation_ready: bool) -> None:
        """记录当前帧的真实拒绝门槛, 并区分窗口和融合目标状态"""
        self._object_missing_log_count += 1
        should_log = (
            self._object_missing_log_count == 1
            or self._object_missing_log_count % self.object_detection_debug_interval == 0
        )
        if not should_log:
            return
        window_state = "仍满足确认门槛" if confirmation_ready else "未满足确认门槛"
        summary = self._object_detection_rejection_summary(detection_rejections)
        self.get_logger().info(
            f"当前帧未形成有效目标, 连续帧数={self._object_missing_log_count}, "
            f"原因={summary}, Mask阈值={self.mask_threshold:.3f}, "
            f"视觉窗口={window_state}, 本帧不发布目标Mask, "
            "已有融合目标不会被当前帧清除"
        )

    def _log_object_detection_pending(self, detection_components) -> None:
        """记录窗口尚未达到证据门槛的目标候选"""
        evidence_count = self.object_detection_confirmation.evidence_count
        if evidence_count != 1:
            return
        summary = self._object_detection_component_summary(detection_components)
        self.get_logger().info(
            f"视觉窗口正在累计, 有效帧={evidence_count}/"
            f"{self.object_detection_confirm_frames}, 窗口占用="
            f"{self.object_detection_confirmation.sample_count}/"
            f"{self.object_detection_confirm_window_frames}, 候选={summary}"
        )

    def _log_object_detection_confirmed(self, detection_components) -> None:
        """确认状态只在窗口首次达标时记录一次"""
        summary = self._object_detection_component_summary(detection_components)
        self.get_logger().info(
            f"当前视觉窗口已通过, 有效帧="
            f"{self.object_detection_confirmation.evidence_count}/"
            f"{self.object_detection_confirm_frames}, 窗口占用="
            f"{self.object_detection_confirmation.sample_count}/"
            f"{self.object_detection_confirm_window_frames}, 已发布目标Mask, "
            f"候选={summary}"
        )

    def _log_object_detection_lost(self) -> None:
        """窗口失效不等于融合目标丢失"""
        self.get_logger().info(
            f"视觉窗口已失效, 有效帧="
            f"{self.object_detection_confirmation.evidence_count}/"
            f"{self.object_detection_confirm_frames}, 窗口占用="
            f"{self.object_detection_confirmation.sample_count}/"
            f"{self.object_detection_confirm_window_frames}, "
            "停止发布当前目标Mask, 历史融合目标继续由融合节点维护"
        )

    @staticmethod
    def _object_detection_component_summary(detection_components) -> str:
        """格式化各相机通过过滤的目标连通域"""
        return ",".join(
            f"{CAMERA_LOG_NAMES.get(component.camera_idx, component.camera_idx)}"
            f"/峰值={component.peak_score:.3f}/像素={component.pixel_count}"
            for component in detection_components
        )

    @staticmethod
    def _object_detection_rejection_summary(detection_rejections) -> str:
        """把过滤器拒绝代码转换成可直接判断的中文门槛信息"""
        reasons = []
        for rejection in detection_rejections:
            camera_name = CAMERA_LOG_NAMES.get(
                rejection.camera_idx,
                rejection.camera_idx,
            )
            if rejection.reason == "peak_below_threshold":
                detail = (
                    f"峰值不足/峰值={rejection.peak_score:.3f}<"
                    f"{rejection.required_peak_score:.3f}/区域像素="
                    f"{rejection.pixel_count}"
                )
            elif rejection.reason == "component_too_small":
                detail = (
                    f"区域过小/像素={rejection.pixel_count}<"
                    f"{rejection.required_pixels}/峰值={rejection.peak_score:.3f}"
                )
            else:
                detail = f"无Mask连通区域/最高分={rejection.peak_score:.3f}"
            reasons.append(f"{camera_name}/{detail}")
        return ",".join(reasons) if reasons else "无过滤诊断"

    def _publish_object_reached(self, binary_mask) -> None:
        reached = self._object_reached_from_mask(binary_mask)
        self.object_reached_publisher.publish(Bool(data=reached))
        if reached and not self._visual_reached_active:
            self.get_logger().info("视觉近距离证据=True, 等待融合目标距离确认")
        self._visual_reached_active = reached

    def _object_reached_from_mask(self, binary_mask) -> bool:
        """视觉近距离只产生候选证据, 不拥有任务完成状态"""
        return self.object_reached_evidence.update(binary_mask)

    def _on_object_search_completed(self, msg: Bool) -> None:
        """Mux 是任务完成唯一所有者, 完成后停止目标检测链"""
        if not msg.data or self.object_search_completed:
            return
        self.object_search_completed = True
        self.get_logger().info("目标搜索任务已由 Mux 确认完成, 停止后续目标检测")

    def _on_object_search_target(self, msg: String) -> None:
        """切换文本特征并清除只属于旧目标的视觉状态"""
        target = normalize_object_search_target(msg.data)
        if target is None:
            self.get_logger().warn("收到空搜索目标, 已忽略")
            return
        if list(self.text_queries) == [target]:
            return

        self.get_logger().info(f"开始切换搜索目标, target={target!r}")
        try:
            with torch.inference_mode():
                text_feats = self.model.forward_on_text([target])
        except Exception as exc:
            self.get_logger().error(
                f"搜索目标文本特征计算失败, 保留原目标, error={exc}"
            )
            return
        self.text_queries = [target]
        self.text_feats = text_feats
        self.object_detection_confirmation.reset()
        self.object_reached_evidence.reset()
        self._object_detection_ready = False
        self._visual_reached_active = False
        self._object_missing_log_count = 0
        self.object_search_completed = False
        self.frontier_uuid_to_scores.clear()
        self.get_logger().info(f"搜索目标已切换, target={target!r}")

    def update_navgraph_with_scores(self, navgraph_msg, geofrontiers, nav_data):
        """只发布当前活动 Frontier 的本帧评分, 不沿用历史视角结果"""
        current_scores = CurrentFrontierScores(self.frontier_uuid_to_scores)
        trav_class_idx = navgraph_msg.trav_classes.index(self.traversability_class)
        # 最新原始图会被多组相机复用, 评分只能修改消息副本
        scored_navgraph = deepcopy(navgraph_msg)
        
        for i in range(self.num_cameras):
            if not geofrontiers[i]:
                continue

            for frontier_node, heading, scores in zip(
                geofrontiers[i]["frontier_nodes"],
                geofrontiers[i]["frontier_headings"],
                nav_data[i]["scores"]
            ):
                uuid = self.uuid_to_str(frontier_node.uuid)

                # 根据朝向对齐程度调制评分
                if self.std_for_frontier_heading is not None:
                    scores *= self.scorer.get_gauss_scores(
                        np.rad2deg(np.arctan2(heading[1], heading[0])),
                        std=self.std_for_frontier_heading,
                        max_score=1.0
                    )

                # 多相机同时看到同一 Frontier 时保留每个方向的最高当前证据
                scores = self.normalize_frontier_scores(scores)
                current_scores.add_visual(uuid, scores, frontier_node)

        scored_navgraph.header.stamp = self.get_clock().now().to_msg()
        for node in scored_navgraph.nodes:
            uuid = self.uuid_to_str(node.uuid)
            if not node.trav_properties[trav_class_idx].is_frontier:
                continue
            if not current_scores.has(uuid):
                # 未评分 frontier 使用默认评分
                scores = self.scorer.get_default_scores(
                    node, trav_class_idx, std=self.std_for_default_scores, def_max_score=self.default_max_score
                ).astype(np.float32)
                current_scores.add_default(uuid, self.normalize_frontier_scores(scores), node)

            scores = self.normalize_frontier_scores(current_scores.scores(uuid))
            current_scores.set_scores(uuid, scores)
            node.properties.append(KeyValue(key="frontier_scores", value=list(scores)))
            node.properties.append(KeyValue(
                key="is_default_scored",
                value=[0.0 if current_scores.is_visual(uuid) else 1.0],
            ))

        self.frontier_uuid_to_scores, removed_uuids, updated_uuids = current_scores.finish()
        return scored_navgraph, removed_uuids, updated_uuids

    @staticmethod
    def normalize_frontier_scores(scores):
        """
        Keep frontier scores finite and bounded for planner cost and RViz display
        """
        scores = np.asarray(scores, dtype=np.float32)
        scores = np.nan_to_num(scores, nan=0.0, posinf=1.0, neginf=0.0)
        scores = np.clip(scores, 0.0, None)
        max_score = float(np.max(scores)) if scores.size > 0 else 0.0
        if max_score > 1.0:
            scores = scores / max_score
        return np.clip(scores, 0.0, 1.0).astype(np.float32)
    
    @staticmethod
    def uuid_to_str(uuid):
        return ''.join([f"{x:03}" for x in uuid.id])
                

def main(args=None):
    rclpy.init(args=args)
    from ament_index_python.packages import get_package_share_directory
    import argparse

    # 分离 ROS 参数和节点自定义参数
    custom_args = rclpy.utilities.remove_ros_args(args)
    
    # 解析剩余的非 ROS 参数
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Configuration file name (YAML) for the node.")
    def str2bool(v):
        return v.lower() in ('true')
    parser.add_argument("--do_object_search", type=str2bool, default=False, help="Enable object search.")
    parser.add_argument("--config-override", action="append", default=[], help="Override config key with key=value.")
    custom_args = parser.parse_args(custom_args[1:])

    conf_name = f"{custom_args.config}"
    if conf_name.endswith(".yaml") is False:
        conf_name += ".yaml"

    conf = Path(conf_name).expanduser()
    if not conf.is_absolute():
        package_share_directory = Path(get_package_share_directory('visual_navigation'))
        conf = package_share_directory / "configs" / conf_name

    config = OmegaConf.load(conf)
    if custom_args.config_override:
        config = OmegaConf.merge(config, OmegaConf.from_dotlist(custom_args.config_override))

    ros2_node = WildOS_Nav(config, do_object_search=custom_args.do_object_search)
    # Keep TF delivery independent from synchronized camera and inference callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros2_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(ros2_node)
        executor.shutdown()
        try:
            ros2_node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
