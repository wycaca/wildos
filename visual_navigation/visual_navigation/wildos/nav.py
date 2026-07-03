import rclpy
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image as ImageMsg, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from graphnav_msgs.msg import NavigationGraph, KeyValue
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from object_search_msgs.msg import ObjectMaskWithTf

from pathlib import Path
from omegaconf import OmegaConf
import cv2
import numpy as np
import torch
from torchvision import transforms

from visual_navigation.utils.tf_lookup_sub import TFEdge, TFLookupSubscriber
from visual_navigation.wildos.goalagnostic_scoring import GoalAgnosticScoring
from visual_navigation.geofrontier_nav.geofrontier_to_image import GeoFrontierToImage
from visual_navigation.wildos.viz import VisualizeGoalAgnosticGeoFrontierScoring
from explorfm import ExploRFMInference
from visual_navigation.utils.object_search_utils import localize_query, get_objectmask_msg

HOME_DIR = Path("/home/ks-server3/han/wildos_ws/src/nebula2-wildos/")
CAMERA_MAPPING = {
    0: "front",
    1: "left",
    2: "right"
}    

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
        "num_angular_bins": 16,
        "reach_in_2D": True,

        # 导航图参数
        "frontiers_range": 12.0,
        "traversability_class": "default",
        "heading_sim_thresh": 0.866,  # 相机朝向和 frontier 朝向的余弦相似度阈值
        "default_max_score": 0.5,
        "std_for_default_scores": 30.0,  # 角度
        "std_for_frontier_heading": 30.0,  # 角度
        "min_frontier_separation": 0.5,  # 米

        # 像素评分参数
        "frontier_threshold": 0.6,
        "frontier_opening_kernel_size": 0,
        "traversability_threshold": 0.8,
        "frontier_w": 2.0,
        "goal_w": 3.0,
        "reachability_w": 2.0,
        "scoring_method": "ADDITIVE",
        "reach_scale": 0.25,
        "compute_paths": True,

        # ROS2 frame 和 topic
        "parent_frame": "spot1/odom",
        "cam_frame": "spot1/realsense/{}_color_optical_frame",
        "camera_img_topic": "/spot1/realsense/{}/color/image_raw/compressed",
        "camera_info_topic": "/spot1/realsense/{}/color/camera_info",
        "odometry_topic": "/spot1/odom",
        "navigation_graph_topic": "/spot1/nav_graph",

        # ROS2 发布 topic
        "scored_navgraph_topic": "/spot1/scored_nav_graph",
        "model_viz_topic": "model_visualization",
        "valid_geofrontiers_topic": "within_range_geofrontiers",
        "score_ring_topic": "/spot1/score_rings",
        "graph_viz_topic": "/spot1/navgraph_viz",

        # ROS2 订阅参数
        "qos_history_depth": 1,
        "syncsub_queue_size": 1,
        "syncsub_slop": 0.2,

        # 目标搜索参数
        "object_search_config": {
            # "text_queries": ["NASA logo"],
            "text_queries": ["orange flag"],
            # "text_queries": ["golf cart"],
            # "text_queries": ["garbage container"],
            "pixel_level_seg": False,
            "mask_threshold": 0.09,#0.08,
            "obj_frontier_score": 0.9,
            "obj_trav_score": 0.9
        },

        # TF 查询配置
        "tf_lookup_config": {
            "buffer_size": 1,       # 消息数量
            "cache_time": 10,       # 秒
            "timer_duration": 0.5,  # 秒
            "lookup_timeout": 0,     # 秒
            "qos_history_depth": 1,  # QoS 队列深度
            "wait_for_oldest": True,  # buffer 满时是否等待
            "clear_buffer_on_process": True,  # 处理后是否清空 buffer
            "spin_thread": False,     # 是否单独线程 spin TF listener
        }
    }

    def __init__(self, config: OmegaConf=OmegaConf.create(), do_object_search=False):
        config = OmegaConf.merge(OmegaConf.create(self.default_config), config)

        # 先初始化模型, 避免 TF listener 过早 spin
        print(f"WildOS 模型初始化开始, object_search={do_object_search}", flush=True)
        np.random.seed(42)
        self.init_model(config, do_object_search)
        print("WildOS 模型初始化完成", flush=True)

        super().__init__(
            node_name='wildos',
            config=config.tf_lookup_config
        )
        self.get_logger().info("WildOS 模型初始化完成")
        if self.object_search_mode:
            self.get_logger().info(f"目标搜索已启用, text_queries={list(self.text_queries)}")

        # 用于转换 ROS 和 OpenCV 图像
        self.br = CvBridge()

        # 导航参数和初始化
        self.num_cameras = config.num_cameras
        self.cam_inverted = config.cams_inverted
        assert self.num_cameras in [1, 3], "Only 1 or 3 cameras are supported."
        self.num_angular_bins = config.num_angular_bins
        self.reach_in_2D = config.reach_in_2D
        
        # 保存当前 frontier node 及其评分
        self.frontier_uuid_to_scores = {}
        self.frontier_uuid_to_scoring_distance = {}
        self.removed_frontier_positions = np.zeros((0, 3), dtype=np.float32)

        # 将导航图 frontier 投影到图像
        self.geofrontier_to_image = GeoFrontierToImage(
            camera_mapping=CAMERA_MAPPING,
            frontiers_range=config.frontiers_range,
            traversability_class=config.traversability_class,
            cams_inverted=self.cam_inverted,
            heading_sim_thresh=config.heading_sim_thresh,
            reach_in_2D=False
        )
        self.traversability_class = config.traversability_class
        self.default_max_score = config.default_max_score
        self.std_for_default_scores = config.std_for_default_scores
        self.std_for_frontier_heading = config.std_for_frontier_heading
        self.min_frontier_separation = config.min_frontier_separation

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

    def init_model(self, config, do_object_search):
        # VLM 初始化
        self.device = "cuda"

        if do_object_search and config.adaptor_version is None:
            config.adaptor_version = "siglip2"
        print(
            f"WildOS 加载视觉模型, model={config.model_version}, adaptor={config.adaptor_version}",
            flush=True,
        )

        # RADIO 模型
        self.model = ExploRFMInference(
            frontier_ckpt=HOME_DIR / "ckpts" / config.frontier_ckpt,
            traversability_ckpt=HOME_DIR / "ckpts" / config.traversability_ckpt,
            model_version=config.model_version,
            adaptor_version=config.adaptor_version,
            adaptor_ckpt_path=HOME_DIR / "ckpts",
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
            self.mask_threshold = config.object_search_config.mask_threshold
            self.obj_frontier_score = config.object_search_config.obj_frontier_score
            self.obj_trav_score = config.object_search_config.obj_trav_score

            assert len(self.text_queries) == 1, "Only single object search is supported in this version."

            print(f"WildOS 计算目标搜索文本特征, text_queries={list(self.text_queries)}", flush=True)
            self.text_feats = self.model.forward_on_text(self.text_queries)
            print("WildOS 目标搜索文本特征计算完成", flush=True)


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
        self.navgraph_vis_pub = self.create_publisher(
            MarkerArray,
            config.graph_viz_topic,
            10
        )
        if self.object_search_mode:
            self.object_mask_publisher = self.create_publisher(
                ObjectMaskWithTf,
                "/spot1/object_mask",
                10
            )

    def init_subscribers(self, config):

        cameraimg_topic_str = config.camera_img_topic
        camerainfo_topic_str = config.camera_info_topic
        img_msg_type = CompressedImage if self.using_compressed_imgs else ImageMsg

        self.camera_subs = {}
        for i in range(self.num_cameras):
            self.camera_subs[i] = {
                "image": Subscriber(self, img_msg_type, cameraimg_topic_str.format(CAMERA_MAPPING[i]), qos_profile=config.qos_history_depth),
                "info": Subscriber(self, CameraInfo, camerainfo_topic_str.format(CAMERA_MAPPING[i]), qos_profile=config.qos_history_depth)
            }
        self.odom_sub = Subscriber(self, Odometry, config.odometry_topic, qos_profile=config.qos_history_depth)
        self.navgraph_sub = Subscriber(self, NavigationGraph, config.navigation_graph_topic, qos_profile=config.qos_history_depth)

        ts_subs = [self.odom_sub, self.navgraph_sub]
        for i in range(self.num_cameras):
            ts_subs.append(self.camera_subs[i]["image"])
            ts_subs.append(self.camera_subs[i]["info"])

        self.ts = ApproximateTimeSynchronizer(
            ts_subs, queue_size=config.syncsub_queue_size, slop=config.syncsub_slop
        )
        self.ts.registerCallback(self.listener_callback)
    
    def listener_callback(self, odom_msg, navgraph_msg, *msgs):
        self.clbk_cntr += 1
        self.get_logger().info(f"Received callback {self.clbk_cntr}")

        assert odom_msg.header.frame_id == self.global_frame, \
            f"Odom frame {odom_msg.header.frame_id} does not match global frame {self.global_frame}"
        # assert navgraph_msg.header.frame_id == self.global_frame, \
        #     f"Navgraph frame {navgraph_msg.header.frame_id} does not match global frame {self.global_frame}"

        self.msg_buffer.add_msg(
            msg={
                "odom": odom_msg,
                "navgraph": navgraph_msg,
                "cam_msgs": msgs
            },
            stamp=odom_msg.header.stamp,
        )

    def do_processing(self, msg, tf_data):
        
        print(f"Started Heavy")

        # 提取消息
        odom_msg = msg["odom"]
        navgraph_msg = msg["navgraph"]
        msgs = msg["cam_msgs"]

        # 提取相机图像和内参
        rgb_imgs, cam_info_msgs = [], []
        for i in range(self.num_cameras):
            if self.using_compressed_imgs:
                convert_func = self.br.compressed_imgmsg_to_cv2
            else:
                convert_func = self.br.imgmsg_to_cv2

            if self.cam_inverted:
                rgb_imgs.append(
                    np.rot90(convert_func(msgs[i * 2], desired_encoding='rgb8'), k=2)
                )
            else:
                rgb_imgs.append(
                    convert_func(msgs[i * 2], desired_encoding='rgb8')
                )
            cam_info_msgs.append(msgs[i * 2 + 1])

        # 从 navgraph_msg 提取 geofrontier
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
            self.get_logger().error(f"Error in extracting geofrontiers: {e}")
            return

        
        # 模型前向推理
        rgb_tensors = [self.transforms(img.copy()) for img in rgb_imgs]
        batch_tensor = torch.stack(rgb_tensors)
        batch_img_traversability, batch_img_frontiers, spatial_feats = self.model.forward(batch_tensor)

        batch_img_frontiers = batch_img_frontiers.cpu().numpy().astype(np.float32)
        batch_img_traversability = batch_img_traversability.cpu().numpy().astype(np.float32)

        if self.object_search_mode:
            if self.model.model_precision.is_fp16():
                spatial_feats = spatial_feats.half()

            text_sim_spatial, binary_mask = localize_query(
                text_feats=self.text_feats,
                spatial_feats=spatial_feats,
                orig_img_shape=rgb_imgs[0].shape[:2],
                pixel_level_seg=self.pixel_level_seg,
                mask_threshold=self.mask_threshold
            )

            if np.sum(binary_mask) > 0:
                tf_list = [tf_data[f"world_from_cam{i}"] for i in range(self.num_cameras)]
                self.object_mask_publisher.publish(
                    get_objectmask_msg(binary_mask, self.cam_inverted, odom_msg, tf_list, cam_info_msgs)
                )
                batch_img_frontiers = np.maximum(batch_img_frontiers, self.obj_frontier_score*binary_mask)
                batch_img_traversability = np.maximum(batch_img_traversability, self.obj_trav_score*binary_mask)
            else:
                self.get_logger().warn("No object detected in the scene, skipping object mask publish.")

        # 给几何 frontier 评分
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

        # 发布评分后的 navgraph
        robot_pos = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ], dtype=np.float32).reshape(1, 3)
        updated_navgraph, removed_uuids, updated_uuids = self.update_navgraph_with_scores(
            navgraph_msg, geofrontiers, nav_data, robot_pos
        )
        self.scored_navgraph_pub.publish(updated_navgraph)
        self.viz.delete_markers(self.withinrange_geofront_pub)
        self.withinrange_geofront_pub.publish(
            self.viz.viz_valid_geofrontiers(geofrontiers, all_cam_data, odom_msg.header, self.geofrontier_viz_colors))

        # 附加源图像时间, 方便 RViz 跟踪 debug 图像流
        model_viz_msg = self.br.cv2_to_imgmsg(
            self.viz.visualize_model_det(nav_data, all_cam_data),
            encoding="rgb8"
        )
        model_viz_msg.header = msgs[0].header
        self.model_viz_pub.publish(model_viz_msg)
        self.navgraph_vis_pub.publish(
            self.viz.visualize_navgraph(
                navgraph_msg,
                self.global_frame,
                self.get_clock().now().to_msg()
            )
        )
        self.score_rings_pub.publish(
            self.viz.visualize_all_heading_scores(
                self.frontier_uuid_to_scores,
                removed_uuids,
                updated_uuids,
                self.global_frame,
                self.get_clock().now().to_msg()
            )
        )
        
        print(f"Finished Heavy")

    def remove_old_frontiers(self, navgraph_msg):
        # 移除不再存在于 navgraph 的 frontier
        trav_class_idx = navgraph_msg.trav_classes.index(self.traversability_class)
        current_uuids = set()
        for node in navgraph_msg.nodes:
            if node.trav_properties[trav_class_idx].is_frontier:
                current_uuids.add(self.uuid_to_str(node.uuid))

        old_uuids = set(self.frontier_uuid_to_scores.keys())
        removed_uuids = []
        for old_uuid in old_uuids:
            if old_uuid not in current_uuids:
                # 保存已移除 frontier 的位置
                del_node = self.frontier_uuid_to_scores[old_uuid][1]
                pos = np.array([
                    del_node.pose.position.x,
                    del_node.pose.position.y,
                    del_node.pose.position.z
                ], dtype=np.float32).reshape(1, 3)
                self.removed_frontier_positions = np.vstack((self.removed_frontier_positions, pos))

                del self.frontier_uuid_to_scores[old_uuid]
                removed_uuids.append(old_uuid)

        return removed_uuids

    def update_navgraph_with_scores(self, navgraph_msg, geofrontiers, nav_data, robot_pos):
        removed_uuids = self.remove_old_frontiers(navgraph_msg)
        updated_uuids = set()
        trav_class_idx = navgraph_msg.trav_classes.index(self.traversability_class)
        scored_navgraph = navgraph_msg
        
        for i in range(self.num_cameras):
            if not geofrontiers[i]:
                continue

            for frontier_node, heading, scores in zip(
                geofrontiers[i]["frontier_nodes"],
                geofrontiers[i]["frontier_headings"],
                nav_data[i]["scores"]
            ):
                uuid = self.uuid_to_str(frontier_node.uuid)
                frontier_pos = np.array([
                    frontier_node.pose.position.x,
                    frontier_node.pose.position.y,
                    frontier_node.pose.position.z
                ], dtype=np.float32).reshape(1, 3)

                # 检查 frontier 是否过近于已移除 frontier
                # 如果过近则把评分置零
                if len(self.removed_frontier_positions) > 0:
                    distances = np.linalg.norm(self.removed_frontier_positions - frontier_pos, axis=1)
                    if np.any(distances < self.min_frontier_separation):
                        scores *= 0.0

                # 只有机器人比之前更接近 frontier 时才更新评分
                node_dist = np.linalg.norm(robot_pos - frontier_pos)
                if uuid in self.frontier_uuid_to_scoring_distance:
                    prev_dist = self.frontier_uuid_to_scoring_distance[uuid]
                    if node_dist < prev_dist:
                        self.frontier_uuid_to_scoring_distance[uuid] = node_dist
                    else:
                        continue
                else:
                    self.frontier_uuid_to_scoring_distance[uuid] = node_dist

                # 根据朝向对齐程度调制评分
                if self.std_for_frontier_heading is not None:
                    scores *= self.scorer.get_gauss_scores(
                        np.rad2deg(np.arctan2(heading[1], heading[0])),
                        std=self.std_for_frontier_heading,
                        max_score=1.0
                    )

                # 更新评分
                scores = self.normalize_frontier_scores(scores)
                updated_uuids.add(uuid)
                self.frontier_uuid_to_scores[uuid] = (scores, frontier_node)

        scored_navgraph.header.stamp = self.get_clock().now().to_msg()
        for node in scored_navgraph.nodes:
            uuid = self.uuid_to_str(node.uuid)
            if uuid in self.frontier_uuid_to_scores:
                scores = self.normalize_frontier_scores(self.frontier_uuid_to_scores[uuid][0])
                self.frontier_uuid_to_scores[uuid] = (scores, self.frontier_uuid_to_scores[uuid][1])
                kv = KeyValue(
                    key = "frontier_scores",
                    value = list(scores)
                )
                node.properties.append(kv)
                kv = KeyValue(
                    key = "is_default_scored",
                    value = [0.0]
                )
                node.properties.append(kv)

            elif node.trav_properties[trav_class_idx].is_frontier:
                # 未评分 frontier 使用默认评分
                scores = self.scorer.get_default_scores(
                    node, trav_class_idx, std=self.std_for_default_scores, def_max_score=self.default_max_score
                ).astype(np.float32)

                node_pos = np.array([
                    node.pose.position.x,
                    node.pose.position.y,
                    node.pose.position.z
                ], dtype=np.float32).reshape(1, 3)

                # 检查 frontier 是否过近于已移除 frontier
                # 如果过近则把评分置零
                if len(self.removed_frontier_positions) > 0:
                    distances = np.linalg.norm(self.removed_frontier_positions - node_pos, axis=1)
                    if np.any(distances < self.min_frontier_separation):
                        scores *= 0.0

                scores = self.normalize_frontier_scores(scores)
                kv = KeyValue(
                    key = "frontier_scores",
                    value = list(scores)
                )
                node.properties.append(kv)
                kv = KeyValue(
                    key = "is_default_scored",
                    value = [1.0]
                )
                node.properties.append(kv)

                updated_uuids.add(uuid)
                self.frontier_uuid_to_scores[uuid] = (scores, node)

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
    try:
        rclpy.spin(ros2_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ros2_node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
