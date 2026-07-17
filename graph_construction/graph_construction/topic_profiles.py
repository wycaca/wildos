from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
import yaml


PROFILE_CONFIG_NAME = "topic_profiles.yaml"

PROFILE_KEY_DESCRIPTIONS = {
    "namespace": "机器人命名空间",
    "ros_domain_id": "ROS_DOMAIN_ID, 用于隔离 DDS graph",
    "rmw_implementation": "当前 elevation 链路默认 RMW 实现",
    "global_frame": "graph 和 planner 使用的全局 frame",
    "parent_frame": "WildOS 视觉节点查询相机 TF 的父 frame",
    "cam_frame": "三相机 optical frame 模板, 使用 front, left, right 填充",
    "odom_parent_frame": "odom adapter 输出 odom 的父 frame",
    "odom_child_frame": "odom adapter 输出 odom 的 child frame",
    "base_frame": "机器人本体 frame",
    "camera_parent_frame": "fallback camera static TF 的父 frame",
    "camera_static_tf_convention": "fallback camera static TF 的坐标约定",
    "camera_image_flip_x": "是否对视觉输入做水平翻转补偿",
    "lidar_parent_frame": "fallback lidar static TF 的父 frame",
    "lidar_frame": "LiDAR frame",
    "pointcloud_axis_mode": "elevation 点云轴向转换模式",
    "pointcloud_output_frame": "elevation 对齐点云输出 frame",
    "pointcloud_input_topic": "elevation 后端使用的原始点云 topic",
    "aligned_lidar_topic": "elevation 对齐后点云 topic",
    "odom_input_topic": "原始 odom topic",
    "odom_output_topic": "适配后 odom topic, 供 scoring 和 planner 使用",
    "planner_odom_topic": "graphnav_planner remap 后使用的 odom topic",
    "elevation_grid_map_topic": "3D elevation GridMap topic",
    "nav_graph_topic": "graph_construction 输出 NavigationGraph topic",
    "graph_construction_viz_topic": "graph_construction MarkerArray 可视化 topic",
    "scored_nav_graph_topic": "WildOS 打分后的 NavigationGraph topic",
    "model_viz_topic": "WildOS 模型输出可视化 topic",
    "valid_geofrontiers_topic": "WildOS 有效几何 frontier 可视化 topic",
    "score_ring_topic": "WildOS heading score 圆环 topic",
    "object_mask_topic": "目标 mask topic",
    "object_target_estimate_topic": "多视角融合目标估计 topic",
    "object_target_estimate_viz_topic": "融合目标协方差可视化 topic",
    "object_target_particles_topic": "目标粒子点云 topic",
    "object_reached_topic": "视觉近距离候选证据 topic",
    "object_search_completed_topic": "Mux 最终任务完成 topic",
    "object_search_initial_goal_distance": "首帧 odom 固定粗目标距离",
    "object_search_initial_goal_heading_deg": "首帧 odom 固定粗目标方向偏移",
    "object_search_mask_threshold": "文本相似度图生成目标 mask 的阈值",
    "object_search_detection_confirm_frames": "视觉目标确认所需证据帧数",
    "object_search_detection_confirm_window_frames": "视觉目标确认滑动窗口帧数",
    "object_target_max_depth": "目标粒子沿相机射线采样的最大深度",
    "visual_frontiers_range": "WildOS 取参与视觉评分的 frontier 范围",
    "visual_frontier_threshold": "视觉 frontier 像素筛选阈值",
    "object_search_detection_debug_interval": "未检测到目标时的诊断日志间隔",
    "object_search_goal_publish_rate": "object_search_goal_mux 发布 /goal_pose 频率",
    "object_search_object_reached_timeout_sec": "目标近距离确认消息超时",
    "object_search_object_reached_max_target_distance": "允许视觉到达确认触发停止的最大目标距离",
    "object_search_reached_mask_fraction": "近距离确认 mask 面积比例阈值",
    "object_search_reached_min_pixel_count": "近距离确认 mask 最小像素数",
    "object_search_reached_confirm_frames": "近距离确认连续帧数",
    "object_search_status_topic": "object_search_goal_mux 状态 topic",
    "goal_pose_topic": "planner 高层 goal 输入 topic",
    "camera_img_topic": "三相机图像 topic 模板",
    "camera_info_topic": "三相机 camera info topic 模板",
}


def load_topic_profile(profile_name: str, profile_file: Optional[str] = None) -> Dict[str, Any]:
    """从安装配置或显式文件加载指定 topic profile"""
    config_path = _resolve_profile_path(profile_file)
    with config_path.open("r", encoding="utf-8") as config_stream:
        config = yaml.safe_load(config_stream) or {}

    profiles = config.get("profiles") or {}
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles)) or "<none>"
        raise ValueError(f"Unknown topic profile '{profile_name}', available profiles: {available}")

    profile = dict(profiles[profile_name])
    profile["name"] = profile_name
    return profile


def profile_key_description(profile_key: str) -> str:
    """给 launch 参数生成可读说明"""
    return PROFILE_KEY_DESCRIPTIONS.get(profile_key, f"topic profile key '{profile_key}'")


def _resolve_profile_path(profile_file: Optional[str]) -> Path:
    if profile_file:
        return Path(profile_file).expanduser()

    share_dir = Path(get_package_share_directory("graph_construction"))
    return share_dir / "configs" / PROFILE_CONFIG_NAME
