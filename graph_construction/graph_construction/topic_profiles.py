from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
import yaml


PROFILE_CONFIG_NAME = "topic_profiles.yaml"

PROFILE_KEY_DESCRIPTIONS = {
    "namespace": "机器人命名空间",
    "ros_domain_id": "ROS_DOMAIN_ID, 用于隔离 DDS graph",
    "rmw_implementation_2d": "2D 链路默认 RMW 实现",
    "rmw_implementation_3d": "elevation/2.5D 链路默认 RMW 实现",
    "global_frame_2d": "2D graph 和 planner 使用的全局 frame",
    "global_frame_3d": "elevation/2.5D graph 和 planner 使用的全局 frame",
    "grid_frame": "2D OccupancyGrid 的 frame",
    "parent_frame": "WildOS 视觉节点查询相机 TF 的父 frame",
    "cam_frame": "三相机 optical frame 模板, 使用 front, left, right 填充",
    "odom_parent_frame": "odom adapter 输出 odom 的父 frame",
    "odom_child_frame": "odom adapter 输出 odom 的 child frame",
    "base_frame": "机器人本体 frame",
    "odom_pose_source_2d": "2D odom adapter 位姿来源, tf 或 message",
    "odom_fallback_to_message_2d": "TF 位姿不可用时是否回退到原始 odom message",
    "camera_parent_frame": "fallback camera static TF 的父 frame",
    "camera_static_tf_convention": "fallback camera static TF 的坐标约定",
    "camera_image_flip_x": "是否对视觉输入做水平翻转补偿",
    "lidar_parent_frame": "fallback lidar static TF 的父 frame",
    "lidar_frame": "LiDAR frame",
    "lidar_assume_input_in_grid_frame": "是否把输入点云 XYZ 直接视为 grid_frame 坐标",
    "pointcloud_axis_mode_3d": "elevation 点云轴向转换模式",
    "pointcloud_output_frame_3d": "elevation 对齐点云输出 frame",
    "lidar_topic": "原始 LiDAR 点云 topic",
    "pointcloud_input_topic": "elevation 后端使用的原始点云 topic",
    "aligned_lidar_topic": "elevation 对齐后点云 topic",
    "launch_livox_grid_builder": "2D 是否启动本仓库 livox_grid_builder",
    "odom_input_topic": "原始 odom topic",
    "odom_output_topic": "适配后 odom topic, 供 scoring 和 planner 使用",
    "grid_odom_topic": "2D grid builder 用于 rolling origin 的 odom topic",
    "grid_origin_mode": "2D grid origin 模式, rolling 或 fixed",
    "grid_resolution": "2D grid 分辨率, 单位 m/cell",
    "grid_local_width": "2D local grid 宽度, 单位 m",
    "grid_local_height": "2D local grid 高度, 单位 m",
    "grid_min_obstacle_height": "2D 障碍点最低高度阈值",
    "grid_max_obstacle_height": "2D 障碍点最高高度阈值",
    "grid_obstacle_inflation_radius": "2D 障碍膨胀半径",
    "grid_origin_snap_to_resolution": "rolling origin 是否按分辨率对齐",
    "grid_force_odd_grid_size": "是否强制 grid cell 数为奇数",
    "grid_robot_clear_radius": "机器人中心强制清空半径",
    "grid_obstacle_detection_mode": "2D 障碍检测模式, ray_height 或 height_diff",
    "grid_height_diff_mark_rays_free": "height_diff 模式是否沿射线标记 free",
    "grid_height_diff_fill_unobserved_as_free": "height_diff 模式是否把未观测区填为 free",
    "grid_height_diff_unknown_border_width": "height_diff 模式保留 unknown 边界宽度",
    "grid_height_diff_obstacle_threshold": "height_diff 模式高度差障碍阈值",
    "grid_high_obstacle_min_height": "height_diff 模式高障碍最低高度",
    "planner_odom_topic": "graphnav_planner remap 后使用的 odom topic",
    "traversability_grid_topic": "2D traversability OccupancyGrid topic",
    "elevation_grid_map_topic": "3D elevation GridMap topic",
    "nav_graph_topic": "graph_construction 输出 NavigationGraph topic",
    "graph_construction_viz_topic": "graph_construction MarkerArray 可视化 topic",
    "scored_nav_graph_topic": "WildOS 打分后的 NavigationGraph topic",
    "model_viz_topic": "WildOS 模型输出可视化 topic",
    "valid_geofrontiers_topic": "WildOS 有效几何 frontier 可视化 topic",
    "score_ring_topic": "WildOS heading score 圆环 topic",
    "graph_viz_topic": "WildOS graph 可视化 topic",
    "object_mask_topic": "目标 mask topic",
    "object_target_pose_topic": "目标语义引导 frontier pose topic",
    "object_target_viz_topic": "目标 frontier 和检测射线可视化 topic",
    "object_reached_topic": "目标近距离确认 topic",
    "object_search_initial_goal_distance": "首帧 odom 固定粗目标距离",
    "object_search_initial_goal_heading_deg": "首帧 odom 固定粗目标方向偏移",
    "object_search_mask_threshold": "文本相似度图生成目标 mask 的阈值",
    "visual_frontiers_range": "WildOS 取参与视觉评分的 frontier 范围",
    "visual_frontier_threshold": "视觉 frontier 像素筛选阈值",
    "object_search_detection_debug_interval": "未检测到目标时的诊断日志间隔",
    "object_search_target_log_period_sec": "目标候选更新日志最小间隔",
    "object_search_goal_publish_rate": "object_search_goal_mux 发布 /goal_pose 频率",
    "object_search_target_timeout_sec": "目标 frontier 输入超时",
    "object_search_latch_target_after_first_detection": "首次检测到目标后是否 latch",
    "object_search_latch_target_timeout_sec": "target latch 超时",
    "object_search_memory_timeout_sec": "目标记忆超时",
    "object_search_memory_goal_distance": "目标记忆引导 goal 距离",
    "object_search_target_reached_radius": "目标 frontier 到达半径",
    "object_search_object_reached_timeout_sec": "目标近距离确认消息超时",
    "object_search_reached_latch_timeout_sec": "目标到达后保持停止 goal 的时间",
    "object_search_object_reached_require_target_distance": "是否要求目标导航点距离足够近才接受视觉到达确认",
    "object_search_object_reached_max_target_distance": "允许视觉到达确认触发停止的最大目标距离",
    "object_search_reached_mask_fraction": "近距离确认 mask 面积比例阈值",
    "object_search_reached_min_pixel_count": "近距离确认 mask 最小像素数",
    "object_search_reached_confirm_frames": "近距离确认连续帧数",
    "object_search_goal_viz_topic": "object_search_goal_mux 目标可视化 topic",
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
