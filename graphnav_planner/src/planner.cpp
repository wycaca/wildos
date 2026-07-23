#include <rclcpp/rclcpp.hpp>
#include <graaflib/graph.h>
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_paths.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <tuple>

#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{

namespace
{

std::vector<Eigen::Vector3d> remaining_path(
  const std::vector<Eigen::Vector3d>& path,
  const Eigen::Vector3d& current_position)
{
  if (path.empty())
  {
    return {};
  }
  if (path.size() == 1)
  {
    return {current_position};
  }

  // 从最近路径段的前向端截取, 避免最近离散节点落在机器人身后造成回拉
  size_t nearest_segment = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t index = 0; index + 1 < path.size(); ++index)
  {
    const Eigen::Vector3d segment = path[index + 1] - path[index];
    const double squared_length = segment.squaredNorm();
    if (squared_length < 1e-12)
    {
      continue;
    }
    const double interpolation = std::clamp(
      (current_position - path[index]).dot(segment) / squared_length,
      0.0,
      1.0);
    const Eigen::Vector3d projection = path[index] + interpolation * segment;
    const double distance = (current_position - projection).squaredNorm();
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_segment = index;
    }
  }

  std::vector<Eigen::Vector3d> suffix{current_position};
  suffix.insert(suffix.end(), path.begin() + nearest_segment + 1, path.end());
  return suffix;
}

struct PathProgress
{
  double distance;
  size_t segment;
  double next_waypoint_distance;
};

PathProgress path_progress_detail(
  const std::vector<Eigen::Vector3d>& path,
  const Eigen::Vector3d& current_position)
{
  if (path.size() < 2)
  {
    return {0.0, 0, 0.0};
  }

  double best_distance = std::numeric_limits<double>::max();
  double best_progress = 0.0;
  size_t best_segment = 0;
  double best_next_waypoint_distance =
    (path[1] - current_position).norm();
  double accumulated_length = 0.0;
  for (size_t index = 0; index + 1 < path.size(); ++index)
  {
    const Eigen::Vector3d segment = path[index + 1] - path[index];
    const double segment_length = segment.norm();
    if (segment_length < 1e-6)
    {
      continue;
    }
    const double interpolation = std::clamp(
      (current_position - path[index]).dot(segment) /
        (segment_length * segment_length),
      0.0,
      1.0);
    const Eigen::Vector3d projection = path[index] + interpolation * segment;
    const double distance = (current_position - projection).squaredNorm();
    if (distance < best_distance)
    {
      best_distance = distance;
      best_progress = accumulated_length + interpolation * segment_length;
      best_segment = index;
      best_next_waypoint_distance =
        (path[index + 1] - current_position).norm();
    }
    accumulated_length += segment_length;
  }
  return {best_progress, best_segment, best_next_waypoint_distance};
}

std::optional<Eigen::Vector3d> terminal_direction(
  const std::vector<Eigen::Vector3d>& path)
{
  for (size_t index = path.size(); index > 1; --index)
  {
    Eigen::Vector3d direction = path[index - 1] - path[index - 2];
    direction.z() = 0.0;
    if (direction.norm() >= 1e-6)
    {
      return direction.normalized();
    }
  }
  return std::nullopt;
}

bool is_route_suffix(
  const std::vector<std::string>& route,
  const std::vector<std::string>& committed_route)
{
  if (route.empty() || route.size() > committed_route.size())
  {
    return false;
  }
  return std::equal(route.rbegin(), route.rend(), committed_route.rbegin());
}

bool route_has_repeated_nodes(const std::vector<std::string>& route)
{
  std::unordered_set<std::string> visited;
  for (const auto& node_uuid : route)
  {
    if (!visited.insert(node_uuid).second)
    {
      return true;
    }
  }
  return false;
}

const char* selection_reason_name(const std::string& reason)
{
  if (reason == "continuation")
  {
    return "延续当前走廊";
  }
  if (reason == "initial")
  {
    return "首次选择";
  }
  if (reason == "branch_recovery")
  {
    return "当前分支释放后恢复";
  }
  if (reason == "branch_recovery")
  {
    return "恢复历史候选分支";
  }
  if (reason == "initial_forward_route")
  {
    return "选择初始前向路线";
  }
  if (reason == "directional_branch_recovery")
  {
    return "初始方向分支恢复";
  }
  if (reason == "confirmed_forward_blocked")
  {
    return "初始前向确认阻塞后改选";
  }
  if (reason == "dead_end_recovery")
  {
    return "确认死路后分级恢复";
  }
  return "未分类原因";
}

const char* release_reason_name(const std::string& reason)
{
  if (reason == "path_invalid")
  {
    return "路径失效";
  }
  if (reason == "no_path_progress")
  {
    return "路径持续无进展";
  }
  return "未分类原因";
}

}  // namespace

Planner::Planner(rclcpp::Logger logger) : logger_(logger)
{
}

void Planner::set_trav_class(std::string trav_class)
{
  trav_class_ = trav_class;
}

void Planner::start_directional_exploration(
  const Eigen::Vector3d& origin,
  const Eigen::Vector3d& direction,
  double lookahead_distance)
{
  if (directional_exploration_)
  {
    return;
  }

  Eigen::Vector3d planar_direction(direction.x(), direction.y(), 0.0);
  if (planar_direction.norm() < 1e-6)
  {
    RCLCPP_WARN(logger_, "初始探索方向无效, 保持普通目标规划模式");
    return;
  }
  directional_exploration_ = DirectionalExploration{
    origin,
    planar_direction.normalized(),
    std::max(lookahead_distance, 1.0),
  };
  directional_blocked_since_.reset();
  directional_alternatives_allowed_ = false;
  exploration_state_ = ExplorationState::follow_branch;
  exploration_state_since_.reset();
  reset_frontier_branch();
  RCLCPP_INFO(
    logger_,
    "固定初始探索方向, 起点=(%.2f, %.2f), 方向=(%.3f, %.3f), 前视距离=%.1fm",
    origin.x(),
    origin.y(),
    directional_exploration_->direction.x(),
    directional_exploration_->direction.y(),
    directional_exploration_->lookahead_distance);
}

bool Planner::has_directional_exploration() const
{
  return directional_exploration_.has_value();
}

Planner::ExplorationState Planner::exploration_state() const
{
  return exploration_state_;
}

void Planner::update_graph(graphnav_msgs::msg::NavigationGraph::ConstSharedPtr graph)
{
  update_traversal_memory(*graph);
  graph_ = graaf::undirected_graph<graphnav_msgs::msg::Node, double>();
  unexplored_space_map_.reset();
  frontier_scores_.clear();
  for (size_t i = 0; i < graph->nodes.size(); i++)
  {
    graphnav_msgs::msg::Node node = graph->nodes[i];
    graph_.add_vertex(node, i);
  }
  if (graph->nodes.empty())
  {
    RCLCPP_WARN_ONCE(
      logger_,
      "收到空导航图, 保留探索状态并等待下一帧");
    return;
  }
  auto trav_class_it = std::find(graph->trav_classes.begin(), graph->trav_classes.end(), trav_class_);
  if (trav_class_it == graph->trav_classes.end())
  {
    RCLCPP_WARN(logger_, "导航图中缺少可通行类别, 类别=%s", trav_class_.c_str());
    trav_class_idx_ = 0;
    return;
  }
  trav_class_idx_ = std::distance(graph->trav_classes.begin(), trav_class_it);
  for (auto edge : graph->edges)
  {
    if (edge.from_idx >= graph->nodes.size() || edge.to_idx >= graph->nodes.size())
    {
      RCLCPP_WARN(logger_, "导航图边索引越界, 已跳过, 起点=%lu, 终点=%lu, 节点数=%zu",
                  static_cast<unsigned long>(edge.from_idx), static_cast<unsigned long>(edge.to_idx),
                  graph->nodes.size());
      continue;
    }
    if (trav_class_idx_ < edge.traversability.size())
    {
      double weight = edge.traversability[trav_class_idx_].traversability_cost;
      const std::string history_key = stable_edge_key(
        graph->nodes[edge.from_idx].uuid,
        graph->nodes[edge.to_idx].uuid);
      if (traversed_edges_.find(history_key) != traversed_edges_.end())
      {
        // 已走过边保留在图中, 仅增加重复使用代价
        // 当死路只有原路可退时 Dijkstra 仍会选择该边, 不会形成不可达
        weight *= 1.0 + revisit_cost_factor_;
      }
      graph_.add_edge(edge.from_idx, edge.to_idx, weight);
    }
  }
  current_node_idx_ = graph->current_node_idx;
  if (current_node_idx_ >= graph->nodes.size())
  {
    reset_frontier_branch();
    RCLCPP_WARN(logger_, "当前节点索引越界, 跳过规划器更新, 当前索引=%lu, 节点数=%zu",
                static_cast<unsigned long>(current_node_idx_), graph->nodes.size());
    return;
  }
  unexplored_space_map_ = compute_unexplored_space_map();
}

void Planner::update_traversal_memory(const graphnav_msgs::msg::NavigationGraph& graph)
{
  if (graph.nodes.empty() || graph.current_node_idx >= graph.nodes.size())
  {
    return;
  }

  const auto& current_node = graph.nodes[graph.current_node_idx];
  const std::string current_uuid = uuid_to_string(current_node.uuid);
  if (last_current_node_uuid_ && *last_current_node_uuid_ != current_uuid)
  {
    bool nodes_are_adjacent = false;
    for (const auto& edge : graph.edges)
    {
      if (edge.from_idx >= graph.nodes.size() || edge.to_idx >= graph.nodes.size())
      {
        continue;
      }
      const std::string from_uuid = uuid_to_string(graph.nodes[edge.from_idx].uuid);
      const std::string to_uuid = uuid_to_string(graph.nodes[edge.to_idx].uuid);
      if ((from_uuid == *last_current_node_uuid_ && to_uuid == current_uuid) ||
          (to_uuid == *last_current_node_uuid_ && from_uuid == current_uuid))
      {
        nodes_are_adjacent = true;
        traversed_edges_.insert(stable_edge_key(
          graph.nodes[edge.from_idx].uuid,
          graph.nodes[edge.to_idx].uuid));
        break;
      }
    }
    if (!nodes_are_adjacent)
    {
      RCLCPP_DEBUG(
        logger_,
        "current node 跨越非相邻 UUID, 本帧不记录 traversal edge");
    }
  }
  last_current_node_uuid_ = current_uuid;
}

void Planner::reset_frontier_branch()
{
  active_branch_.reset();
  path_invalid_since_.reset();
  path_invalid_frames_ = 0;
}

void Planner::clear_untried_branches()
{
  exploration_memory_.clear();
}

void Planner::remember_untried_branch(
  const std::string& junction_uuid,
  const Eigen::Vector3d& junction_position,
  const std::string& node_uuid,
  const Eigen::Vector3d& position,
  const Eigen::Vector3d& direction)
{
  exploration_memory_.remember(
    junction_uuid,
    junction_position,
    node_uuid,
    position,
    direction);
}

std::optional<Planner::BranchRecord> Planner::untried_branch_for_candidate(
  const FrontierCandidate& candidate) const
{
  return exploration_memory_.find(
    candidate.uuid,
    candidate.position,
    terminal_direction(candidate.path_points),
    frontier_failure_merge_radius_);
}

void Planner::erase_untried_branch(const FrontierCandidate& candidate)
{
  exploration_memory_.erase(
    candidate.uuid,
    candidate.position,
    terminal_direction(candidate.path_points),
    frontier_failure_merge_radius_);
}

void Planner::set_exploration_state(
  ExplorationState state,
  rclcpp::Time current_time,
  const char* reason)
{
  if (exploration_state_ == state)
  {
    return;
  }
  exploration_state_ = state;
  exploration_state_since_ = current_time;
  exploration_diagnostics_.state_transitions++;
  RCLCPP_INFO(
    logger_,
    "探索状态切换, state=%d, reason=%s",
    static_cast<int>(state),
    reason);
}

void Planner::update_active_branch_progress(
  const Eigen::Vector3d& current_position,
  rclcpp::Time current_time)
{
  if (!active_branch_)
  {
    return;
  }

  // 使用实际路径弧长和下一路点接近判断进展, 初始方向只参与候选偏好
  const PathProgress progress = path_progress_detail(
    active_branch_->path_points,
    current_position);
  const bool advanced_segment = progress.segment > active_branch_->progress_segment;
  const bool advanced_along_path =
    progress.distance >= active_branch_->max_path_progress + 0.25;
  const bool approached_next_waypoint =
    progress.segment == active_branch_->progress_segment &&
    progress.next_waypoint_distance <=
      active_branch_->min_next_waypoint_distance - 0.25;
  const Eigen::Vector3d movement =
    current_position - active_branch_->progress_position;
  Eigen::Vector3d next_direction = Eigen::Vector3d::Zero();
  if (progress.segment + 1 < active_branch_->path_points.size())
  {
    next_direction = active_branch_->path_points[progress.segment + 1] -
      active_branch_->progress_position;
  }
  const bool moved_toward_route = movement.norm() >= 0.25 &&
    next_direction.norm() >= 1e-6 &&
    movement.dot(next_direction.normalized()) >= 0.1;
  if (advanced_segment || advanced_along_path || approached_next_waypoint ||
      moved_toward_route)
  {
    active_branch_->progress_time = current_time;
    active_branch_->progress_position = current_position;
  }
  if (advanced_segment)
  {
    active_branch_->progress_segment = progress.segment;
    active_branch_->min_next_waypoint_distance = progress.next_waypoint_distance;
  }
  else if (progress.segment == active_branch_->progress_segment)
  {
    active_branch_->min_next_waypoint_distance = std::min(
      active_branch_->min_next_waypoint_distance,
      progress.next_waypoint_distance);
  }
  active_branch_->max_path_progress = std::max(
    active_branch_->max_path_progress,
    progress.distance);
}

bool Planner::committed_path_invalid(
  const NodeIdsByUuid& node_ids_by_uuid,
  const std::string& current_uuid,
  const Eigen::Vector3d& current_position) const
{
  if (!active_branch_ ||
      active_branch_->path_node_uuids.size() != active_branch_->path_points.size())
  {
    return false;
  }
  if (active_branch_->path_points.empty())
  {
    return true;
  }

  // 从机器人当前路径位置向后检查, 已走过路段失效不影响当前提交分支
  size_t check_begin = 0;
  const auto current_path_it = std::find(
    active_branch_->path_node_uuids.begin(),
    active_branch_->path_node_uuids.end(),
    current_uuid);
  if (current_path_it != active_branch_->path_node_uuids.end())
  {
    check_begin = std::distance(active_branch_->path_node_uuids.begin(), current_path_it);
  }
  else
  {
    const auto nearest = std::min_element(
      active_branch_->path_points.begin(),
      active_branch_->path_points.end(),
      [&current_position](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
        return (lhs - current_position).squaredNorm() <
          (rhs - current_position).squaredNorm();
      });
    check_begin = std::distance(active_branch_->path_points.begin(), nearest);
  }

  for (size_t index = check_begin;
       index + 1 < active_branch_->path_node_uuids.size();
       ++index)
  {
    const auto from_it = node_ids_by_uuid.find(active_branch_->path_node_uuids[index]);
    const auto to_it = node_ids_by_uuid.find(active_branch_->path_node_uuids[index + 1]);
    if (from_it != node_ids_by_uuid.end() && to_it != node_ids_by_uuid.end() &&
        !graph_.has_edge(from_it->second, to_it->second))
    {
      // 两端节点仍存在但 edge 消失才确认失效, 节点短暂缺失继续使用缓存路径
      return true;
    }
  }
  return false;
}

void Planner::release_active_branch(const char* reason, rclcpp::Time current_time)
{
  if (!active_branch_)
  {
    return;
  }
  record_failed_branch(current_time);
  if (std::string(reason) == "path_invalid")
  {
    exploration_diagnostics_.invalid_path_releases++;
  }
  else if (std::string(reason) == "no_path_progress")
  {
    exploration_diagnostics_.stalled_releases++;
  }
  RCLCPP_WARN(
    logger_,
    "%s, 原因=%s(%s), frontier=%s",
    directional_exploration_ ? "释放初始方向探索分支" : "释放目标接近中继分支",
    release_reason_name(reason),
    reason,
    active_branch_->frontier_uuid.c_str());
  reset_frontier_branch();
  if (directional_exploration_)
  {
    set_exploration_state(
      ExplorationState::check_dead_end,
      current_time,
      reason);
  }
}

bool Planner::branch_is_suppressed(
  const std::string& frontier_uuid,
  const Eigen::Vector3d& position,
  const std::optional<Eigen::Vector3d>& direction,
  rclcpp::Time current_time) const
{
  for (const auto& failed : failed_branches_)
  {
    const bool same_uuid = frontier_uuid == failed.frontier_uuid;
    Eigen::Vector3d delta = position - failed.position;
    delta.z() = 0.0;
    bool same_corridor = delta.norm() <= frontier_failure_merge_radius_;
    if (same_corridor && direction && failed.direction &&
        direction->norm() >= 1e-6 && failed.direction->norm() >= 1e-6)
    {
      same_corridor = std::abs(
        direction->normalized().dot(failed.direction->normalized())) >= 0.5;
    }
    if ((same_uuid || same_corridor) &&
        (failed.retry_after - current_time).seconds() > 0.0)
    {
      return true;
    }
  }
  return false;
}

void Planner::record_failed_branch(rclcpp::Time current_time)
{
  if (!active_branch_)
  {
    return;
  }

  FailedBranch* matched = nullptr;
  for (auto& failed : failed_branches_)
  {
    Eigen::Vector3d delta = active_branch_->frontier_position - failed.position;
    delta.z() = 0.0;
    bool same_corridor = delta.norm() <= frontier_failure_merge_radius_;
    if (same_corridor && active_branch_->terminal_direction && failed.direction)
    {
      same_corridor = std::abs(
        active_branch_->terminal_direction->normalized().dot(
          failed.direction->normalized())) >= 0.5;
    }
    if (failed.frontier_uuid == active_branch_->frontier_uuid || same_corridor)
    {
      matched = &failed;
      break;
    }
  }

  if (matched == nullptr)
  {
    failed_branches_.push_back(FailedBranch{
      active_branch_->frontier_uuid,
      active_branch_->frontier_position,
      active_branch_->terminal_direction,
      0,
      current_time,
    });
    matched = &failed_branches_.back();
  }

  matched->frontier_uuid = active_branch_->frontier_uuid;
  matched->position = active_branch_->frontier_position;
  matched->direction = active_branch_->terminal_direction;
  matched->failure_count += 1;
  const double cooldown = frontier_failure_cooldown_ *
    static_cast<double>(std::min<std::uint32_t>(matched->failure_count, 3));
  matched->retry_after = current_time + rclcpp::Duration::from_seconds(cooldown);
  RCLCPP_WARN(
    logger_,
    "记录失败探索走廊, frontier=%s, 位置=(%.2f, %.2f), 失败次数=%u, 冷却=%.1fs",
    matched->frontier_uuid.c_str(),
    matched->position.x(),
    matched->position.y(),
    matched->failure_count,
    cooldown);
}

Planner::BranchRelation Planner::classify_branch_candidate(
  const FrontierCandidate& candidate,
  const NodeIdsByUuid& node_ids_by_uuid) const
{
  if (!active_branch_)
  {
    return BranchRelation::none;
  }
  if (candidate.uuid == active_branch_->frontier_uuid)
  {
    return BranchRelation::same_frontier;
  }
  if (active_branch_->recovering_branch &&
      (candidate.position - active_branch_->frontier_position).norm() <= 2.0)
  {
    // 恢复到保存的分支入口后, 允许附近实时 Frontier 接替入口节点
    return BranchRelation::recovery_handoff;
  }

  // 正常延伸必须完整经过已提交路径尾部, 共享机器人附近的公共前缀不算同一分支
  const auto tail_it = std::find(
    candidate.path_node_uuids.begin(),
    candidate.path_node_uuids.end(),
    active_branch_->frontier_uuid);
  if (tail_it != candidate.path_node_uuids.end())
  {
    const size_t candidate_tail = std::distance(candidate.path_node_uuids.begin(), tail_it);
    if (candidate_tail + 1 < candidate.path_node_uuids.size())
    {
      size_t overlap = 0;
      size_t active_index = active_branch_->path_node_uuids.size();
      size_t candidate_index = candidate_tail + 1;
      while (active_index > 0 && candidate_index > 0 &&
             active_branch_->path_node_uuids[active_index - 1] ==
               candidate.path_node_uuids[candidate_index - 1])
      {
        --active_index;
        --candidate_index;
        ++overlap;
      }
      const bool has_ordered_tail = overlap >= 2 || candidate_tail == 0;
      Eigen::Vector3d extension =
        candidate.path_points[candidate_tail + 1] - active_branch_->frontier_position;
      extension.z() = 0.0;
      const bool extends_forward = !active_branch_->terminal_direction ||
        extension.norm() < 1e-6 ||
        extension.normalized().dot(*active_branch_->terminal_direction) >= 0.0;
      if (has_ordered_tail && extends_forward)
      {
        return BranchRelation::ordered_extension;
      }
    }
  }

  // 仅在旧尾节点从图中消失时启用空间迁移, 防止路口候选借距离阈值抢占分支
  if (node_ids_by_uuid.find(active_branch_->frontier_uuid) != node_ids_by_uuid.end())
  {
    return BranchRelation::none;
  }
  Eigen::Vector3d migration = candidate.position - active_branch_->frontier_position;
  migration.z() = 0.0;
  if (migration.norm() > frontier_continuity_radius_)
  {
    return BranchRelation::none;
  }
  if (active_branch_->terminal_direction && migration.norm() >= 1e-6 &&
      migration.normalized().dot(*active_branch_->terminal_direction) < 0.0)
  {
    return BranchRelation::none;
  }
  return BranchRelation::spatial_migration;
}

int Planner::branch_relation_rank(BranchRelation relation)
{
  switch (relation)
  {
    case BranchRelation::ordered_extension:
    case BranchRelation::recovery_handoff:
      return 0;
    case BranchRelation::spatial_migration:
      return 1;
    case BranchRelation::same_frontier:
      return 2;
    default:
      return 3;
  }
}

bool Planner::extend_active_branch(
  const FrontierCandidate& candidate,
  BranchRelation relation,
  const Eigen::Vector3d& current_position,
  rclcpp::Time current_time)
{
  if (!active_branch_ || relation == BranchRelation::none ||
      relation == BranchRelation::same_frontier)
  {
    return false;
  }

  if (candidate.path_node_uuids.size() != candidate.path_points.size() ||
      route_has_repeated_nodes(candidate.path_node_uuids))
  {
    RCLCPP_WARN(logger_, "候选路线包含重复节点或元数据不一致, 已拒绝分支延伸");
    return false;
  }

  if (should_hold_route_update(candidate, relation, current_time))
  {
    exploration_diagnostics_.held_updates++;
    return false;
  }

  // 分支身份继续保留, 执行路线始终替换为当前节点到新 Frontier 的简单路径
  const bool path_changed = active_branch_->path_node_uuids != candidate.path_node_uuids;
  const bool completes_recovery =
    active_branch_->recovering_branch &&
    !candidate.is_recovery_branch &&
    (relation == BranchRelation::recovery_handoff ||
     relation == BranchRelation::ordered_extension);
  active_branch_->path_node_uuids = candidate.path_node_uuids;
  active_branch_->path_points = candidate.path_points;
  active_branch_->frontier_position = candidate.position;
  active_branch_->frontier_uuid = candidate.uuid;
  active_branch_->terminal_direction = terminal_direction(active_branch_->path_points);
  active_branch_->recovering_branch = candidate.is_recovery_branch;
  active_branch_->directional_backtrack = candidate.directional_backtrack;
  const PathProgress progress = path_progress_detail(
    active_branch_->path_points,
    current_position);
  active_branch_->max_path_progress = progress.distance;
  active_branch_->progress_segment = progress.segment;
  active_branch_->min_next_waypoint_distance = progress.next_waypoint_distance;
  if (path_changed &&
      (relation == BranchRelation::ordered_extension ||
       relation == BranchRelation::recovery_handoff))
  {
    active_branch_->progress_time = current_time;
    active_branch_->progress_position = current_position;
  }
  if (path_changed)
  {
    active_branch_->route_change_time = current_time;
  }
  if (completes_recovery)
  {
    active_branch_->recovering_branch = false;
    const auto local_direction = terminal_direction(candidate.path_points);
    if (directional_exploration_ && local_direction)
    {
      directional_exploration_->origin = current_position;
      directional_exploration_->direction = *local_direction;
      active_branch_->backtrack_limit =
        directional_max_initial_backtrack_;
      active_branch_->directional_backtrack = 0.0;
    }
    set_exploration_state(
      ExplorationState::follow_branch,
      current_time,
      "recovery_handoff");
  }
  return path_changed;
}

bool Planner::should_hold_route_update(
  const FrontierCandidate& candidate,
  BranchRelation relation,
  rclcpp::Time current_time) const
{
  if (!active_branch_ || relation == BranchRelation::recovery_handoff)
  {
    return false;
  }
  const double held_for = (current_time - active_branch_->route_change_time).seconds();
  if (held_for < frontier_route_hold_duration_)
  {
    return true;
  }
  if (relation == BranchRelation::same_frontier)
  {
    return false;
  }
  Eigen::Vector3d frontier_shift = candidate.position - active_branch_->frontier_position;
  frontier_shift.z() = 0.0;
  return frontier_shift.norm() < frontier_min_update_distance_;
}

Planner::ExplorationDiagnostics Planner::take_exploration_diagnostics()
{
  const ExplorationDiagnostics diagnostics = exploration_diagnostics_;
  exploration_diagnostics_ = ExplorationDiagnostics{};
  return diagnostics;
}

// 暂存完整探索上下文, 让目标观察使用普通寻路而不污染分支记忆
bool Planner::suspend_exploration_state()
{
  if (suspended_exploration_ || !directional_exploration_)
  {
    return false;
  }
  suspended_exploration_ = SuspendedExploration{
    directional_exploration_,
    directional_blocked_since_,
    directional_alternatives_allowed_,
    exploration_state_,
    exploration_state_since_,
    active_branch_,
    exploration_memory_,
    failed_branches_,
    path_invalid_since_,
    path_invalid_frames_,
    direct_path_node_uuids_,
    direct_path_points_,
  };
  reset_frontier_branch();
  clear_untried_branches();
  directional_exploration_.reset();
  directional_blocked_since_.reset();
  directional_alternatives_allowed_ = false;
  failed_branches_.clear();
  exploration_state_ = ExplorationState::follow_branch;
  exploration_state_since_.reset();
  direct_path_node_uuids_.clear();
  direct_path_points_.clear();
  return true;
}

// 恢复目标抢占前的方向、活动分支、岔路栈和失败冷却记录
bool Planner::resume_exploration_state()
{
  if (!suspended_exploration_)
  {
    return false;
  }
  directional_exploration_ = suspended_exploration_->directional_exploration;
  directional_blocked_since_ =
    suspended_exploration_->directional_blocked_since;
  directional_alternatives_allowed_ =
    suspended_exploration_->directional_alternatives_allowed;
  exploration_state_ = suspended_exploration_->exploration_state;
  exploration_state_since_ = suspended_exploration_->exploration_state_since;
  active_branch_ = suspended_exploration_->active_branch;
  exploration_memory_ = suspended_exploration_->exploration_memory;
  failed_branches_ = suspended_exploration_->failed_branches;
  path_invalid_since_ = suspended_exploration_->path_invalid_since;
  path_invalid_frames_ = suspended_exploration_->path_invalid_frames;
  direct_path_node_uuids_ =
    suspended_exploration_->direct_path_node_uuids;
  direct_path_points_ = suspended_exploration_->direct_path_points;
  suspended_exploration_.reset();
  return true;
}

void Planner::reset_exploration_state()
{
  suspended_exploration_.reset();
  reset_frontier_branch();
  clear_untried_branches();
  directional_exploration_.reset();
  directional_blocked_since_.reset();
  directional_alternatives_allowed_ = false;
  failed_branches_.clear();
  path_invalid_since_.reset();
  path_invalid_frames_ = 0;
  exploration_state_ = ExplorationState::follow_branch;
  exploration_state_since_.reset();
  direct_path_node_uuids_.clear();
  direct_path_points_.clear();
}

std::string Planner::stable_edge_key(
  const graphnav_msgs::msg::UUID& from_uuid,
  const graphnav_msgs::msg::UUID& to_uuid)
{
  const std::string from = uuid_to_string(from_uuid);
  const std::string to = uuid_to_string(to_uuid);
  return from <= to ? from + "|" + to : to + "|" + from;
}

std::optional<UnexploredSpaceMap> Planner::compute_unexplored_space_map()
{
  if (graph_.get_vertices().empty())
  {
    return std::nullopt;
  }
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (auto& [id, node] : graph_.get_vertices())
  {
    const auto& pos = node.pose.position;
    min_x = std::min(min_x, pos.x);
    max_x = std::max(max_x, pos.x);
    min_y = std::min(min_y, pos.y);
    max_y = std::max(max_y, pos.y);
  }
  if (!std::isfinite(min_x) || !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y) ||
      max_x < min_x || max_y < min_y)
  {
    RCLCPP_WARN(logger_, "导航图边界非法, 跳过未探索区域地图构建");
    return std::nullopt;
  }
  double resolution = 1.0;
  double margin = 10.0;
  UnexploredSpaceMap unexplored_map(min_x, max_x, min_y, max_y, margin, resolution);
  for (auto& [id, node] : graph_.get_vertices())
  {
    if (trav_class_idx_ < node.trav_properties.size())
    {
      double explored_radius = node.trav_properties[trav_class_idx_].explored_radius;
      if (explored_radius > 0)
      {
        const auto& pos = node.pose.position;
        unexplored_map.mark_explored(pos.x, pos.y, explored_radius);
      }
    }
  }
  return unexplored_map;
}

Planner::PlanningResult Planner::plan_to_goal(
  Eigen::Vector3d& goal,
  double goal_radius,
  rclcpp::Time current_time,
  const std::optional<Eigen::Vector3d>& robot_position,
  bool timing_inputs_healthy)
{
  if (!unexplored_space_map_)
  {
    RCLCPP_DEBUG(logger_, "planner 暂无有效导航图, 跳过路径规划");
    return {{}, false};
  }
  const auto& current_node = graph_.get_vertex(current_node_idx_);
  const Eigen::Vector3d graph_current_position(
    current_node.pose.position.x,
    current_node.pose.position.y,
    current_node.pose.position.z);
  const Eigen::Vector3d current_position = robot_position.value_or(graph_current_position);

  Eigen::Vector3d planning_goal = goal;
  if (directional_exploration_)
  {
    // 固定方向使用沿初始轴前移的虚拟目标, 避免越过有限粗目标后被反向吸引
    const double progress = std::max(
      0.0,
      (current_position - directional_exploration_->origin).dot(
        directional_exploration_->direction));
    planning_goal = directional_exploration_->origin +
      directional_exploration_->direction *
      (progress + directional_exploration_->lookahead_distance);
    planning_goal.z() = goal.z();
  }
  unexplored_space_map_->compute_distance_from(planning_goal.x(), planning_goal.y());
  frontier_scores_.clear();

  std::vector<std::tuple<graaf::vertex_id_t, double>> goal_radius_edges;
  std::unordered_map<graaf::vertex_id_t, double> frontier_costs;
  NodeIdsByUuid node_ids_by_uuid;

  for (const auto& [id, node] : graph_.get_vertices())
  {
    Eigen::Vector3d node_pos;
    node_pos.x() = node.pose.position.x;
    node_pos.y() = node.pose.position.y;
    node_pos.z() = node.pose.position.z;
    node_ids_by_uuid[uuid_to_string(node.uuid)] = id;

    if (trav_class_idx_ < node.trav_properties.size() && node.trav_properties[trav_class_idx_].is_frontier)
    {
      double frontier_path_distance = std::numeric_limits<double>::max();
      double frontier_score = -1.0;
      double frontier_cost = std::numeric_limits<double>::max();

      for (const auto& frontier_pt : node.trav_properties[trav_class_idx_].frontier_points)
      {
        double d = unexplored_space_map_->query_distance_to(frontier_pt.x, frontier_pt.y);
        frontier_path_distance = std::min(frontier_path_distance, d);
      }

      // goal 和 node 重合时跳过方向评分, 避免 NaN heading
      Eigen::Vector3d goal_delta = planning_goal - node_pos;
      Eigen::Vector3d heading = goal_delta.norm() > 1e-6 ? goal_delta.normalized() : Eigen::Vector3d::UnitX();
      bool has_frontier_scores = false;
      double frontier_dist_cost_factor = frontier_dist_cost_factor_;
      for (const auto& kv: node.properties)
      {
        if (kv.key == "frontier_scores")
        {
          has_frontier_scores = true;
          int num_bins = kv.value.size();
          if (num_bins == 0)
          {
            continue;
          }
          double angle_per_bin = 2 * M_PI / num_bins;
          double heading_angle = std::atan2(heading.y(), heading.x());
          if (heading_angle < 0)
            heading_angle += 2 * M_PI;
          int best_bin = static_cast<int>(std::round(heading_angle / angle_per_bin)) % num_bins;
          frontier_score = std::clamp(static_cast<double>(kv.value[best_bin]), 0.0, 1.0);
          double planner_score = std::max(frontier_score, 1e-3);
          frontier_dist_cost_factor = 1.0 - frontier_score_factor_ * std::log(planner_score);
          break;
        }
      }
      if (!has_frontier_scores)
      {
        frontier_cost = frontier_path_distance * frontier_dist_cost_factor_;
      }
      else
      {
        frontier_cost = frontier_path_distance * frontier_dist_cost_factor;
      }
      frontier_scores_[id] = std::make_pair(node, std::make_pair(frontier_score, frontier_cost));
      if (std::isfinite(frontier_cost))
      {
        frontier_costs[id] = frontier_cost;
      }
    }

    // 真实目标附近节点优先于 frontier, 直接连接 virtual goal
    double node_goal_dist = (node_pos - planning_goal).norm();
    if (!directional_exploration_ && node_goal_dist < goal_radius)
    {
      double goal_cost = goal_dist_cost_factor_ * node_goal_dist;
      goal_radius_edges.emplace_back(id, goal_cost);
    }
  }

  // 先在不含 virtual goal 的持久图上计算真实 graph path cost
  // 这样多个候选不会通过 virtual goal 互相短接, visited edge 代价也会完整计入排行
  const auto base_shortest_paths = graaf::algorithm::dijkstra_shortest_paths(graph_, current_node_idx_);
  goal_radius_edges.erase(
    std::remove_if(
      goal_radius_edges.begin(),
      goal_radius_edges.end(),
      [&base_shortest_paths](const auto& goal_edge) {
        return base_shortest_paths.find(std::get<0>(goal_edge)) == base_shortest_paths.end();
      }),
    goal_radius_edges.end());
  struct PathMetadata
  {
    std::vector<std::string> node_uuids;
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d initial_direction;
  };
  const auto describe_path = [this, &current_position](const auto& path) {
    PathMetadata metadata{{}, {}, Eigen::Vector3d::Zero()};
    for (const auto path_node_id : path.vertices)
    {
      const auto& path_node = graph_.get_vertex(path_node_id);
      const std::string path_node_uuid = uuid_to_string(path_node.uuid);
      metadata.node_uuids.push_back(path_node_uuid);
      const Eigen::Vector3d path_node_position(
        path_node.pose.position.x,
        path_node.pose.position.y,
        path_node.pose.position.z);
      metadata.points.push_back(path_node_position);
      if (metadata.initial_direction.squaredNorm() < 1e-12)
      {
        const Eigen::Vector3d direction = path_node_position - current_position;
        if (direction.norm() >= 0.25)
        {
          metadata.initial_direction = direction.normalized();
        }
      }
    }
    return metadata;
  };
  const auto semantic_frontier_position = [this](
    const graphnav_msgs::msg::Node& node,
    const Eigen::Vector3d& owner_position) -> Eigen::Vector3d {
    if (!directional_exploration_ || trav_class_idx_ >= node.trav_properties.size())
    {
      return owner_position;
    }
    const auto& frontier_points =
      node.trav_properties[trav_class_idx_].frontier_points;
    if (frontier_points.empty())
    {
      return owner_position;
    }

    // Frontier owner 可位于侧后方, 方向语义使用真实边界点质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& point : frontier_points)
    {
      centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }
    return centroid / static_cast<double>(frontier_points.size());
  };
  struct DirectionalMetrics
  {
    Eigen::Vector3d direction;
    double forward_progress;
    double backtrack;
    double alignment;
  };
  // 用完整路线终点和最大回退量判断语义方向
  const auto directional_metrics = [this, &current_position](
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& frontier_position) {
    DirectionalMetrics metrics{Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0};
    if (!directional_exploration_)
    {
      return metrics;
    }

    const Eigen::Vector3d heading = directional_exploration_->direction;
    const double current_projection =
      (current_position - directional_exploration_->origin).dot(heading);
    double minimum_projection = current_projection;
    for (const auto& point : points)
    {
      const double projection =
        (point - directional_exploration_->origin).dot(heading);
      minimum_projection = std::min(minimum_projection, projection);
    }
    const double frontier_projection =
      (frontier_position - directional_exploration_->origin).dot(heading);
    metrics.forward_progress = frontier_projection - current_projection;
    metrics.backtrack = std::max(0.0, current_projection - minimum_projection);

    metrics.direction = frontier_position - directional_exploration_->origin;
    metrics.direction.z() = 0.0;
    if (metrics.direction.norm() >= 1e-6)
    {
      metrics.direction.normalize();
      metrics.alignment = metrics.direction.dot(heading);
    }
    return metrics;
  };

  std::vector<FrontierCandidate> frontier_candidates;
  for (const auto& [id, frontier_cost] : frontier_costs)
  {
    const auto path_it = base_shortest_paths.find(id);
    if (path_it == base_shortest_paths.end())
    {
      continue;
    }
    const auto& node = graph_.get_vertex(id);
    PathMetadata metadata = describe_path(path_it->second);
    if (metadata.initial_direction.squaredNorm() < 1e-12)
    {
      const Eigen::Vector3d direction(
        node.pose.position.x - current_position.x(),
        node.pose.position.y - current_position.y(),
        node.pose.position.z - current_position.z());
      if (direction.norm() >= 1e-6)
      {
        metadata.initial_direction = direction.normalized();
      }
    }
    const Eigen::Vector3d frontier_position(
      node.pose.position.x,
      node.pose.position.y,
      node.pose.position.z);
    const Eigen::Vector3d semantic_position = semantic_frontier_position(
      node,
      frontier_position);
    const DirectionalMetrics metrics = directional_metrics(
      metadata.points,
      semantic_position);
    frontier_candidates.push_back(FrontierCandidate{
      id,
      frontier_position,
      uuid_to_string(node.uuid),
      std::move(metadata.node_uuids),
      std::move(metadata.points),
      metadata.initial_direction,
      metrics.direction,
      metrics.forward_progress,
      metrics.backtrack,
      metrics.alignment,
      frontier_cost,
      path_it->second.total_weight + frontier_cost,
      0,
      false,
    });
  }

  std::unordered_set<std::string> live_node_uuids;
  live_node_uuids.reserve(node_ids_by_uuid.size());
  for (const auto& [node_uuid, node_id] : node_ids_by_uuid)
  {
    (void)node_id;
    live_node_uuids.insert(node_uuid);
  }
  exploration_memory_.prune_missing_nodes(live_node_uuids);
  const auto& untried_branches = exploration_memory_.untried_branches();
  const auto ordered_recovery_branches = exploration_memory_.recovery_order();
  std::unordered_map<std::string, std::uint64_t> recovery_priority_by_node;
  recovery_priority_by_node.reserve(ordered_recovery_branches.size());
  for (std::size_t index = 0; index < ordered_recovery_branches.size(); ++index)
  {
    recovery_priority_by_node.emplace(
      ordered_recovery_branches[index].node_uuid,
      ordered_recovery_branches.size() - index);
  }
  const auto recovery_priority = [&recovery_priority_by_node](
      const BranchRecord& branch) {
      const auto priority_it = recovery_priority_by_node.find(branch.node_uuid);
      return priority_it == recovery_priority_by_node.end()
        ? std::uint64_t{0}
        : priority_it->second;
    };

  const bool had_active_branch = active_branch_.has_value();
  bool branch_released = false;
  if (timing_inputs_healthy)
  {
    update_active_branch_progress(current_position, current_time);
  }
  else if (active_branch_)
  {
    // 输入异常期间冻结失败计时, 恢复后重新开始连续确认
    active_branch_->start_time = current_time;
    active_branch_->progress_time = current_time;
    path_invalid_since_.reset();
    path_invalid_frames_ = 0;
  }
  if (!timing_inputs_healthy)
  {
    if (directional_blocked_since_)
    {
      directional_blocked_since_ = current_time;
    }
  }
  const bool path_invalid = committed_path_invalid(
    node_ids_by_uuid,
    uuid_to_string(current_node.uuid),
    current_position);
  if (timing_inputs_healthy && path_invalid)
  {
    path_invalid_frames_++;
    if (!path_invalid_since_)
    {
      path_invalid_since_ = current_time;
    }
  }
  else
  {
    path_invalid_since_.reset();
    path_invalid_frames_ = 0;
  }
  if (path_invalid_since_ &&
      path_invalid_frames_ >= path_invalid_confirm_frames_ &&
      (current_time - *path_invalid_since_).seconds() >= path_invalid_confirm_duration_)
  {
    release_active_branch("path_invalid", current_time);
    branch_released = true;
  }

  if (timing_inputs_healthy && frontier_progress_timeout_ > 0.0 && active_branch_ &&
      (current_time - active_branch_->start_time).seconds() >= frontier_progress_start_grace_ &&
      (current_time - active_branch_->progress_time).seconds() >= frontier_progress_timeout_)
  {
    release_active_branch("no_path_progress", current_time);
    branch_released = true;
  }
  if (branch_released && directional_exploration_)
  {
    directional_alternatives_allowed_ = true;
    directional_blocked_since_.reset();
  }
  if (directional_exploration_ &&
      exploration_state_ == ExplorationState::check_dead_end)
  {
    const rclcpp::Time state_since =
      exploration_state_since_.value_or(current_time);
    const double observe_elapsed = (current_time - state_since).seconds();
    if (observe_elapsed >= recovery_observe_duration_)
    {
      set_exploration_state(
        ExplorationState::choose_branch,
        current_time,
        "dead_end_confirmed");
    }
  }
  const auto candidate_direction = [](const FrontierCandidate& candidate) {
    const auto route_direction = terminal_direction(candidate.path_points);
    if (route_direction)
    {
      return route_direction;
    }
    if (candidate.directional_direction.squaredNorm() >= 1e-12)
    {
      return std::optional<Eigen::Vector3d>(candidate.directional_direction.normalized());
    }
    if (candidate.initial_direction.squaredNorm() >= 1e-12)
    {
      return std::optional<Eigen::Vector3d>(candidate.initial_direction.normalized());
    }
    return std::optional<Eigen::Vector3d>{};
  };
  std::optional<FrontierCandidate> selected_frontier;
  std::string selection_reason = branch_released ? "branch_recovery" : "initial";
  double selection_backtrack_limit = directional_max_initial_backtrack_;
  if (goal_radius_edges.empty() && !untried_branches.empty())
  {
    std::unordered_set<std::string> live_frontier_uuids;
    for (auto& candidate : frontier_candidates)
    {
      const auto branch = untried_branch_for_candidate(candidate);
      if (branch)
      {
        candidate.is_recovery_branch = true;
        candidate.recovery_order = recovery_priority(*branch);
      }
      live_frontier_uuids.insert(candidate.uuid);
    }
    for (const auto& [branch_key, branch] : untried_branches)
    {
      (void)branch_key;
      if (live_frontier_uuids.find(branch.node_uuid) != live_frontier_uuids.end())
      {
        continue;
      }
      const auto node_id_it = node_ids_by_uuid.find(branch.node_uuid);
      if (node_id_it == node_ids_by_uuid.end())
      {
        continue;
      }
      const auto path_it = base_shortest_paths.find(node_id_it->second);
      if (path_it == base_shortest_paths.end())
      {
        continue;
      }
      const auto& node = graph_.get_vertex(node_id_it->second);
      PathMetadata metadata = describe_path(path_it->second);
      if (metadata.initial_direction.squaredNorm() < 1e-12)
      {
        if (branch.discovery_direction.squaredNorm() >= 1e-12)
        {
          metadata.initial_direction = branch.discovery_direction;
        }
        else
        {
          const Eigen::Vector3d direction = branch.position - current_position;
          if (direction.norm() >= 1e-6)
          {
            metadata.initial_direction = direction.normalized();
          }
        }
      }
      const Eigen::Vector3d frontier_position(
        node.pose.position.x,
        node.pose.position.y,
        node.pose.position.z);
      DirectionalMetrics metrics = directional_metrics(
        metadata.points,
        frontier_position);
      if (directional_exploration_ && metrics.direction.squaredNorm() < 1e-12 &&
          branch.discovery_direction.squaredNorm() >= 1e-12)
      {
        metrics.direction = branch.discovery_direction.normalized();
        metrics.alignment = metrics.direction.dot(directional_exploration_->direction);
      }
      FrontierCandidate candidate{
        node_id_it->second,
        frontier_position,
        branch.node_uuid,
        std::move(metadata.node_uuids),
        std::move(metadata.points),
        metadata.initial_direction,
        metrics.direction,
        metrics.forward_progress,
        metrics.backtrack,
        metrics.alignment,
        branch_recovery_cost_penalty_,
        path_it->second.total_weight + branch_recovery_cost_penalty_,
        recovery_priority(branch),
        true,
      };
      if (branch_is_suppressed(
          candidate.uuid,
          candidate.position,
          candidate_direction(candidate),
          current_time))
      {
        continue;
      }
      frontier_candidates.push_back(std::move(candidate));
    }
  }

  if (goal_radius_edges.empty() && !selected_frontier && !frontier_candidates.empty())
  {
    std::vector<const FrontierCandidate*> eligible_candidates;
    for (const auto& candidate : frontier_candidates)
    {
      const bool is_failed_branch = branch_is_suppressed(
        candidate.uuid,
        candidate.position,
        candidate_direction(candidate),
        current_time);
      if (!is_failed_branch)
      {
        eligible_candidates.push_back(&candidate);
      }
    }
    const FrontierCandidate* global_best = nullptr;
    const auto recovery_cost = [](const FrontierCandidate* candidate) {
      return candidate->total_cost + candidate->directional_backtrack;
    };
    if (!eligible_candidates.empty())
    {
      global_best = *std::min_element(
        eligible_candidates.begin(),
        eligible_candidates.end(),
        [](const FrontierCandidate* lhs, const FrontierCandidate* rhs) {
          return lhs->total_cost < rhs->total_cost;
        });
    }
    const FrontierCandidate* continuity_best = nullptr;
    BranchRelation continuity_relation = BranchRelation::none;
    if (active_branch_)
    {
      for (const FrontierCandidate* candidate : eligible_candidates)
      {
        const BranchRelation relation = classify_branch_candidate(
          *candidate,
          node_ids_by_uuid);
        if (relation == BranchRelation::none)
        {
          continue;
        }
        if (directional_exploration_ && !active_branch_->recovering_branch &&
            relation != BranchRelation::recovery_handoff &&
            (candidate->directional_backtrack > active_branch_->backtrack_limit ||
             candidate->directional_backtrack >
               active_branch_->directional_backtrack +
               directional_max_continuation_backtrack_ ||
             candidate->directional_forward_progress <
               directional_min_continuation_progress_))
        {
          exploration_diagnostics_.negative_extension_rejections++;
          continue;
        }
        if (continuity_best == nullptr ||
            branch_relation_rank(relation) < branch_relation_rank(continuity_relation) ||
            (branch_relation_rank(relation) == branch_relation_rank(continuity_relation) &&
             candidate->total_cost < continuity_best->total_cost))
        {
          continuity_best = candidate;
          continuity_relation = relation;
        }
      }
    }

    // 活动分支只接受同走廊后继, Frontier 短暂失配时继续执行缓存路径
    // 只有持续无进展触发 stalled recovery 后才允许选择其他分支
    const FrontierCandidate* chosen = continuity_best;
    if (chosen != nullptr)
    {
      selection_reason = "continuation";
    }
    else if (!active_branch_)
    {
      // 前向确认阶段禁止无条件回退到全局最低代价候选
      if (directional_exploration_ && !directional_alternatives_allowed_)
      {
        for (const FrontierCandidate* candidate : eligible_candidates)
        {
          if (candidate->directional_forward_progress < directional_min_forward_progress_ ||
              candidate->directional_backtrack > directional_max_initial_backtrack_ ||
              candidate->directional_alignment <= 0.0)
          {
            continue;
          }
          if (chosen == nullptr ||
              candidate->directional_alignment > chosen->directional_alignment + 1e-6 ||
              (std::abs(candidate->directional_alignment - chosen->directional_alignment) <= 1e-6 &&
               candidate->total_cost < chosen->total_cost))
          {
            chosen = candidate;
          }
        }
        if (chosen != nullptr)
        {
          directional_blocked_since_.reset();
          selection_reason = "initial_forward_route";
        }
        else
        {
          if (!directional_blocked_since_)
          {
            directional_blocked_since_ = current_time;
            const auto best_aligned = std::max_element(
              eligible_candidates.begin(),
              eligible_candidates.end(),
              [](const FrontierCandidate* lhs, const FrontierCandidate* rhs) {
                return lhs->directional_alignment < rhs->directional_alignment;
              });
            if (best_aligned != eligible_candidates.end())
            {
              RCLCPP_WARN(
                logger_,
                "暂无可用初始前向路线, 最佳前进量=%.2fm, 回退量=%.2fm, 方向一致度=%.3f",
                (*best_aligned)->directional_forward_progress,
                (*best_aligned)->directional_backtrack,
                (*best_aligned)->directional_alignment);
            }
          }
          const double blocked_time =
            (current_time - *directional_blocked_since_).seconds();
          if (blocked_time >= directional_block_confirm_timeout_)
          {
            directional_alternatives_allowed_ = true;
            directional_blocked_since_.reset();
            set_exploration_state(
              ExplorationState::check_dead_end,
              current_time,
              "initial_direction_blocked");
            RCLCPP_WARN(
              logger_,
              "初始前向路线持续不可用, %.1f 秒后确认其他分支",
              blocked_time);
          }
          else
          {
            selection_reason = "initial_forward_pending";
          }
        }
      }
      if (directional_exploration_ && directional_alternatives_allowed_ &&
          exploration_state_ != ExplorationState::check_dead_end)
      {
        if (exploration_state_ == ExplorationState::exploration_exhausted &&
            !eligible_candidates.empty())
        {
          set_exploration_state(
            ExplorationState::choose_branch,
            current_time,
            "new_frontier_available");
        }

        // 死路后只选择之前记录的未探索分支
        if (exploration_state_ == ExplorationState::choose_branch)
        {
          for (const FrontierCandidate* candidate : eligible_candidates)
          {
            if (!candidate->is_recovery_branch)
            {
              continue;
            }
            if (chosen == nullptr ||
                candidate->recovery_order > chosen->recovery_order ||
                (candidate->recovery_order == chosen->recovery_order &&
                 recovery_cost(candidate) < recovery_cost(chosen)))
            {
              chosen = candidate;
            }
          }
          if (chosen != nullptr)
          {
            selection_reason = "branch_recovery";
            selection_backtrack_limit =
              std::numeric_limits<double>::infinity();
            set_exploration_state(
              ExplorationState::backtrack,
              current_time,
              "untried_branch_selected");
          }
          else
          {
            set_exploration_state(
              ExplorationState::exploration_exhausted,
              current_time,
              "no_untried_branch");
          }
        }
      }
      else if (!directional_exploration_)
      {
        chosen = global_best;
      }
    }
    if (chosen != nullptr)
    {
      selected_frontier = *chosen;
    }
  }

  if (goal_radius_edges.empty())
  {
    for (const auto& candidate : frontier_candidates)
    {
      if (candidate.is_recovery_branch)
      {
        continue;
      }
      if (selected_frontier && candidate.uuid == selected_frontier->uuid)
      {
        continue;
      }
      if (branch_is_suppressed(
          candidate.uuid,
          candidate.position,
          candidate_direction(candidate),
          current_time))
      {
        continue;
      }
      if (active_branch_ &&
          classify_branch_candidate(candidate, node_ids_by_uuid) != BranchRelation::none)
      {
        continue;
      }
      if (selected_frontier &&
          candidate.initial_direction.squaredNorm() >= 1e-12 &&
          selected_frontier->initial_direction.squaredNorm() >= 1e-12 &&
          candidate.initial_direction.dot(selected_frontier->initial_direction) >= 0.5)
      {
        continue;
      }
      const Eigen::Vector3d discovery_direction =
        candidate.directional_direction.squaredNorm() >= 1e-12 ?
        candidate.directional_direction : candidate.initial_direction;
      remember_untried_branch(
        uuid_to_string(current_node.uuid),
        current_position,
        candidate.uuid,
        candidate.position,
        discovery_direction);
    }
    if (selected_frontier)
    {
      erase_untried_branch(*selected_frontier);
    }
  }

  if (active_branch_ && !selected_frontier && goal_radius_edges.empty())
  {
    // Frontier 短暂消失时继续执行缓存路径, 不发布新版本也不切换其他分支
    return {remaining_path(active_branch_->path_points, current_position), false};
  }

  if (!goal_radius_edges.empty())
  {
    const auto best_goal_edge = std::min_element(
      goal_radius_edges.begin(),
      goal_radius_edges.end(),
      [&base_shortest_paths](const auto& lhs, const auto& rhs) {
        return base_shortest_paths.at(std::get<0>(lhs)).total_weight + std::get<1>(lhs) <
          base_shortest_paths.at(std::get<0>(rhs)).total_weight + std::get<1>(rhs);
      });
    PathMetadata metadata = describe_path(base_shortest_paths.at(std::get<0>(*best_goal_edge)));
    const bool route_unchanged = is_route_suffix(
      metadata.node_uuids,
      direct_path_node_uuids_);
    if (!route_unchanged)
    {
      direct_path_node_uuids_ = metadata.node_uuids;
      direct_path_points_ = metadata.points;
      const auto& target_node = graph_.get_vertex(std::get<0>(*best_goal_edge));
      const Eigen::Vector3d target_node_position(
        target_node.pose.position.x,
        target_node.pose.position.y,
        target_node.pose.position.z);
      const double route_cost =
        base_shortest_paths.at(std::get<0>(*best_goal_edge)).total_weight +
        std::get<1>(*best_goal_edge);
      RCLCPP_INFO(
        logger_,
        "目标附近已有可达节点, 生成直接目标路线, 目标=(%.2f, %.2f), "
        "节点到目标=%.2fm, 路线总代价=%.2f",
        planning_goal.x(),
        planning_goal.y(),
        (target_node_position - planning_goal).norm(),
        route_cost);
    }
    reset_frontier_branch();
    return {metadata.points, !route_unchanged};
  }

  if (!selected_frontier)
  {
    RCLCPP_DEBUG(logger_, "没有可达 frontier 或 goal radius node, 跳过路径规划");
    const bool path_changed = branch_released || !direct_path_points_.empty();
    direct_path_node_uuids_.clear();
    direct_path_points_.clear();
    return {{}, path_changed};
  }

  const BranchRelation selected_relation = classify_branch_candidate(
    *selected_frontier,
    node_ids_by_uuid);
  if (active_branch_ && selected_relation == BranchRelation::same_frontier)
  {
    const bool route_unchanged = is_route_suffix(
      selected_frontier->path_node_uuids,
      active_branch_->path_node_uuids);
    const bool route_update_held = !route_unchanged && should_hold_route_update(
      *selected_frontier,
      selected_relation,
      current_time);
    bool route_update_applied = false;
    if (route_update_held)
    {
      exploration_diagnostics_.held_updates++;
    }
    else if (!route_has_repeated_nodes(selected_frontier->path_node_uuids))
    {
      const bool route_rebased =
        active_branch_->path_node_uuids != selected_frontier->path_node_uuids;
      active_branch_->path_node_uuids = selected_frontier->path_node_uuids;
      active_branch_->path_points = selected_frontier->path_points;
      active_branch_->terminal_direction = terminal_direction(active_branch_->path_points);
      active_branch_->directional_backtrack =
        selected_frontier->directional_backtrack;
      if (route_rebased)
      {
        route_update_applied = !route_unchanged;
        const PathProgress progress = path_progress_detail(
          active_branch_->path_points,
          current_position);
        active_branch_->max_path_progress = progress.distance;
        active_branch_->progress_segment = progress.segment;
        active_branch_->min_next_waypoint_distance = progress.next_waypoint_distance;
        if (route_unchanged)
        {
          active_branch_->progress_time = current_time;
          active_branch_->progress_position = current_position;
        }
        else
        {
          active_branch_->route_change_time = current_time;
        }
      }
    }
    if (route_update_applied)
    {
      exploration_diagnostics_.route_changes++;
      exploration_diagnostics_.continuation_updates++;
    }
    return {
      remaining_path(active_branch_->path_points, current_position),
      route_update_applied,
    };
  }

  bool path_changed = false;
  if (active_branch_ && selected_relation != BranchRelation::none)
  {
    path_changed = extend_active_branch(
      *selected_frontier,
      selected_relation,
      current_position,
      current_time);
  }
  else
  {
    std::optional<Eigen::Vector3d> branch_direction =
      terminal_direction(selected_frontier->path_points);
    const PathProgress initial_progress = path_progress_detail(
      selected_frontier->path_points,
      current_position);
    active_branch_ = ActiveBranch{
      selected_frontier->position,
      selected_frontier->uuid,
      current_time,
      current_time,
      initial_progress.distance,
      initial_progress.segment,
      initial_progress.next_waypoint_distance,
      current_position,
      selected_frontier->path_node_uuids,
      selected_frontier->path_points,
      branch_direction,
      selected_frontier->is_recovery_branch,
      directional_exploration_ ? selection_backtrack_limit :
        std::numeric_limits<double>::infinity(),
      selected_frontier->directional_backtrack,
      current_time,
    };
    if (directional_exploration_ &&
        !selected_frontier->is_recovery_branch)
    {
      set_exploration_state(
        ExplorationState::follow_branch,
        current_time,
        "branch_selected");
    }
    path_changed = true;
  }
  if (path_changed)
  {
    exploration_diagnostics_.route_changes++;
    if (selection_reason == "continuation")
    {
      exploration_diagnostics_.continuation_updates++;
    }
    else if (selection_reason == "branch_recovery")
    {
      exploration_diagnostics_.branch_recoveries++;
    }
    else if (selection_reason == "directional_branch_recovery")
    {
      exploration_diagnostics_.directional_recoveries++;
    }
    else if (selection_reason == "dead_end_recovery")
    {
      exploration_diagnostics_.dead_end_recoveries++;
    }
    const bool branch_extended = had_active_branch && !branch_released;
    if (directional_exploration_)
    {
      RCLCPP_INFO(
        logger_,
        "%s, 原因=%s(%s), frontier=%s, 中继点=(%.2f, %.2f), "
        "前进量=%.2fm, 回退量=%.2fm, 方向一致度=%.3f, 总代价=%.2f",
        branch_extended ? "初始方向探索分支延伸" : "初始方向探索分支选择",
        selection_reason_name(selection_reason),
        selection_reason.c_str(),
        selected_frontier->uuid.c_str(),
        selected_frontier->position.x(),
        selected_frontier->position.y(),
        selected_frontier->directional_forward_progress,
        selected_frontier->directional_backtrack,
        selected_frontier->directional_alignment,
        selected_frontier->total_cost);
    }
    else
    {
      RCLCPP_INFO(
        logger_,
        "%s, 原因=%s(%s), frontier=%s, 目标=(%.2f, %.2f), "
        "中继点=(%.2f, %.2f), 中继到目标=%.2fm, 总代价=%.2f",
        branch_extended ? "目标接近中继分支延伸" : "目标接近中继分支选择",
        selection_reason_name(selection_reason),
        selection_reason.c_str(),
        selected_frontier->uuid.c_str(),
        planning_goal.x(),
        planning_goal.y(),
        selected_frontier->position.x(),
        selected_frontier->position.y(),
        (selected_frontier->position - planning_goal).norm(),
        selected_frontier->total_cost);
    }
  }
  direct_path_node_uuids_.clear();
  direct_path_points_.clear();
  if (!active_branch_)
  {
    return {{}, path_changed};
  }
  return {remaining_path(active_branch_->path_points, current_position), path_changed};
}

visualization_msgs::msg::MarkerArray Planner::get_score_visualization(const rclcpp::Time& stamp,
                                                                      std::string frame_id,
                                                                      bool with_id_text) const
{
  visualization_msgs::msg::MarkerArray markers;

  // delete all previous markers
  visualization_msgs::msg::Marker delete_all_infcost;
  delete_all_infcost.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_infcost.header.frame_id = frame_id;
  delete_all_infcost.header.stamp = stamp;
  delete_all_infcost.ns = "frontier_inf_cost";
  delete_all_infcost.id = 0;
  markers.markers.push_back(delete_all_infcost);

  visualization_msgs::msg::Marker delete_all_idtext;
  delete_all_idtext.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_idtext.header.frame_id = frame_id;
  delete_all_idtext.header.stamp = stamp;
  delete_all_idtext.ns = "frontier_id_text";
  delete_all_idtext.id = 0;
  markers.markers.push_back(delete_all_idtext);

  // 显示当前分支继承半径, 用于确认拐角处是否保持同一探索方向
  if (active_branch_) {
    visualization_msgs::msg::Marker frontier_marker;
    frontier_marker.header.frame_id = frame_id;
    frontier_marker.header.stamp = stamp;
    frontier_marker.ns = "local_frontier";
    frontier_marker.id = 0;
    frontier_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    frontier_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_marker.pose.position.x = active_branch_->frontier_position.x();
    frontier_marker.pose.position.y = active_branch_->frontier_position.y();
    frontier_marker.pose.position.z = active_branch_->frontier_position.z();
    frontier_marker.scale.x = frontier_continuity_radius_ * 2;
    frontier_marker.scale.y = frontier_continuity_radius_ * 2;
    frontier_marker.scale.z = 0.1;
    frontier_marker.color.a = 0.5;
    frontier_marker.color.r = 0.0;
    frontier_marker.color.g = 1.0;
    frontier_marker.color.b = 0.0;
    markers.markers.push_back(frontier_marker);
  }
  else{
    // delete
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "local_frontier";
    delete_marker.id = 0;
    markers.markers.push_back(delete_marker);
  }

  // 显示距离当前分支判定为无进展还剩多少时间
  if (active_branch_)
  {
    double time_diff = (stamp - active_branch_->progress_time).seconds();
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = stamp;
    text_marker.ns = "local_frontier_time";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = active_branch_->frontier_position.x() + frontier_continuity_radius_;
    text_marker.pose.position.y = active_branch_->frontier_position.y() + frontier_continuity_radius_;
    text_marker.pose.position.z = active_branch_->frontier_position.z() + 1.0;  // raise text above the node
    text_marker.scale.z = 1.5;           // only scale.z is used for text
    text_marker.color.a = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    std::ostringstream ss;
    ss << "Branch continuity\nTimeout: " << frontier_progress_timeout_ - time_diff;
    text_marker.text = ss.str();
    markers.markers.push_back(text_marker);
  }

  // visualize cubes for nodes with inf frontier cost
  int id_counter = 1;

  // normalize costs to [0,1] for visualization
  double min_val = 30.0;
  double max_val = 400.0;

  for (const auto& [id, score_pair] : frontier_scores_)
  {
    const auto& node = score_pair.first;
    double frontier_score = score_pair.second.first;
    double frontier_cost = score_pair.second.second;
    // if (frontier_cost == std::numeric_limits<double>::infinity() && frontier_score > 0)
    if (true)
    {
      double norm_cost = (frontier_cost - min_val) / (max_val - min_val);
      norm_cost = std::min(1.0, std::max(0.0, norm_cost));
      std_msgs::msg::ColorRGBA color = colormapJet(norm_cost);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "frontier_inf_cost";
      marker.id = id_counter;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = node.pose;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      // marker.color.a = 0.5;
      // marker.color.r = 1.0;
      // marker.color.g = 0.0;
      // marker.color.b = 0.0;
      marker.color = color;
      markers.markers.push_back(marker);
    }
    if (with_id_text)
    {
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = stamp;
      text_marker.ns = "frontier_id_text";
      text_marker.id = id_counter;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose = node.pose;
      text_marker.pose.position.z += 0.2;  // raise text above the node
      text_marker.scale.z = 0.28;          // only scale.z is used for text
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      std::ostringstream ss;
      // ss << "Score: " << frontier_score << " Cost: " << frontier_cost;
      // upto 2 decimal places
      ss << std::fixed << std::setprecision(2);
      if (std::isfinite(frontier_score) && frontier_score >= 0.0)
      {
        ss << std::clamp(frontier_score, 0.0, 1.0);
      }
      else
      {
        ss << "--";
      }
      text_marker.text = ss.str();
      markers.markers.push_back(text_marker);
    }
    id_counter++;
  }
  // }
  return markers;
}


}  // namespace graphnav_planner
