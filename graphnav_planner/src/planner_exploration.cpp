#include <algorithm>
#include <cmath>
#include <limits>

#include "graphnav_planner/planner.hpp"
#include "planner_internal.hpp"

namespace graphnav_planner
{

using namespace detail;

// Clear the committed route without discarding long-lived branch and failure memory
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
  // Accept any clear path progress signal, this permits detours and short lateral motion
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
  // Validate only the untraversed suffix, disappeared nodes behind the robot are irrelevant
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
  // Merge nearby UUID aliases by position and direction, repeated failures extend cooldown
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
  // Prefer stable identity, then ordered topology, and use spatial continuity as fallback
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
  // Rebase to the live graph path, never append a stale cached tail
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
  // Save the complete exploration transaction before target navigation takes ownership
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
  // Resume only an explicitly suspended transaction, stale target states cannot recreate history
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

}  // namespace graphnav_planner
