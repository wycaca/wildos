#include <graaflib/algorithm/shortest_path/dijkstra_shortest_paths.h>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <unordered_map>

#include "graphnav_planner/planner.hpp"
#include "planner_internal.hpp"

namespace graphnav_planner
{

using namespace detail;

// Build reachable candidates, apply exploration policy, then return only the executable route suffix
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

  // Phase 1, normalize the moving exploration goal and collect graph-local costs
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
  frontier_score_nodes_.clear();

  std::vector<std::tuple<graaf::vertex_id_t, double>> goal_radius_edges;
  std::unordered_map<graaf::vertex_id_t, double> frontier_costs;
  NodeIdsByUuid node_ids_by_uuid;
  size_t frontier_owner_count = 0;

  for (const auto& [id, node] : graph_.get_vertices())
  {
    Eigen::Vector3d node_pos;
    node_pos.x() = node.pose.position.x;
    node_pos.y() = node.pose.position.y;
    node_pos.z() = node.pose.position.z;
    node_ids_by_uuid[uuid_to_string(node.uuid)] = id;

    if (trav_class_idx_ < node.trav_properties.size() && node.trav_properties[trav_class_idx_].is_frontier)
    {
      frontier_owner_count++;
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
      frontier_score_nodes_[id] = node;
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

  // Phase 2, run Dijkstra before adding a virtual goal, candidates cannot shortcut each other
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

  // Phase 3, enrich each reachable Frontier with path, direction, and recovery metadata
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
  // Phase 4, apply committed-route, forward-only, dead-end, and recovery policy
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
    // Phase 5a, a metric target uses the cheapest reachable node inside its radius
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
    // Phase 5b, keep safe forward motion while Frontier ownership is unavailable
    // Frontier 尚未连入机器人所在分量时, 先沿初始方向走到最远可达安全节点
    if (directional_exploration_ &&
        exploration_state_ == ExplorationState::follow_branch)
    {
      std::optional<PathMetadata> safe_forward_route;
      DirectionalMetrics safe_forward_metrics{
        Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0};
      double safe_forward_cost = std::numeric_limits<double>::infinity();
      for (const auto& [node_id, shortest_path] : base_shortest_paths)
      {
        (void)node_id;
        PathMetadata metadata = describe_path(shortest_path);
        if (metadata.points.size() < 2)
        {
          continue;
        }
        const Eigen::Vector3d& endpoint = metadata.points.back();
        if ((endpoint - current_position).norm() < directional_min_forward_progress_)
        {
          continue;
        }
        const DirectionalMetrics metrics = directional_metrics(
          metadata.points,
          endpoint);
        if (metrics.forward_progress < directional_min_forward_progress_ ||
            metrics.backtrack > directional_max_initial_backtrack_ ||
            metrics.alignment <= 0.0)
        {
          continue;
        }
        const bool farther_forward = !safe_forward_route ||
          metrics.forward_progress >
            safe_forward_metrics.forward_progress + 1e-6;
        const bool same_progress_lower_cost = safe_forward_route &&
          std::abs(
            metrics.forward_progress -
            safe_forward_metrics.forward_progress) <= 1e-6 &&
          shortest_path.total_weight < safe_forward_cost;
        if (farther_forward || same_progress_lower_cost)
        {
          safe_forward_route = std::move(metadata);
          safe_forward_metrics = metrics;
          safe_forward_cost = shortest_path.total_weight;
        }
      }
      if (safe_forward_route)
      {
        const bool route_unchanged = is_route_suffix(
          safe_forward_route->node_uuids,
          direct_path_node_uuids_);
        if (!route_unchanged)
        {
          direct_path_node_uuids_ = safe_forward_route->node_uuids;
          direct_path_points_ = safe_forward_route->points;
          exploration_diagnostics_.safe_node_fallbacks++;
          RCLCPP_INFO(
            logger_,
            "Frontier 暂不可达, 先沿初始方向前往安全节点, "
            "终点=(%.2f, %.2f), 前进量=%.2fm, 路径点=%zu",
            safe_forward_route->points.back().x(),
            safe_forward_route->points.back().y(),
            safe_forward_metrics.forward_progress,
            safe_forward_route->points.size());
        }
        directional_blocked_since_.reset();
        directional_alternatives_allowed_ = false;
        set_exploration_state(
          ExplorationState::follow_branch,
          current_time,
          "safe_forward_node_available");
        return {safe_forward_route->points, !route_unchanged};
      }
    }

    bool blocked_timer_started = false;
    if (directional_exploration_ &&
        !directional_alternatives_allowed_ &&
        timing_inputs_healthy)
    {
      if (!directional_blocked_since_)
      {
        directional_blocked_since_ = current_time;
        blocked_timer_started = true;
        RCLCPP_WARN(
          logger_,
          "初始方向没有可执行路线, Frontier owner=%zu, "
          "有限代价=%zu, 可达候选=%zu, 开始阻塞确认",
          frontier_owner_count,
          frontier_costs.size(),
          frontier_candidates.size());
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
          "no_reachable_route");
        RCLCPP_WARN(
          logger_,
          "初始方向持续没有可执行路线, %.1f 秒后检查其他分支",
          blocked_time);
      }
    }
    if (directional_exploration_ &&
        directional_alternatives_allowed_ &&
        exploration_state_ == ExplorationState::choose_branch &&
        frontier_candidates.empty())
    {
      set_exploration_state(
        ExplorationState::exploration_exhausted,
        current_time,
        "no_reachable_branch");
    }
    exploration_diagnostics_.no_route_cycles++;
    RCLCPP_DEBUG(logger_, "没有可达 frontier 或 goal radius node, 跳过路径规划");
    const bool path_changed = branch_released ||
      !direct_path_points_.empty() ||
      blocked_timer_started;
    direct_path_node_uuids_.clear();
    direct_path_points_.clear();
    return {{}, path_changed};
  }

  const BranchRelation selected_relation = classify_branch_candidate(
    *selected_frontier,
    node_ids_by_uuid);
  // Phase 5c, update the committed route and return only its executable suffix
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

}  // namespace graphnav_planner
