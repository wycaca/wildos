#include <rclcpp/rclcpp.hpp>
#include <graaflib/graph.h>
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_path.h>
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_paths.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <map>
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

  const auto nearest = std::min_element(
    path.begin(),
    path.end(),
    [&current_position](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
      return (lhs - current_position).squaredNorm() < (rhs - current_position).squaredNorm();
    });
  return std::vector<Eigen::Vector3d>(nearest, path.end());
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
  reset_frontier_branch();
  RCLCPP_INFO(
    logger_,
    "固定初始探索方向, origin=(%.2f, %.2f), direction=(%.3f, %.3f), lookahead=%.1f",
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
    reset_frontier_branch();
    RCLCPP_WARN_ONCE(logger_, "收到空导航图, 跳过 planner 更新");
    return;
  }
  auto trav_class_it = std::find(graph->trav_classes.begin(), graph->trav_classes.end(), trav_class_);
  if (trav_class_it == graph->trav_classes.end())
  {
    RCLCPP_WARN(logger_, "Traversability class %s not found in graph", trav_class_.c_str());
    trav_class_idx_ = 0;
    return;
  }
  trav_class_idx_ = std::distance(graph->trav_classes.begin(), trav_class_it);
  for (auto edge : graph->edges)
  {
    if (edge.from_idx >= graph->nodes.size() || edge.to_idx >= graph->nodes.size())
    {
      RCLCPP_WARN(logger_, "导航图 edge 越界, 跳过 edge, from=%lu, to=%lu, nodes=%zu",
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
    RCLCPP_WARN(logger_, "current_node_idx 越界, 跳过 planner 更新, current=%lu, nodes=%zu",
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
}

void Planner::clear_deferred_branches()
{
  deferred_branches_.clear();
  next_deferred_branch_order_ = 0;
}

void Planner::remember_deferred_branch(
  const std::string& node_uuid,
  const Eigen::Vector3d& position,
  const Eigen::Vector3d& direction)
{
  auto existing = deferred_branches_.find(node_uuid);
  if (existing != deferred_branches_.end())
  {
    existing->second.position = position;
    existing->second.discovery_direction = direction;
    return;
  }
  deferred_branches_.emplace(
    node_uuid,
    DeferredBranch{node_uuid, position, direction, next_deferred_branch_order_++});
}

void Planner::reset_exploration_state()
{
  reset_frontier_branch();
  clear_deferred_branches();
  directional_exploration_.reset();
  stalled_frontier_.reset();
  stalled_frontier_uuid_.reset();
  stalled_branch_direction_.reset();
  stalled_frontier_until_.reset();
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
    RCLCPP_WARN(logger_, "导航图边界非法, 跳过 unexplored map 构建");
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

std::vector<Eigen::Vector3d> Planner::plan_to_goal(
  Eigen::Vector3d& goal,
  double goal_radius,
  rclcpp::Time current_time,
  const std::optional<Eigen::Vector3d>& robot_position)
{
  if (!unexplored_space_map_)
  {
    RCLCPP_DEBUG(logger_, "planner 暂无有效导航图, 跳过路径规划");
    return {};
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

  struct FrontierCandidate
  {
    graaf::vertex_id_t id;
    Eigen::Vector3d position;
    std::string uuid;
    bool inherits_latest_branch;
    std::vector<std::string> path_node_uuids;
    Eigen::Vector3d initial_direction;
    double frontier_cost;
    double total_cost;
    bool is_deferred;
  };

  std::vector<std::tuple<graaf::vertex_id_t, double>> goal_radius_edges;
  std::unordered_map<graaf::vertex_id_t, double> frontier_costs;
  std::unordered_map<std::string, graaf::vertex_id_t> node_ids_by_uuid;

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
    bool inherits_latest_branch;
    std::vector<std::string> node_uuids;
    Eigen::Vector3d initial_direction;
  };
  const auto describe_path = [this, &current_position](const auto& path) {
    PathMetadata metadata{false, {}, Eigen::Vector3d::Zero()};
    bool is_first_path_node = true;
    for (const auto path_node_id : path.vertices)
    {
      if (is_first_path_node)
      {
        is_first_path_node = false;
        continue;
      }
      const auto& path_node = graph_.get_vertex(path_node_id);
      const std::string path_node_uuid = uuid_to_string(path_node.uuid);
      metadata.node_uuids.push_back(path_node_uuid);
      if (active_branch_ &&
          (path_node_uuid == active_branch_->frontier_uuid ||
           active_branch_->path_node_uuids.find(path_node_uuid) !=
             active_branch_->path_node_uuids.end()))
      {
        metadata.inherits_latest_branch = true;
      }
      if (metadata.initial_direction.squaredNorm() < 1e-12)
      {
        const Eigen::Vector3d path_node_position(
          path_node.pose.position.x,
          path_node.pose.position.y,
          path_node.pose.position.z);
        const Eigen::Vector3d direction = path_node_position - current_position;
        if (direction.norm() >= 0.25)
        {
          metadata.initial_direction = direction.normalized();
        }
      }
    }
    return metadata;
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
    frontier_candidates.push_back(FrontierCandidate{
      id,
      Eigen::Vector3d(
        node.pose.position.x,
        node.pose.position.y,
        node.pose.position.z),
      uuid_to_string(node.uuid),
      metadata.inherits_latest_branch,
      std::move(metadata.node_uuids),
      metadata.initial_direction,
      frontier_cost,
      path_it->second.total_weight + frontier_cost,
      false,
    });
  }

  for (auto it = deferred_branches_.begin(); it != deferred_branches_.end();)
  {
    if (node_ids_by_uuid.find(it->first) == node_ids_by_uuid.end())
    {
      it = deferred_branches_.erase(it);
    }
    else
    {
      ++it;
    }
  }

  const auto continues_selected_branch = [this, &current_position](const FrontierCandidate& candidate) {
    if (!active_branch_)
    {
      return false;
    }
    if (active_branch_->recovering_deferred &&
        (candidate.position - active_branch_->frontier_position).norm() <= 2.0)
    {
      // 到达保存的分支入口后允许接管附近当前 Frontier, 不再沿用旧视觉分数
      return true;
    }
    const bool has_topology_overlap =
      candidate.uuid == active_branch_->frontier_uuid || candidate.inherits_latest_branch;
    if (has_topology_overlap)
    {
      // 旧路径节点可能已经位于机器人后方, 只有首段不反向时才允许拓扑重叠继承
      // 这避免候选沿历史节点回退时被误判为当前走廊的自然延伸
      return !active_branch_->direction || candidate.initial_direction.squaredNorm() < 1e-12 ||
        candidate.initial_direction.dot(*active_branch_->direction) >= 0.0;
    }
    if ((candidate.position - active_branch_->frontier_position).norm() >
        frontier_continuity_radius_)
    {
      return false;
    }

    // 空间距离只能作为 UUID 暂时不连续时的回退条件
    // 额外要求方向夹角不超过 60 度, 避免路口另一侧 frontier 被误认成同一分支
    const Eigen::Vector3d previous_heading =
      active_branch_->frontier_position - current_position;
    const Eigen::Vector3d candidate_heading = candidate.position - current_position;
    if (previous_heading.norm() < 1e-6 || candidate_heading.norm() < 1e-6)
    {
      return true;
    }
    return previous_heading.normalized().dot(candidate_heading.normalized()) >= 0.5;
  };

  const bool had_active_branch = active_branch_.has_value();
  if (active_branch_ &&
      (current_position - active_branch_->progress_position).norm() >= 0.25)
  {
    // 活动分支暂时失配时仍按真实 odom 更新进度, 机器人仍在执行旧路径时不误判死路
    active_branch_->progress_position = current_position;
    active_branch_->progress_time = current_time;
  }

  bool branch_stalled = false;
  if (frontier_progress_timeout_ > 0.0 && active_branch_ &&
      (current_time - active_branch_->progress_time).seconds() >= frontier_progress_timeout_)
  {
    // 当前分支持续无进展时暂时屏蔽其空间邻域
    // 屏蔽时间复用 progress timeout, 避免再引入一组 blacklist 参数
    stalled_frontier_ = active_branch_->frontier_position;
    stalled_frontier_uuid_ = active_branch_->frontier_uuid;
    stalled_branch_direction_ = active_branch_->direction;
    stalled_frontier_until_ = current_time + rclcpp::Duration::from_seconds(frontier_progress_timeout_);
    branch_stalled = true;
    RCLCPP_WARN(
      logger_,
      "探索分支无进展, timeout=%.1f, frontier=%s",
      frontier_progress_timeout_,
      active_branch_->frontier_uuid.c_str());
    reset_frontier_branch();
  }
  if (stalled_frontier_until_ &&
      (current_time - *stalled_frontier_until_).seconds() >= 0.0)
  {
    stalled_frontier_.reset();
    stalled_frontier_uuid_.reset();
    stalled_branch_direction_.reset();
    stalled_frontier_until_.reset();
  }

  const auto matches_stalled_branch = [this](
    const std::string& uuid,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction) {
    if (!stalled_frontier_)
    {
      return false;
    }
    if (stalled_frontier_uuid_ && uuid == *stalled_frontier_uuid_)
    {
      return true;
    }
    if ((position - *stalled_frontier_).norm() > frontier_continuity_radius_)
    {
      return false;
    }
    if (stalled_branch_direction_ && direction.squaredNorm() >= 1e-12)
    {
      return direction.dot(*stalled_branch_direction_) >= 0.5;
    }
    return true;
  };

  std::optional<FrontierCandidate> selected_frontier;
  std::string selection_reason = branch_stalled ? "stalled_recovery" : "initial";
  if (goal_radius_edges.empty() && branch_stalled && !deferred_branches_.empty())
  {
    const DeferredBranch* oldest_branch = nullptr;
    std::optional<FrontierCandidate> oldest_candidate;
    for (const auto& [branch_uuid, branch] : deferred_branches_)
    {
      const auto node_id_it = node_ids_by_uuid.find(branch_uuid);
      if (node_id_it == node_ids_by_uuid.end())
      {
        continue;
      }
      const auto path_it = base_shortest_paths.find(node_id_it->second);
      if (path_it == base_shortest_paths.end())
      {
        continue;
      }
      if (matches_stalled_branch(branch_uuid, branch.position, branch.discovery_direction))
      {
        continue;
      }
      if (oldest_branch != nullptr &&
          oldest_branch->discovery_order <= branch.discovery_order)
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
      oldest_branch = &branch;
      oldest_candidate = FrontierCandidate{
        node_id_it->second,
        Eigen::Vector3d(
          node.pose.position.x,
          node.pose.position.y,
          node.pose.position.z),
        branch_uuid,
        false,
        std::move(metadata.node_uuids),
        metadata.initial_direction,
        0.0,
        path_it->second.total_weight,
        true,
      };
    }
    if (oldest_candidate)
    {
      selected_frontier = std::move(oldest_candidate);
      selection_reason = "deferred_recovery";
    }
  }

  if (goal_radius_edges.empty() && !selected_frontier && !frontier_candidates.empty())
  {
    std::vector<const FrontierCandidate*> eligible_candidates;
    for (const auto& candidate : frontier_candidates)
    {
      const bool is_stalled_branch = matches_stalled_branch(
        candidate.uuid,
        candidate.position,
        candidate.initial_direction);
      if (!is_stalled_branch)
      {
        eligible_candidates.push_back(&candidate);
      }
    }
    const FrontierCandidate* global_best = nullptr;
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
    if (active_branch_)
    {
      for (const FrontierCandidate* candidate : eligible_candidates)
      {
        if (!continues_selected_branch(*candidate))
        {
          continue;
        }
        if (continuity_best == nullptr || candidate->total_cost < continuity_best->total_cost)
        {
          continuity_best = candidate;
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
      if (directional_exploration_ && !branch_stalled)
      {
        for (const FrontierCandidate* candidate : eligible_candidates)
        {
          if (candidate->initial_direction.squaredNorm() < 1e-12 ||
              candidate->initial_direction.dot(directional_exploration_->direction) < 0.0)
          {
            continue;
          }
          if (chosen == nullptr || candidate->total_cost < chosen->total_cost)
          {
            chosen = candidate;
          }
        }
        selection_reason = chosen != nullptr ? "initial_heading" : "initial_heading_blocked";
      }
      chosen = chosen != nullptr ? chosen : global_best;
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
      if (selected_frontier && candidate.uuid == selected_frontier->uuid)
      {
        continue;
      }
      if (matches_stalled_branch(candidate.uuid, candidate.position, candidate.initial_direction))
      {
        continue;
      }
      if (active_branch_ && continues_selected_branch(candidate))
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
      remember_deferred_branch(candidate.uuid, candidate.position, candidate.initial_direction);
    }
    if (selected_frontier)
    {
      deferred_branches_.erase(selected_frontier->uuid);
    }
  }

  if (active_branch_ && !selected_frontier && goal_radius_edges.empty())
  {
    // 缓存路径只保留离机器人最近点之后的后缀, 避免重复发布已走过路段造成回头
    if (active_branch_->direction &&
        (active_branch_->frontier_position - current_position).dot(*active_branch_->direction) < 0.0)
    {
      return {current_position};
    }
    return remaining_path(active_branch_->path_points, current_position);
  }

  if (goal_radius_edges.empty() && !selected_frontier)
  {
    RCLCPP_DEBUG(logger_, "没有可达 frontier 或 goal radius node, 跳过路径规划");
    return {};
  }

  graphnav_msgs::msg::Node virtual_goal_node;
  virtual_goal_node.pose.position.x = planning_goal.x();
  virtual_goal_node.pose.position.y = planning_goal.y();
  virtual_goal_node.pose.position.z = planning_goal.z();
  graaf::vertex_id_t virtual_goal = graph_.add_vertex(virtual_goal_node);
  for (const auto& [id, goal_cost] : goal_radius_edges)
  {
    graph_.add_edge(id, virtual_goal, goal_cost);
  }
  if (selected_frontier)
  {
    graph_.add_edge(selected_frontier->id, virtual_goal, selected_frontier->frontier_cost);
  }

  auto path = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, virtual_goal);
  std::vector<Eigen::Vector3d> path_points;
  if (path)
  {
    for (const auto& node_id : path->vertices)
    {
      // virtual goal 只参与图搜索, 默认不进入可执行路径
      if (node_id == virtual_goal && !append_virtual_goal_to_path_)
      {
        continue;
      }
      const auto& node = graph_.get_vertex(node_id);
      const auto& pos = node.pose.position;
      path_points.push_back(Eigen::Vector3d(pos.x, pos.y, pos.z));
    }
  }

  if (path && selected_frontier)
  {
    const bool same_branch = continues_selected_branch(*selected_frontier);
    if (!same_branch || !active_branch_)
    {
      RCLCPP_INFO(
        logger_,
        "%s, reason=%s, frontier=%s, total_cost=%.2f",
        had_active_branch && !branch_stalled ? "探索分支切换" : "探索分支选择",
        selection_reason.c_str(),
        selected_frontier->uuid.c_str(),
        selected_frontier->total_cost);
    }

    std::unordered_set<std::string> path_node_uuids;
    path_node_uuids.insert(
      selected_frontier->path_node_uuids.begin(),
      selected_frontier->path_node_uuids.end());
    std::optional<Eigen::Vector3d> branch_direction;
    if (selected_frontier->initial_direction.squaredNorm() >= 1e-12)
    {
      branch_direction = selected_frontier->initial_direction;
    }
    else if (same_branch && active_branch_)
    {
      branch_direction = active_branch_->direction;
    }
    const rclcpp::Time progress_time = same_branch && active_branch_ ?
      active_branch_->progress_time : current_time;
    const Eigen::Vector3d progress_position = same_branch && active_branch_ ?
      active_branch_->progress_position : current_position;
    active_branch_ = ActiveBranch{
      selected_frontier->position,
      selected_frontier->uuid,
      progress_time,
      progress_position,
      std::move(path_node_uuids),
      branch_direction,
      path_points,
      selected_frontier->is_deferred,
    };
  }
  else if (!goal_radius_edges.empty())
  {
    reset_frontier_branch();
  }

  graph_.remove_vertex(virtual_goal);
  return path_points;
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
