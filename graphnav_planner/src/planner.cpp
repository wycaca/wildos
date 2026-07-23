#include <rclcpp/rclcpp.hpp>
#include <graaflib/graph.h>
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_paths.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <tuple>

#include "graphnav_planner/planner.hpp"
#include "planner_internal.hpp"

namespace graphnav_planner
{

namespace detail
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

}  // namespace detail

using namespace detail;

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
  // Update traversal history before replacing graph indices, UUIDs remain stable across frames
  update_traversal_memory(*graph);
  graph_ = graaf::undirected_graph<graphnav_msgs::msg::Node, double>();
  unexplored_space_map_.reset();
  frontier_score_nodes_.clear();
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
  // Record only observed current-node transitions, graph refreshes alone do not mark traversed edges
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
  // Rebuild the small Frontier-distance grid from the current persistent graph
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

}  // namespace graphnav_planner
