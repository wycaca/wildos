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

Planner::Planner(rclcpp::Logger logger) : logger_(logger)
{
}

void Planner::set_trav_class(std::string trav_class)
{
  trav_class_ = trav_class;
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
      const auto history_it = traversed_edge_counts_.find(history_key);
      if (history_it != traversed_edge_counts_.end())
      {
        // 已走过边保留在图中, 仅增加重复使用代价
        // 当死路只有原路可退时 Dijkstra 仍会选择该边, 不会形成不可达
        weight *= 1.0 + revisit_cost_factor_ * static_cast<double>(history_it->second);
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
        traversed_edge_counts_[stable_edge_key(
          graph.nodes[edge.from_idx].uuid,
          graph.nodes[edge.to_idx].uuid)]++;
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
  latest_frontier_.reset();
  latest_frontier_uuid_.reset();
  latest_frontier_progress_time_.reset();
  latest_frontier_best_distance_ = std::numeric_limits<double>::max();
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

std::vector<Eigen::Vector3d> Planner::plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time)
{
  if (!unexplored_space_map_)
  {
    RCLCPP_DEBUG(logger_, "planner 暂无有效导航图, 跳过路径规划");
    return {};
  }
  unexplored_space_map_->compute_distance_from(goal.x(), goal.y());
  frontier_scores_.clear();

  struct FrontierCandidate
  {
    graaf::vertex_id_t id;
    Eigen::Vector3d position;
    std::string uuid;
    double frontier_cost;
    double total_cost;
  };

  std::vector<std::tuple<graaf::vertex_id_t, double>> goal_radius_edges;
  std::unordered_map<graaf::vertex_id_t, double> frontier_costs;

  for (const auto& [id, node] : graph_.get_vertices())
  {
    Eigen::Vector3d node_pos;
    node_pos.x() = node.pose.position.x;
    node_pos.y() = node.pose.position.y;
    node_pos.z() = node.pose.position.z;

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
      Eigen::Vector3d goal_delta = goal - node_pos;
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
    double node_goal_dist = (node_pos - goal).norm();
    if (node_goal_dist < goal_radius)
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
  std::vector<FrontierCandidate> frontier_candidates;
  for (const auto& [id, frontier_cost] : frontier_costs)
  {
    const auto path_it = base_shortest_paths.find(id);
    if (path_it == base_shortest_paths.end())
    {
      continue;
    }
    const auto& node = graph_.get_vertex(id);
    frontier_candidates.push_back(FrontierCandidate{
      id,
      Eigen::Vector3d(
        node.pose.position.x,
        node.pose.position.y,
        node.pose.position.z),
      uuid_to_string(node.uuid),
      frontier_cost,
      path_it->second.total_weight + frontier_cost,
    });
  }

  if (frontier_progress_timeout_ > 0.0 && latest_frontier_ && latest_frontier_progress_time_ &&
      (current_time - *latest_frontier_progress_time_).seconds() >= frontier_progress_timeout_)
  {
    // 当前分支持续无进展时暂时屏蔽其空间邻域
    // 屏蔽时间复用 progress timeout, 避免再引入一组 blacklist 参数
    stalled_frontier_ = latest_frontier_;
    stalled_frontier_until_ = current_time + rclcpp::Duration::from_seconds(frontier_progress_timeout_);
    reset_frontier_branch();
  }
  if (stalled_frontier_until_ &&
      (current_time - *stalled_frontier_until_).seconds() >= 0.0)
  {
    stalled_frontier_.reset();
    stalled_frontier_until_.reset();
  }

  std::optional<FrontierCandidate> selected_frontier;
  if (goal_radius_edges.empty() && !frontier_candidates.empty())
  {
    std::vector<const FrontierCandidate*> eligible_candidates;
    for (const auto& candidate : frontier_candidates)
    {
      const bool is_stalled_branch = stalled_frontier_ &&
        (candidate.position - *stalled_frontier_).norm() <= frontier_continuity_radius_;
      if (!is_stalled_branch)
      {
        eligible_candidates.push_back(&candidate);
      }
    }
    // 如果所有 frontier 都位于刚失败的分支, 允许回退到完整候选集
    // 该回退保证真实死路场景仍可继续规划, 而不是返回空路径
    if (eligible_candidates.empty())
    {
      for (const auto& candidate : frontier_candidates)
      {
        eligible_candidates.push_back(&candidate);
      }
    }

    const FrontierCandidate* global_best = *std::min_element(
      eligible_candidates.begin(),
      eligible_candidates.end(),
      [](const FrontierCandidate* lhs, const FrontierCandidate* rhs) {
        return lhs->total_cost < rhs->total_cost;
      });
    const FrontierCandidate* continuity_best = nullptr;
    if (latest_frontier_)
    {
      for (const FrontierCandidate* candidate : eligible_candidates)
      {
        const bool same_uuid = latest_frontier_uuid_ && candidate->uuid == *latest_frontier_uuid_;
        const bool same_branch =
          (candidate->position - *latest_frontier_).norm() <= frontier_continuity_radius_;
        if (!same_uuid && !same_branch)
        {
          continue;
        }
        if (continuity_best == nullptr || candidate->total_cost < continuity_best->total_cost)
        {
          continuity_best = candidate;
        }
      }
    }

    // 当前分支只要仍接近全局最优就继续保持
    // 只有新分支总代价至少优于 switch margin 才允许主动切换
    const FrontierCandidate* chosen = global_best;
    if (continuity_best != nullptr &&
        continuity_best->total_cost <= global_best->total_cost + frontier_switch_margin_)
    {
      chosen = continuity_best;
    }
    selected_frontier = *chosen;
  }

  if (goal_radius_edges.empty() && !selected_frontier)
  {
    RCLCPP_DEBUG(logger_, "没有可达 frontier 或 goal radius node, 跳过路径规划");
    return {};
  }

  graphnav_msgs::msg::Node virtual_goal_node;
  virtual_goal_node.pose.position.x = goal.x();
  virtual_goal_node.pose.position.y = goal.y();
  virtual_goal_node.pose.position.z = goal.z();
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
    const auto& current_node = graph_.get_vertex(current_node_idx_);
    const Eigen::Vector3d current_position(
      current_node.pose.position.x,
      current_node.pose.position.y,
      current_node.pose.position.z);
    const double current_distance = (selected_frontier->position - current_position).norm();
    const bool same_uuid = latest_frontier_uuid_ &&
      selected_frontier->uuid == *latest_frontier_uuid_;
    const bool same_branch = latest_frontier_ &&
      (selected_frontier->position - *latest_frontier_).norm() <= frontier_continuity_radius_;
    if (!same_uuid && !same_branch)
    {
      latest_frontier_best_distance_ = current_distance;
      latest_frontier_progress_time_ = current_time;
    }
    else if (current_distance + 0.25 < latest_frontier_best_distance_)
    {
      // 使用固定 0.25 m 进展门槛过滤 current node 小幅抖动
      latest_frontier_best_distance_ = current_distance;
      latest_frontier_progress_time_ = current_time;
    }
    latest_frontier_ = selected_frontier->position;
    latest_frontier_uuid_ = selected_frontier->uuid;
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
  if (latest_frontier_) {
    visualization_msgs::msg::Marker frontier_marker;
    frontier_marker.header.frame_id = frame_id;
    frontier_marker.header.stamp = stamp;
    frontier_marker.ns = "local_frontier";
    frontier_marker.id = 0;
    frontier_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    frontier_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_marker.pose.position.x = latest_frontier_->x();
    frontier_marker.pose.position.y = latest_frontier_->y();
    frontier_marker.pose.position.z = latest_frontier_->z();
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
  if (latest_frontier_progress_time_)
  {
    double time_diff = (stamp - *latest_frontier_progress_time_).seconds();
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = stamp;
    text_marker.ns = "local_frontier_time";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = latest_frontier_->x() + frontier_continuity_radius_;
    text_marker.pose.position.y = latest_frontier_->y() + frontier_continuity_radius_;
    text_marker.pose.position.z = latest_frontier_->z() + 1.0;  // raise text above the node
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
