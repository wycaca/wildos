#pragma once

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{
namespace test
{

inline graphnav_msgs::msg::Node make_node(
  std::uint8_t id,
  double x,
  double y,
  bool is_frontier)
{
  graphnav_msgs::msg::Node node;
  node.uuid.id.fill(0);
  node.uuid.id[0] = id;
  node.pose.position.x = x;
  node.pose.position.y = y;
  node.trav_properties.resize(1);
  node.trav_properties[0].is_frontier = is_frontier;
  node.trav_properties[0].explored_radius = 1.0;
  if (is_frontier)
  {
    geometry_msgs::msg::Point frontier_point;
    frontier_point.x = x;
    frontier_point.y = y;
    node.trav_properties[0].frontier_points.push_back(frontier_point);
  }
  return node;
}

inline graphnav_msgs::msg::Edge make_edge(
  std::uint32_t from_idx,
  std::uint32_t to_idx,
  double cost = 5.0)
{
  graphnav_msgs::msg::Edge edge;
  edge.from_idx = from_idx;
  edge.to_idx = to_idx;
  edge.traversability.resize(1);
  edge.traversability[0].traversability_cost = cost;
  return edge;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_opposite_branch_graph(
  bool frontiers_active)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, 5.0, 0.0, frontiers_active),
    make_node(3, -5.0, 0.0, frontiers_active),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_single_forward_graph(
  bool frontier_active)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, 5.0, 0.0, frontier_active),
  };
  graph->edges = {make_edge(0, 1)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_forward_detour_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, -1.0, 0.0, false),
    make_node(3, 5.0, 0.0, true),
    make_node(4, -4.0, 0.0, true),
  };
  graph->edges = {
    make_edge(0, 1, 1.0),
    make_edge(1, 2, 2.0),
    make_edge(0, 3, 0.1),
  };
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_backward_only_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, -5.0, 0.0, true),
  };
  graph->edges = {make_edge(0, 1)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_frontier_owner_behind_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  auto forward_frontier = make_node(2, -0.1, 0.0, true);
  forward_frontier.trav_properties[0].frontier_points[0].x = 3.0;
  auto rear_frontier = make_node(3, -2.0, 0.0, true);
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    forward_frontier,
    rear_frontier,
  };
  graph->edges = {
    make_edge(0, 1, 1.0),
    make_edge(0, 2, 0.1),
  };
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_extended_branch_graph(
  std::uint32_t current_node_idx = 1)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, 5.0, 0.0, false),
    make_node(3, -5.0, 0.0, true),
    make_node(4, 10.0, 0.0, true),
  };
  graph->edges = {
    make_edge(0, 1),
    make_edge(0, 2),
    make_edge(1, 3),
  };
  graph->current_node_idx = current_node_idx;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_shared_prefix_graph(
  bool forward_frontier,
  bool side_frontier)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, 2.0, 0.0, false),
    make_node(3, 6.0, 0.0, forward_frontier),
    make_node(4, 2.0, 5.0, side_frontier),
  };
  graph->edges = {
    make_edge(0, 1, 1.0),
    make_edge(1, 2, 1.0),
    make_edge(1, 3, 20.0),
  };
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_invalid_forward_path_graph()
{
  auto graph = make_opposite_branch_graph(true);
  graph->edges = {make_edge(0, 2)};
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_migrated_frontier_graph(
  double frontier_x = 6.0)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(3, -5.0, 0.0, true),
    make_node(4, frontier_x, 0.0, true),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_negative_extension_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(2, 5.0, 0.0, false),
    make_node(4, 6.0, 0.0, false),
    make_node(5, 4.0, 0.0, false),
    make_node(6, 10.0, 0.0, true),
  };
  graph->edges = {
    make_edge(0, 1, 1.0),
    make_edge(1, 2, 1.0),
    make_edge(2, 3, 1.0),
  };
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_backward_migration_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(4, 2.0, 0.0, true),
  };
  graph->edges = {make_edge(0, 1)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_failed_alias_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(4, 6.0, 0.0, true),
    make_node(3, -5.0, 0.0, true),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_recovery_handoff_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(3, -5.0, 0.0, true),
    make_node(4, -4.0, 0.0, true),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

inline graphnav_msgs::msg::NavigationGraph::SharedPtr make_direct_goal_graph(
  std::uint32_t current_node_idx)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(2, 3.0, 0.0, false),
    make_node(3, 6.0, 0.0, false),
  };
  graph->edges = {make_edge(0, 1), make_edge(1, 2)};
  graph->current_node_idx = current_node_idx;
  return graph;
}

inline Planner make_planner()
{
  Planner planner(rclcpp::get_logger("test_exploration_state"));
  planner.set_trav_class("default");
  planner.frontier_continuity_radius_ = 5.0;
  planner.frontier_progress_timeout_ = 12.0;
  planner.start_directional_exploration(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::UnitX(),
    30.0);
  return planner;
}

}  // namespace test
}  // namespace graphnav_planner
