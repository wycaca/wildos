#include <gtest/gtest.h>

#include <memory>
#include <optional>

#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{
namespace
{

graphnav_msgs::msg::Node make_node(std::uint8_t id, double x, bool is_frontier)
{
  graphnav_msgs::msg::Node node;
  node.uuid.id.fill(0);
  node.uuid.id[0] = id;
  node.pose.position.x = x;
  node.trav_properties.resize(1);
  node.trav_properties[0].is_frontier = is_frontier;
  node.trav_properties[0].explored_radius = 1.0;
  if (is_frontier) {
    geometry_msgs::msg::Point frontier_point;
    frontier_point.x = x;
    node.trav_properties[0].frontier_points.push_back(frontier_point);
  }
  return node;
}

graphnav_msgs::msg::Edge make_edge(std::uint32_t from_idx, std::uint32_t to_idx)
{
  graphnav_msgs::msg::Edge edge;
  edge.from_idx = from_idx;
  edge.to_idx = to_idx;
  edge.traversability.resize(1);
  edge.traversability[0].traversability_cost = 5.0;
  return edge;
}

graphnav_msgs::msg::NavigationGraph::SharedPtr make_graph(bool frontiers_active)
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, false),
    make_node(2, 5.0, frontiers_active),
    make_node(3, -5.0, frontiers_active),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

Planner make_planner()
{
  Planner planner(rclcpp::get_logger("test_deferred_branch"));
  planner.set_trav_class("default");
  planner.frontier_continuity_radius_ = 10.0;
  planner.frontier_progress_timeout_ = 12.0;
  planner.start_directional_exploration(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::UnitX(),
    30.0);
  return planner;
}

TEST(DeferredBranch, KeepsRearBranchInactiveWhileForwardBranchProgresses)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_graph(true));

  const auto initial_path = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_FALSE(initial_path.empty());
  EXPECT_GT(initial_path.back().x(), 0.0);

  planner.update_graph(make_graph(false));
  const auto continued_path = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  ASSERT_FALSE(continued_path.empty());
  EXPECT_GT(continued_path.back().x(), 0.0);
}

TEST(DeferredBranch, RestoresOldestBranchAfterConfirmedStall)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_graph(true));
  const auto initial_path = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_FALSE(initial_path.empty());
  EXPECT_GT(initial_path.back().x(), 0.0);

  planner.update_graph(make_graph(false));
  const auto recovery_path = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(13, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_FALSE(recovery_path.empty());
  EXPECT_LT(recovery_path.back().x(), 0.0);
}

}  // namespace
}  // namespace graphnav_planner
