#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{
namespace
{

graphnav_msgs::msg::Node make_node(
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

graphnav_msgs::msg::Edge make_edge(
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_opposite_branch_graph(
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_forward_detour_graph()
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_backward_only_graph()
{
  auto graph = make_opposite_branch_graph(false);
  graph->nodes[2] = make_node(3, -5.0, 0.0, true);
  return graph;
}

graphnav_msgs::msg::NavigationGraph::SharedPtr make_frontier_owner_behind_graph()
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_extended_branch_graph()
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
  graph->current_node_idx = 0;
  return graph;
}

graphnav_msgs::msg::NavigationGraph::SharedPtr make_shared_prefix_graph(
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_invalid_forward_path_graph()
{
  auto graph = make_opposite_branch_graph(true);
  graph->edges = {make_edge(0, 2)};
  return graph;
}

graphnav_msgs::msg::NavigationGraph::SharedPtr make_migrated_frontier_graph()
{
  auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->trav_classes = {"default"};
  graph->nodes = {
    make_node(1, 0.0, 0.0, false),
    make_node(3, -5.0, 0.0, true),
    make_node(4, 6.0, 0.0, true),
  };
  graph->edges = {make_edge(0, 1), make_edge(0, 2)};
  graph->current_node_idx = 0;
  return graph;
}

graphnav_msgs::msg::NavigationGraph::SharedPtr make_backward_migration_graph()
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_failed_alias_graph()
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_deferred_handoff_graph()
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

graphnav_msgs::msg::NavigationGraph::SharedPtr make_direct_goal_graph(
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

Planner make_planner()
{
  Planner planner(rclcpp::get_logger("test_deferred_branch"));
  planner.set_trav_class("default");
  planner.frontier_continuity_radius_ = 5.0;
  planner.frontier_progress_timeout_ = 12.0;
  planner.start_directional_exploration(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::UnitX(),
    30.0);
  return planner;
}

TEST(CommittedBranch, SuppressesRepeatedPublicationForSameFrontier)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));

  const auto initial = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(initial.path_changed);
  ASSERT_FALSE(initial.path.empty());
  EXPECT_GT(initial.path.back().x(), 0.0);

  planner.update_graph(make_opposite_branch_graph(true));
  const auto repeated = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(repeated.path_changed);
  ASSERT_FALSE(repeated.path.empty());
  EXPECT_GT(repeated.path.back().x(), 0.0);
}

TEST(DirectionalSelection, UsesWholeRouteWhenFirstSegmentPointsBackward)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_forward_detour_graph());

  const auto selected = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  ASSERT_TRUE(selected.path_changed);
  ASSERT_FALSE(selected.path.empty());
  EXPECT_DOUBLE_EQ(selected.path[1].x(), -1.0);
  EXPECT_DOUBLE_EQ(selected.path.back().x(), 5.0);
}

TEST(DirectionalSelection, ConfirmsForwardBlockBeforeSelectingRearBranch)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_backward_only_graph());

  const auto pending = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(pending.path_changed);
  EXPECT_TRUE(pending.path.empty());

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(6, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(recovered.path_changed);
  ASSERT_FALSE(recovered.path.empty());
  EXPECT_LT(recovered.path.back().x(), 0.0);
}

TEST(DirectionalSelection, UsesFrontierPointsWhenOwnerNodeIsBehind)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_frontier_owner_behind_graph());

  const auto selected = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  ASSERT_TRUE(selected.path_changed);
  ASSERT_FALSE(selected.path.empty());
  EXPECT_DOUBLE_EQ(selected.path.back().x(), -0.1);
}

TEST(CommittedBranch, KeepsCachedPathWhenFrontierTemporarilyDisappears)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_opposite_branch_graph(false));
  const auto continued = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(continued.path_changed);
  ASSERT_FALSE(continued.path.empty());
  EXPECT_GT(continued.path.back().x(), 0.0);
}

TEST(CommittedBranch, PublishesOnlyOrderedTailExtension)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_extended_branch_graph());
  const auto extended = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(4.0, 0.0, 0.0));
  EXPECT_TRUE(extended.path_changed);
  ASSERT_FALSE(extended.path.empty());
  EXPECT_DOUBLE_EQ(extended.path.front().x(), 4.0);
  EXPECT_DOUBLE_EQ(extended.path.back().x(), 10.0);
}

TEST(CommittedBranch, RejectsCandidateThatOnlySharesPathPrefix)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_shared_prefix_graph(true, true));
  const auto initial = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(initial.path_changed);
  ASSERT_FALSE(initial.path.empty());
  EXPECT_GT(initial.path.back().x(), 5.0);

  planner.update_graph(make_shared_prefix_graph(false, true));
  const auto retained = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(retained.path_changed);
  ASSERT_FALSE(retained.path.empty());
  EXPECT_GT(retained.path.back().x(), 5.0);
  EXPECT_DOUBLE_EQ(retained.path.back().y(), 0.0);
}

TEST(CommittedBranch, StartGracePreventsPrematureNoProgressRecovery)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_opposite_branch_graph(false));
  EXPECT_FALSE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(0.0, 1.0, 0.0)).path_changed);
  const auto retained = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(13, 0, RCL_ROS_TIME),
    Eigen::Vector3d(-1.0, 0.0, 0.0));
  EXPECT_FALSE(retained.path_changed);
  ASSERT_FALSE(retained.path.empty());
  EXPECT_GT(retained.path.back().x(), 0.0);

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d(-1.0, 0.0, 0.0));
  EXPECT_TRUE(recovered.path_changed);
  ASSERT_FALSE(recovered.path.empty());
  EXPECT_LT(recovered.path.back().x(), 0.0);
}

TEST(CommittedBranch, ReleasesBranchWhenCommittedEdgeDisappears)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_invalid_forward_path_graph());
  const auto pending = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(pending.path_changed);
  ASSERT_FALSE(pending.path.empty());
  EXPECT_GT(pending.path.back().x(), 0.0);

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(recovered.path_changed);
  ASSERT_FALSE(recovered.path.empty());
  EXPECT_LT(recovered.path.back().x(), 0.0);
}

TEST(CommittedBranch, FailedCorridorsSuppressNearbyUuidAliases)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  const auto first_recovery = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(first_recovery.path_changed);
  ASSERT_FALSE(first_recovery.path.empty());
  EXPECT_LT(first_recovery.path.back().x(), 0.0);

  planner.update_graph(make_failed_alias_graph());
  const auto exhausted = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(42, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(exhausted.path_changed);
  EXPECT_TRUE(exhausted.path.empty());
}

TEST(CommittedBranch, DeferredHandoffRebuildsPathWithoutReturningToOldTail)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_deferred_handoff_graph());
  const auto handoff = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(22, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(handoff.path_changed);
  ASSERT_EQ(handoff.path.size(), 2U);
  EXPECT_DOUBLE_EQ(handoff.path.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(handoff.path.back().x(), -4.0);
}

TEST(CommittedBranch, AllowsForwardSpatialMigrationAfterUuidDisappears)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_migrated_frontier_graph());
  const auto migrated = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(migrated.path_changed);
  ASSERT_FALSE(migrated.path.empty());
  EXPECT_DOUBLE_EQ(migrated.path.back().x(), 6.0);
}

TEST(CommittedBranch, RejectsBackwardSpatialMigrationWithinContinuityRadius)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_backward_migration_graph());
  const auto retained = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(retained.path_changed);
  ASSERT_FALSE(retained.path.empty());
  EXPECT_DOUBLE_EQ(retained.path.back().x(), 5.0);
}

TEST(CommittedBranch, SuppressesDirectGoalPathWhenOnlyTraversedPrefixChanges)
{
  Planner planner(rclcpp::get_logger("test_direct_goal"));
  planner.set_trav_class("default");
  Eigen::Vector3d goal(6.0, 0.0, 0.0);
  planner.update_graph(make_direct_goal_graph(0));

  const auto initial = planner.plan_to_goal(
    goal,
    1.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(initial.path_changed);
  ASSERT_EQ(initial.path.size(), 3U);

  planner.update_graph(make_direct_goal_graph(1));
  const auto progressed = planner.plan_to_goal(
    goal,
    1.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d(3.0, 0.0, 0.0));
  EXPECT_FALSE(progressed.path_changed);
  ASSERT_EQ(progressed.path.size(), 2U);
  EXPECT_DOUBLE_EQ(progressed.path.back().x(), 6.0);
}

}  // namespace
}  // namespace graphnav_planner
