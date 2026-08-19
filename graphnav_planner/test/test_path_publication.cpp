#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

TEST(CommittedBranch, SuppressesRepeatedPublicationForSameFrontier)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  auto graph = make_opposite_branch_graph(true);
  EXPECT_EQ(planner.update_graph(graph), Planner::GraphUpdate::rebuilt);

  const auto initial = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(initial.path_changed);
  ASSERT_FALSE(initial.path.empty());
  EXPECT_GT(initial.path.back().x(), 0.0);

  EXPECT_EQ(
    planner.update_graph(make_opposite_branch_graph(true)),
    Planner::GraphUpdate::unchanged);
  const auto repeated = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(repeated.path_changed);
  ASSERT_FALSE(repeated.path.empty());
  EXPECT_GT(repeated.path.back().x(), 0.0);
}

TEST(CommittedBranch, UpdatesScoresWithoutRebuildingTopology)
{
  Planner planner = make_planner();
  auto graph = make_opposite_branch_graph(true);
  ASSERT_EQ(planner.update_graph(graph), Planner::GraphUpdate::rebuilt);

  auto scored_graph = make_opposite_branch_graph(true);
  graphnav_msgs::msg::KeyValue score_property;
  score_property.key = "frontier_scores";
  score_property.value = {0.8F, 0.2F};
  scored_graph->nodes[1].properties.push_back(score_property);

  EXPECT_EQ(
    planner.update_graph(scored_graph),
    Planner::GraphUpdate::scores_only);
  EXPECT_EQ(
    planner.update_graph(scored_graph),
    Planner::GraphUpdate::unchanged);
}

}  // namespace
}  // namespace graphnav_planner
