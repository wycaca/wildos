#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

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

  const auto observing = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(6, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(observing.path_changed);
  EXPECT_TRUE(observing.path.empty());

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(14, 0, RCL_ROS_TIME),
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

TEST(DirectionalSelection, UsesReachableForwardNodeBeforeFrontierConnects)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_direct_goal_graph(0));

  const auto initial = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  ASSERT_TRUE(initial.path_changed);
  ASSERT_EQ(initial.path.size(), 3U);
  EXPECT_DOUBLE_EQ(initial.path.back().x(), 6.0);

  const auto repeated = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(repeated.path_changed);
  ASSERT_EQ(repeated.path.size(), 3U);
  EXPECT_DOUBLE_EQ(repeated.path.back().x(), 6.0);
}

TEST(DirectionalSelection, LeavesForwardFallbackAfterReachingSafeEndpoint)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_direct_goal_graph(0));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_direct_goal_graph(2));
  const auto stopped = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d(6.0, 0.0, 0.0));

  EXPECT_TRUE(stopped.path_changed);
  EXPECT_TRUE(stopped.path.empty());

  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(7, 0, RCL_ROS_TIME),
    Eigen::Vector3d(6.0, 0.0, 0.0));
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::check_dead_end);

  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(11, 0, RCL_ROS_TIME),
    Eigen::Vector3d(6.0, 0.0, 0.0));
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::exploration_exhausted);
}

}  // namespace
}  // namespace graphnav_planner
