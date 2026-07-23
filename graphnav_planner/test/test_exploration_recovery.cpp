#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

TEST(ExplorationState, ConfirmsDeadEndBeforeBacktracking)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::follow_branch);

  const auto checking = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(checking.path_changed);
  EXPECT_TRUE(checking.path.empty());
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::check_dead_end);

  const auto backtracking = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(24, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(backtracking.path_changed);
  EXPECT_FALSE(backtracking.path.empty());
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::backtrack);

  planner.update_graph(make_recovery_handoff_graph());
  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(25, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::follow_branch);
}

TEST(ExplorationState, EntersExhaustedWhenNoBranchRemains)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_single_forward_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  const auto exhausted = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(24, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  EXPECT_FALSE(exhausted.path_changed);
  EXPECT_TRUE(exhausted.path.empty());
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::exploration_exhausted);
}

TEST(ExplorationState, EmptyGraphDoesNotClearActiveBranch)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  auto empty_graph =
    std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  empty_graph->trav_classes = {"default"};
  planner.update_graph(empty_graph);
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::follow_branch);

  planner.update_graph(make_opposite_branch_graph(true));
  const auto resumed = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(resumed.path_changed);
  EXPECT_FALSE(resumed.path.empty());
}

TEST(ExplorationState, ExplicitResetClearsRecoveryState)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);
  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::check_dead_end);

  planner.reset_exploration_state();

  EXPECT_EQ(
    planner.exploration_state(),
    Planner::ExplorationState::follow_branch);
  EXPECT_FALSE(planner.has_directional_exploration());
}

}  // namespace
}  // namespace graphnav_planner
