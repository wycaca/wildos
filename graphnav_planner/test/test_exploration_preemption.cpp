#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

TEST(ExplorationPreemption, RestoresDirectionalBranchAfterTargetOverride)
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

  EXPECT_TRUE(planner.suspend_exploration_state());
  EXPECT_FALSE(planner.has_directional_exploration());
  EXPECT_FALSE(planner.suspend_exploration_state());

  EXPECT_TRUE(planner.resume_exploration_state());
  EXPECT_TRUE(planner.has_directional_exploration());
  EXPECT_FALSE(planner.resume_exploration_state());

  const auto resumed = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(resumed.path_changed);
  ASSERT_FALSE(resumed.path.empty());
  EXPECT_GT(resumed.path.back().x(), 0.0);
}

TEST(ExplorationPreemption, TargetTakeoverBypassesExplorationRouteHold)
{
  Planner planner = make_planner();
  planner.frontier_route_hold_duration_ = 100.0;
  planner.update_graph(make_opposite_branch_graph(true));
  Eigen::Vector3d exploration_goal(30.0, 0.0, 0.0);
  const auto initial = planner.plan_to_goal(
    exploration_goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(initial.path_changed);
  ASSERT_FALSE(initial.path.empty());
  EXPECT_GT(initial.path.back().x(), 0.0);

  EXPECT_TRUE(planner.suspend_exploration_state());
  Eigen::Vector3d target_goal(-5.0, 0.0, 0.0);
  const auto target_path = planner.plan_to_goal(
    target_goal,
    0.75,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  EXPECT_TRUE(target_path.path_changed);
  ASSERT_FALSE(target_path.path.empty());
  EXPECT_LT(target_path.path.back().x(), 0.0);
}

}  // namespace
}  // namespace graphnav_planner
