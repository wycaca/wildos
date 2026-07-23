#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

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
