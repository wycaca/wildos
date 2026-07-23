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

}  // namespace
}  // namespace graphnav_planner
