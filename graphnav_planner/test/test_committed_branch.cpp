#include "planner_test_utils.hpp"

namespace graphnav_planner
{
namespace
{

using namespace test;

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

TEST(CommittedBranch, RejectsContinuationThatExceedsNormalBacktrackLimit)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_extended_branch_graph(0));
  const auto retained = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(4.0, 0.0, 0.0));
  EXPECT_FALSE(retained.path_changed);
  ASSERT_FALSE(retained.path.empty());
  EXPECT_DOUBLE_EQ(retained.path.back().x(), 5.0);
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

  const auto observing = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d(-1.0, 0.0, 0.0));
  EXPECT_TRUE(observing.path_changed);
  EXPECT_TRUE(observing.path.empty());

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(24, 0, RCL_ROS_TIME),
    Eigen::Vector3d(-1.0, 0.0, 0.0));
  EXPECT_TRUE(recovered.path_changed);
  ASSERT_FALSE(recovered.path.empty());
  EXPECT_LT(recovered.path.back().x(), 0.0);
}

TEST(CommittedBranch, UsesDetourPathProgressInsteadOfInitialAxis)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_forward_detour_graph());
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  const auto progressed = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d(-1.0, 0.0, 0.0));
  EXPECT_FALSE(progressed.path_changed);
  ASSERT_FALSE(progressed.path.empty());
  EXPECT_GT(progressed.path.back().x(), 0.0);
}

TEST(CommittedBranch, PausesFailureTimerWithoutPlanningStaleInputs)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.pause_failure_timers(rclcpp::Time(30, 0, RCL_ROS_TIME));

  const auto recovered_input = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(31, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(recovered_input.path_changed);
  ASSERT_FALSE(recovered_input.path.empty());
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

  const auto second_pending = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_FALSE(second_pending.path_changed);
  ASSERT_FALSE(second_pending.path.empty());

  const auto observing = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(observing.path_changed);
  EXPECT_TRUE(observing.path.empty());

  const auto recovered = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(11, 0, RCL_ROS_TIME),
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

  const auto observing = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(21, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(observing.path_changed);
  EXPECT_TRUE(observing.path.empty());

  const auto first_recovery = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(29, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(first_recovery.path_changed);
  ASSERT_FALSE(first_recovery.path.empty());
  EXPECT_LT(first_recovery.path.back().x(), 0.0);

  planner.update_graph(make_failed_alias_graph());
  const auto exhausted = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(50, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());
  EXPECT_TRUE(exhausted.path_changed);
  EXPECT_TRUE(exhausted.path.empty());
}

TEST(CommittedBranch, RecoveryHandoffRebuildsPathWithoutReturningToOldTail)
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
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(29, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_recovery_handoff_graph());
  const auto handoff = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(30, 0, RCL_ROS_TIME),
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
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(migrated.path_changed);
  ASSERT_FALSE(migrated.path.empty());
  EXPECT_DOUBLE_EQ(migrated.path.back().x(), 6.0);
}

TEST(CommittedBranch, HoldsForwardMigrationDuringMinimumDuration)
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
  const auto held = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));

  EXPECT_FALSE(held.path_changed);
  ASSERT_FALSE(held.path.empty());
  EXPECT_DOUBLE_EQ(held.path.back().x(), 5.0);
  EXPECT_EQ(planner.take_exploration_diagnostics().held_updates, 1U);
}

TEST(CommittedBranch, CoalescesSmallFrontierMigrationAfterHold)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_migrated_frontier_graph(5.5));
  const auto held = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    Eigen::Vector3d(1.0, 0.0, 0.0));

  EXPECT_FALSE(held.path_changed);
  ASSERT_FALSE(held.path.empty());
  EXPECT_DOUBLE_EQ(held.path.back().x(), 5.0);
}

TEST(CommittedBranch, RejectsNewBacktrackDuringNormalContinuation)
{
  Planner planner = make_planner();
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.update_graph(make_opposite_branch_graph(true));
  ASSERT_TRUE(planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero()).path_changed);

  planner.update_graph(make_negative_extension_graph());
  const auto retained = planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    Eigen::Vector3d(5.5, 0.0, 0.0));

  EXPECT_FALSE(retained.path_changed);
  ASSERT_FALSE(retained.path.empty());
  EXPECT_DOUBLE_EQ(retained.path.back().x(), 5.0);
  EXPECT_GT(
    planner.take_exploration_diagnostics().negative_extension_rejections,
    0U);
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

}  // namespace
}  // namespace graphnav_planner
