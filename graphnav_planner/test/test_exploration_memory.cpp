#include <gtest/gtest.h>

#include <optional>

#include "graphnav_planner/exploration_memory.hpp"

namespace graphnav_planner
{
namespace
{

TEST(ExplorationBranchMemory, KeepsMultipleDirectionsAtSameJunction)
{
  ExplorationBranchMemory memory;
  const Eigen::Vector3d junction = Eigen::Vector3d::Zero();

  memory.remember(
    "junction",
    junction,
    "east",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Vector3d::UnitX());
  memory.remember(
    "junction",
    junction,
    "north",
    Eigen::Vector3d(0.0, 3.0, 0.0),
    Eigen::Vector3d::UnitY());

  ASSERT_EQ(memory.size(), 2U);
  const auto ordered = memory.recovery_order();
  ASSERT_EQ(ordered.size(), 2U);
  EXPECT_EQ(ordered[0].node_uuid, "north");
  EXPECT_EQ(ordered[1].node_uuid, "east");

  EXPECT_TRUE(memory.erase(
    "north",
    Eigen::Vector3d(0.0, 3.0, 0.0),
    Eigen::Vector3d::UnitY(),
    0.5));
  ASSERT_EQ(memory.size(), 1U);
  EXPECT_EQ(memory.recovery_order()[0].node_uuid, "east");
}

TEST(ExplorationBranchMemory, MergesMovingFrontierInSameDirection)
{
  ExplorationBranchMemory memory;
  const Eigen::Vector3d junction = Eigen::Vector3d::Zero();

  memory.remember(
    "junction",
    junction,
    "old_frontier",
    Eigen::Vector3d(3.0, 0.0, 0.0),
    Eigen::Vector3d::UnitX());
  memory.remember(
    "junction",
    junction,
    "new_frontier",
    Eigen::Vector3d(4.0, 0.1, 0.0),
    Eigen::Vector3d(1.0, 0.05, 0.0));

  ASSERT_EQ(memory.size(), 1U);
  const auto matched = memory.find(
    "new_frontier",
    Eigen::Vector3d(4.0, 0.1, 0.0),
    std::nullopt,
    0.5);
  ASSERT_TRUE(matched.has_value());
  EXPECT_EQ(matched->node_uuid, "new_frontier");
}

TEST(ExplorationBranchMemory, RecoversNestedJunctionsInStackOrder)
{
  ExplorationBranchMemory memory;

  memory.remember(
    "outer",
    Eigen::Vector3d::Zero(),
    "outer_branch",
    Eigen::Vector3d(0.0, 3.0, 0.0),
    Eigen::Vector3d::UnitY());
  memory.remember(
    "inner",
    Eigen::Vector3d(5.0, 0.0, 0.0),
    "inner_north",
    Eigen::Vector3d(5.0, 3.0, 0.0),
    Eigen::Vector3d::UnitY());
  memory.remember(
    "inner",
    Eigen::Vector3d(5.0, 0.0, 0.0),
    "inner_south",
    Eigen::Vector3d(5.0, -3.0, 0.0),
    -Eigen::Vector3d::UnitY());

  auto ordered = memory.recovery_order();
  ASSERT_EQ(ordered.size(), 3U);
  EXPECT_EQ(ordered[0].node_uuid, "inner_south");
  EXPECT_EQ(ordered[1].node_uuid, "inner_north");
  EXPECT_EQ(ordered[2].node_uuid, "outer_branch");

  EXPECT_TRUE(memory.erase(
    "inner_south",
    Eigen::Vector3d(5.0, -3.0, 0.0),
    -Eigen::Vector3d::UnitY(),
    0.5));
  EXPECT_TRUE(memory.erase(
    "inner_north",
    Eigen::Vector3d(5.0, 3.0, 0.0),
    Eigen::Vector3d::UnitY(),
    0.5));

  ordered = memory.recovery_order();
  ASSERT_EQ(ordered.size(), 1U);
  EXPECT_EQ(ordered[0].node_uuid, "outer_branch");
}

}  // namespace
}  // namespace graphnav_planner
