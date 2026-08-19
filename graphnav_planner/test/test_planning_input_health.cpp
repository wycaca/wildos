#include <gtest/gtest.h>

#include "graphnav_planner/planning_input_health.hpp"

namespace graphnav_planner
{

TEST(PlanningInputHealth, StaleGraphPublishesSingleHoldThenStops)
{
  const auto health = evaluate_planning_input_health(2.1, 0.1, 2.0, 1.0);

  EXPECT_EQ(health.action(false), PlanningInputAction::publish_hold);
  EXPECT_EQ(health.action(true), PlanningInputAction::stop);
}

TEST(PlanningInputHealth, StaleOdomStopsWithoutHold)
{
  const auto health = evaluate_planning_input_health(0.1, 1.1, 2.0, 1.0);

  EXPECT_EQ(health.action(false), PlanningInputAction::stop);
}

TEST(PlanningInputHealth, FutureInputsFailClosed)
{
  const auto future_graph = evaluate_planning_input_health(-0.11, 0.1, 2.0, 1.0);
  const auto future_odom = evaluate_planning_input_health(0.1, -0.11, 2.0, 1.0);

  EXPECT_EQ(future_graph.action(false), PlanningInputAction::stop);
  EXPECT_EQ(future_odom.action(false), PlanningInputAction::stop);
}

TEST(PlanningInputHealth, FreshInputsResumePlanning)
{
  const auto health = evaluate_planning_input_health(0.1, 0.1, 2.0, 1.0);

  EXPECT_EQ(health.action(true), PlanningInputAction::plan);
}

}  // namespace graphnav_planner
