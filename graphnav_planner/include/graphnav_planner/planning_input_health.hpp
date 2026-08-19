#pragma once

#include <cmath>

namespace graphnav_planner
{

enum class PlanningInputAction
{
  plan,
  publish_hold,
  stop,
};

struct PlanningInputHealth
{
  double graph_age;
  double odom_age;
  bool graph_fresh;
  bool odom_fresh;
  bool graph_stale;

  bool healthy() const
  {
    return graph_fresh && odom_fresh;
  }

  PlanningInputAction action(bool hold_already_published) const
  {
    if (healthy())
    {
      return PlanningInputAction::plan;
    }
    if (graph_stale && odom_fresh && !hold_already_published)
    {
      return PlanningInputAction::publish_hold;
    }
    return PlanningInputAction::stop;
  }
};

inline PlanningInputHealth evaluate_planning_input_health(
  double graph_age,
  double odom_age,
  double max_graph_age,
  double max_odom_age,
  double future_tolerance = 0.1)
{
  const auto fresh = [future_tolerance](double age, double maximum_age) {
      return std::isfinite(age) && age >= -future_tolerance && age <= maximum_age;
    };
  return {
    graph_age,
    odom_age,
    fresh(graph_age, max_graph_age),
    fresh(odom_age, max_odom_age),
    std::isfinite(graph_age) && graph_age > max_graph_age,
  };
}

}  // namespace graphnav_planner
