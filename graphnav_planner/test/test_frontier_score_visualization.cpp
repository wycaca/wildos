#include "planner_test_utils.hpp"

#include <cmath>

namespace graphnav_planner
{
namespace
{

using namespace test;

TEST(FrontierScoreVisualization, DrawsDirectionalRingWithPaperColors)
{
  auto graph = make_single_forward_graph(true);
  graphnav_msgs::msg::KeyValue scores;
  scores.key = "frontier_scores";
  scores.value = {0.0F, 0.5F, 1.0F, 0.25F};
  graph->nodes[1].properties.push_back(scores);

  Planner planner = make_planner();
  planner.update_graph(graph);
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  const auto markers = planner.get_score_visualization(
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    "odom");

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(
    markers.markers[0].action,
    visualization_msgs::msg::Marker::DELETEALL);

  const auto& ring = markers.markers[1];
  EXPECT_EQ(ring.type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(ring.ns, "frontier_score_rings");
  ASSERT_EQ(ring.points.size(), 40U);
  ASSERT_EQ(ring.colors.size(), ring.points.size());

  // Low and high scores use the paper-style blue and red endpoints
  EXPECT_FLOAT_EQ(ring.colors[0].r, 0.0F);
  EXPECT_FLOAT_EQ(ring.colors[0].b, 1.0F);
  EXPECT_FLOAT_EQ(ring.colors[20].r, 1.0F);
  EXPECT_FLOAT_EQ(ring.colors[20].g, 0.15F);
  EXPECT_FLOAT_EQ(ring.colors[20].b, 0.0F);

  // Bin zero is centered on positive X, matching Planner heading selection
  const double expected_offset = std::sqrt(0.5);
  EXPECT_NEAR(ring.points[0].x, 5.0 + expected_offset, 1e-6);
  EXPECT_NEAR(ring.points[0].y, -expected_offset, 1e-6);
  EXPECT_NEAR(ring.points[9].x, 5.0 + expected_offset, 1e-6);
  EXPECT_NEAR(ring.points[9].y, expected_offset, 1e-6);
}

TEST(FrontierScoreVisualization, ClearsRingWhenScoresAreUnavailable)
{
  Planner planner = make_planner();
  planner.update_graph(make_single_forward_graph(true));
  Eigen::Vector3d goal(30.0, 0.0, 0.0);
  planner.plan_to_goal(
    goal,
    3.0,
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    Eigen::Vector3d::Zero());

  const auto markers = planner.get_score_visualization(
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    "odom");

  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(
    markers.markers[0].action,
    visualization_msgs::msg::Marker::DELETEALL);
}

}  // namespace
}  // namespace graphnav_planner
