#include "graphnav_planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace graphnav_planner
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRingRadius = 1.0;
constexpr double kRingHeightOffset = 0.08;
constexpr double kRingLineWidth = 0.12;
constexpr int kArcSegments = 5;

std_msgs::msg::ColorRGBA score_color(double raw_score)
{
  const double score = std::clamp(
    std::isfinite(raw_score) ? raw_score : 0.0,
    0.0,
    1.0);
  std_msgs::msg::ColorRGBA color;
  color.a = 0.85;
  if (score < 0.5)
  {
    const double ratio = score / 0.5;
    color.g = 0.25 + 0.55 * ratio;
    color.b = 1.0 - 0.65 * ratio;
  }
  else
  {
    const double ratio = (score - 0.5) / 0.5;
    color.r = 1.0;
    color.g = 1.0 - 0.85 * ratio;
  }
  return color;
}

const std::vector<float>* heading_scores(const graphnav_msgs::msg::Node& node)
{
  for (const auto& property : node.properties)
  {
    if (property.key == "frontier_scores" && !property.value.empty())
    {
      return &property.value;
    }
  }
  return nullptr;
}

geometry_msgs::msg::Point ring_point(
  const graphnav_msgs::msg::Node& node,
  double angle)
{
  geometry_msgs::msg::Point point;
  point.x = node.pose.position.x + kRingRadius * std::cos(angle);
  point.y = node.pose.position.y + kRingRadius * std::sin(angle);
  point.z = node.pose.position.z + kRingHeightOffset;
  return point;
}

void append_score_arc(
  visualization_msgs::msg::Marker& marker,
  const graphnav_msgs::msg::Node& node,
  double start_angle,
  double end_angle,
  const std_msgs::msg::ColorRGBA& color)
{
  // Split each score bin into short line segments to form a smooth ring
  for (int segment = 0; segment < kArcSegments; ++segment)
  {
    const double first_ratio =
      static_cast<double>(segment) / kArcSegments;
    const double second_ratio =
      static_cast<double>(segment + 1) / kArcSegments;
    marker.points.push_back(ring_point(
      node,
      start_angle + (end_angle - start_angle) * first_ratio));
    marker.points.push_back(ring_point(
      node,
      start_angle + (end_angle - start_angle) * second_ratio));
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  }
}

}  // namespace

visualization_msgs::msg::MarkerArray Planner::get_score_visualization(
  const rclcpp::Time& stamp,
  std::string frame_id) const
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "frontier_score_rings";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker rings;
  rings.header = clear.header;
  rings.ns = "frontier_score_rings";
  rings.id = 1;
  rings.type = visualization_msgs::msg::Marker::LINE_LIST;
  rings.action = visualization_msgs::msg::Marker::ADD;
  rings.pose.orientation.w = 1.0;
  rings.scale.x = kRingLineWidth;

  for (const auto& [id, node] : frontier_score_nodes_)
  {
    static_cast<void>(id);
    const auto* scores = heading_scores(node);
    if (!scores)
    {
      continue;
    }

    const double angle_per_bin = 2.0 * kPi / scores->size();
    for (std::size_t bin = 0; bin < scores->size(); ++bin)
    {
      // Score bins are centered on 0, 2pi/N, ..., matching Planner selection
      const double center_angle = bin * angle_per_bin;
      append_score_arc(
        rings,
        node,
        center_angle - 0.5 * angle_per_bin,
        center_angle + 0.5 * angle_per_bin,
        score_color((*scores)[bin]));
    }
  }

  if (!rings.points.empty())
  {
    markers.markers.push_back(std::move(rings));
  }
  return markers;
}

}  // namespace graphnav_planner
