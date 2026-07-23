#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace graphnav_planner::detail
{

struct PathProgress
{
  double distance;
  size_t segment;
  double next_waypoint_distance;
};

std::vector<Eigen::Vector3d> remaining_path(
  const std::vector<Eigen::Vector3d>& path,
  const Eigen::Vector3d& current_position);

PathProgress path_progress_detail(
  const std::vector<Eigen::Vector3d>& path,
  const Eigen::Vector3d& current_position);

std::optional<Eigen::Vector3d> terminal_direction(
  const std::vector<Eigen::Vector3d>& path);

bool is_route_suffix(
  const std::vector<std::string>& route,
  const std::vector<std::string>& committed_route);

bool route_has_repeated_nodes(const std::vector<std::string>& route);

const char* selection_reason_name(const std::string& reason);

const char* release_reason_name(const std::string& reason);

}  // namespace graphnav_planner::detail
