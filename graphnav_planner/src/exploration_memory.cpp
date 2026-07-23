#include "graphnav_planner/exploration_memory.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace graphnav_planner
{

void ExplorationBranchMemory::clear()
{
  junction_stack_.clear();
  junctions_.clear();
  untried_branches_.clear();
  next_branch_order_ = 0;
}

void ExplorationBranchMemory::remember(
  const std::string& junction_uuid,
  const Eigen::Vector3d& junction_position,
  const std::string& node_uuid,
  const Eigen::Vector3d& position,
  const Eigen::Vector3d& direction)
{
  // Keep a moving Frontier identity in its original branch record
  for (auto& [key, branch] : untried_branches_)
  {
    (void)key;
    if (branch.node_uuid == node_uuid)
    {
      branch.position = position;
      branch.discovery_direction = direction;
      return;
    }
  }

  const int sector = direction_sector(direction);
  const std::string key = branch_key(junction_uuid, sector, node_uuid);
  auto junction_it = junctions_.find(junction_uuid);
  if (junction_it == junctions_.end())
  {
    junction_stack_.push_back(junction_uuid);
    junction_it = junctions_.emplace(
      junction_uuid,
      JunctionRecord{junction_uuid, junction_position, {}}).first;
  }

  auto existing = untried_branches_.find(key);
  if (existing != untried_branches_.end())
  {
    existing->second.node_uuid = node_uuid;
    existing->second.position = position;
    existing->second.discovery_direction = direction;
    return;
  }

  untried_branches_.emplace(
    key,
    BranchRecord{
      junction_uuid,
      junction_position,
      node_uuid,
      position,
      direction,
      sector,
      next_branch_order_++,
    });
  junction_it->second.branch_keys.push_back(key);
}

std::optional<ExplorationBranchMemory::BranchRecord>
ExplorationBranchMemory::find(
  const std::string& node_uuid,
  const Eigen::Vector3d& position,
  const std::optional<Eigen::Vector3d>& direction,
  double merge_radius) const
{
  for (const auto& [key, branch] : untried_branches_)
  {
    (void)key;
    if (branch.node_uuid == node_uuid)
    {
      return branch;
    }
    Eigen::Vector3d delta = branch.position - position;
    delta.z() = 0.0;
    if (delta.norm() > merge_radius)
    {
      continue;
    }
    if (!direction || branch.discovery_direction.norm() < 1e-6 ||
        direction->dot(branch.discovery_direction.normalized()) >= 0.5)
    {
      return branch;
    }
  }
  return std::nullopt;
}

bool ExplorationBranchMemory::erase(
  const std::string& node_uuid,
  const Eigen::Vector3d& position,
  const std::optional<Eigen::Vector3d>& direction,
  double merge_radius)
{
  const auto matched = find(node_uuid, position, direction, merge_radius);
  if (!matched)
  {
    return false;
  }
  erase_key(branch_key(
    matched->junction_uuid,
    matched->direction_sector,
    matched->node_uuid));
  return true;
}

void ExplorationBranchMemory::prune_missing_nodes(
  const std::unordered_set<std::string>& live_node_uuids)
{
  std::vector<std::string> stale_keys;
  for (const auto& [key, branch] : untried_branches_)
  {
    if (live_node_uuids.find(branch.node_uuid) == live_node_uuids.end())
    {
      stale_keys.push_back(key);
    }
  }
  for (const auto& key : stale_keys)
  {
    erase_key(key);
  }
}

std::vector<ExplorationBranchMemory::BranchRecord>
ExplorationBranchMemory::recovery_order() const
{
  // Recover the newest junction first, then its newest branch
  std::vector<BranchRecord> ordered;
  for (auto junction_it = junction_stack_.rbegin();
       junction_it != junction_stack_.rend();
       ++junction_it)
  {
    const auto record_it = junctions_.find(*junction_it);
    if (record_it == junctions_.end())
    {
      continue;
    }
    std::vector<BranchRecord> branches;
    for (const auto& key : record_it->second.branch_keys)
    {
      const auto branch_it = untried_branches_.find(key);
      if (branch_it != untried_branches_.end())
      {
        branches.push_back(branch_it->second);
      }
    }
    std::sort(
      branches.begin(),
      branches.end(),
      [](const BranchRecord& lhs, const BranchRecord& rhs) {
        return lhs.discovery_order > rhs.discovery_order;
      });
    ordered.insert(ordered.end(), branches.begin(), branches.end());
  }
  return ordered;
}

const std::unordered_map<std::string, ExplorationBranchMemory::BranchRecord>&
ExplorationBranchMemory::untried_branches() const
{
  return untried_branches_;
}

bool ExplorationBranchMemory::empty() const
{
  return untried_branches_.empty();
}

std::size_t ExplorationBranchMemory::size() const
{
  return untried_branches_.size();
}

int ExplorationBranchMemory::direction_sector(const Eigen::Vector3d& direction)
{
  constexpr int sector_count = 8;
  Eigen::Vector3d planar(direction.x(), direction.y(), 0.0);
  if (planar.norm() < 1e-6)
  {
    return -1;
  }
  double angle = std::atan2(planar.y(), planar.x());
  if (angle < 0.0)
  {
    angle += 2.0 * std::numbers::pi;
  }
  const double sector_width = 2.0 * std::numbers::pi / sector_count;
  return static_cast<int>(std::floor(
    (angle + 0.5 * sector_width) / sector_width)) % sector_count;
}

std::string ExplorationBranchMemory::branch_key(
  const std::string& junction_uuid,
  int sector,
  const std::string& node_uuid)
{
  if (sector < 0)
  {
    return junction_uuid + "|node|" + node_uuid;
  }
  return junction_uuid + "|sector|" + std::to_string(sector);
}

void ExplorationBranchMemory::erase_key(const std::string& key)
{
  const auto branch_it = untried_branches_.find(key);
  if (branch_it == untried_branches_.end())
  {
    return;
  }
  const std::string junction_uuid = branch_it->second.junction_uuid;
  untried_branches_.erase(branch_it);
  const auto junction_it = junctions_.find(junction_uuid);
  if (junction_it == junctions_.end())
  {
    return;
  }
  auto& keys = junction_it->second.branch_keys;
  keys.erase(std::remove(keys.begin(), keys.end(), key), keys.end());
  if (!keys.empty())
  {
    return;
  }
  junctions_.erase(junction_it);
  junction_stack_.erase(
    std::remove(junction_stack_.begin(), junction_stack_.end(), junction_uuid),
    junction_stack_.end());
}

}  // namespace graphnav_planner
