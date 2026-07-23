#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphnav_planner
{

class ExplorationBranchMemory
{
public:
  struct BranchRecord
  {
    std::string junction_uuid;
    Eigen::Vector3d junction_position;
    std::string node_uuid;
    Eigen::Vector3d position;
    Eigen::Vector3d discovery_direction;
    int direction_sector;
    std::uint64_t discovery_order;
  };

  struct JunctionRecord
  {
    std::string node_uuid;
    Eigen::Vector3d position;
    std::vector<std::string> branch_keys;
  };

  void clear();
  void remember(
    const std::string& junction_uuid,
    const Eigen::Vector3d& junction_position,
    const std::string& node_uuid,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction);
  std::optional<BranchRecord> find(
    const std::string& node_uuid,
    const Eigen::Vector3d& position,
    const std::optional<Eigen::Vector3d>& direction,
    double merge_radius) const;
  bool erase(
    const std::string& node_uuid,
    const Eigen::Vector3d& position,
    const std::optional<Eigen::Vector3d>& direction,
    double merge_radius);
  void prune_missing_nodes(const std::unordered_set<std::string>& live_node_uuids);
  std::vector<BranchRecord> recovery_order() const;

  const std::unordered_map<std::string, BranchRecord>& untried_branches() const;
  bool empty() const;
  std::size_t size() const;

private:
  static int direction_sector(const Eigen::Vector3d& direction);
  static std::string branch_key(
    const std::string& junction_uuid,
    int sector,
    const std::string& node_uuid);
  void erase_key(const std::string& key);

  std::vector<std::string> junction_stack_;
  std::unordered_map<std::string, JunctionRecord> junctions_;
  std::unordered_map<std::string, BranchRecord> untried_branches_;
  std::uint64_t next_branch_order_ = 0;
};

}  // namespace graphnav_planner
