#pragma once

#include <cstdint>
#include <optional>

#include <object_search_msgs/msg/object_search_status.hpp>

namespace graphnav_planner
{

struct ObjectSearchMode
{
  bool target_override = false;
  bool directional_exploration = false;
  bool observation = false;
};

inline std::optional<ObjectSearchMode> object_search_mode(uint8_t state)
{
  using Status = object_search_msgs::msg::ObjectSearchStatus;
  switch (state)
  {
    case Status::WAIT_FOR_ODOM: return ObjectSearchMode{};
    case Status::STARTUP_OBSERVATION: return ObjectSearchMode{false, false, true};
    case Status::SEARCHING_WITH_INITIAL_GOAL: return ObjectSearchMode{false, true, false};
    case Status::TARGET_PENDING_OBSERVATION: return ObjectSearchMode{true, false, true};
    case Status::TARGET_PENDING_REPOSITION: return ObjectSearchMode{true, false, false};
    case Status::TARGET_APPROACH_COARSE: return ObjectSearchMode{true, false, false};
    case Status::TARGET_OBSERVATION: return ObjectSearchMode{true, false, true};
    case Status::TARGET_APPROACH_METRIC: return ObjectSearchMode{true, false, false};
    case Status::TARGET_FINAL_OBSERVATION: return ObjectSearchMode{true, false, true};
    case Status::TARGET_FINAL_REPOSITION: return ObjectSearchMode{true, false, false};
    case Status::TARGET_REACHED_VIEWPOINT: return ObjectSearchMode{true, false, false};
    default: return std::nullopt;
  }
}

}  // namespace graphnav_planner
