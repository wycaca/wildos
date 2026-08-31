#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace graphnav_planner
{

struct TimingSummary
{
  size_t count = 0;
  double average_ms = 0.0;
  double p95_ms = 0.0;
  double maximum_ms = 0.0;
};

template<typename Range>
TimingSummary summarize_timings(const Range& timings)
{
  // 复制后排序, 保留原始固定窗口顺序供下一次增量记录
  if (timings.empty())
  {
    return {};
  }
  std::vector<double> samples(timings.begin(), timings.end());
  std::sort(samples.begin(), samples.end());
  const size_t p95_index = std::min(
    static_cast<size_t>(std::ceil(samples.size() * 0.95)) - 1,
    samples.size() - 1);
  return {
    samples.size(),
    std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size(),
    samples[p95_index],
    samples.back(),
  };
}

}  // namespace graphnav_planner
