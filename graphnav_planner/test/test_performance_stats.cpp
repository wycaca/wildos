#include <deque>
#include <gtest/gtest.h>
#include "graphnav_planner/performance_stats.hpp"

namespace graphnav_planner
{

TEST(PerformanceStats, EmptyWindowReturnsZeros)
{
  const auto summary = summarize_timings(std::deque<double>{});

  EXPECT_EQ(summary.count, 0U);
  EXPECT_DOUBLE_EQ(summary.average_ms, 0.0);
  EXPECT_DOUBLE_EQ(summary.p95_ms, 0.0);
  EXPECT_DOUBLE_EQ(summary.maximum_ms, 0.0);
}

TEST(PerformanceStats, ReportsAverageP95AndMaximum)
{
  std::deque<double> samples;
  for (size_t value = 1; value <= 100; value++)
  {
    samples.push_back(static_cast<double>(value));
  }

  const auto summary = summarize_timings(samples);

  EXPECT_EQ(summary.count, 100U);
  EXPECT_DOUBLE_EQ(summary.average_ms, 50.5);
  EXPECT_DOUBLE_EQ(summary.p95_ms, 95.0);
  EXPECT_DOUBLE_EQ(summary.maximum_ms, 100.0);
}

}  // namespace graphnav_planner
