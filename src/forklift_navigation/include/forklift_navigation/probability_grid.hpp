// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <memory>
#include "karto_sdk/Karto.h"
#include "forklift_navigation/occupancy_probability.hpp"

namespace forklift_navigation
{
// Use Karto's ray tracing and corrected scans, exposing its protected counters
// through a derived grid rather than modifying the installed library.
class ProbabilityGrid : public karto::OccupancyGrid
{
public:
  using karto::OccupancyGrid::OccupancyGrid;

  static std::unique_ptr<ProbabilityGrid> build(
    const karto::LocalizedRangeScanVector & scans, double resolution,
    unsigned int min_pass_through = 2, double occupancy_threshold = 0.1)
  {
    if (scans.empty()) {return nullptr;}
    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(scans, resolution, width, height, offset);
    auto grid = std::make_unique<ProbabilityGrid>(width, height, offset, resolution);
    grid->SetMinPassThrough(min_pass_through);
    grid->SetOccupancyThreshold(occupancy_threshold);
    grid->CreateFromScans(scans);
    return grid;
  }

  int8_t probability(int x, int y)
  {
    const auto index = GridIndex(karto::Vector2<kt_int32s>(x, y));
    const auto hits = GetCellHitsCounts()->GetDataPointer()[index];
    const auto passes = GetCellPassCounts()->GetDataPointer()[index];
    // Karto TraceLine counts the endpoint once, then RayTrace increments both
    // pass and hit counters there. Remove that extra pass to count each beam
    // once; otherwise even a pure wall would asymptote to probability 0.5.
    return occupancyProbability(hits, passes - hits);
  }
};
}  // namespace forklift_navigation
