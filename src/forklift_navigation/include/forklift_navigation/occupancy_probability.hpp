// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>

namespace forklift_navigation
{
// Beta(1,1) prior on the hit/pass ratio: unknown cells stay unknown;
// repeated free/occupied evidence approaches 0/100 without a hard threshold.
inline int8_t occupancyProbability(uint32_t hits, uint32_t passes)
{
  if (passes == 0) {return -1;}
  return static_cast<int8_t>(std::lround(
    100.0 * (static_cast<double>(hits) + 1.0) /
    (static_cast<double>(passes) + 2.0)));
}
}  // namespace forklift_navigation
