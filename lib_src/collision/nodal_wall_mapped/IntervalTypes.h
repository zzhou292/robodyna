// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObserverTypes.h"

#if defined(__CUDACC__)
#define TL_WALL_INTERVAL_HD __host__ __device__
#else
#define TL_WALL_INTERVAL_HD
#endif
namespace tlfea::contact::nodal_wall_mapped {
struct IntervalEndpoints {
  double lower, upper;
  TL_WALL_INTERVAL_HD Q4IntegralInterval Get() const noexcept { return {lower, upper}; }
  TL_WALL_INTERVAL_HD void Set(Q4IntegralInterval value) noexcept {
    lower = value.lower;
    upper = value.upper;
  }
};
struct IntervalSummary {
  IntervalEndpoints kick, drift, moment_y, moment_z;
  double kick_work, drift_work;
  double addition_kick, addition_drift, force_uncertainty, quadratic;
  double maximum_term;
  std::uint8_t nonzero_moments;
  bool serial;
};
static_assert(std::is_trivial<IntervalSummary>::value, "Typed shared interval summary");
static_assert(sizeof(IntervalSummary) == 128, "Forecast complete interval summary");
struct IntervalScratch {
  IntervalSummary* data = nullptr;
  std::size_t count = 0;
};
} // namespace tlfea::contact::nodal_wall_mapped
#undef TL_WALL_INTERVAL_HD
