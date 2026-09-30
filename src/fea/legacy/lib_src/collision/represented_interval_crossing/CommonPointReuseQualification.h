// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "ExactProjectionDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct CommonPointReuseCounters {
  std::size_t endpoint_queries = 0;
  std::size_t point_comparisons = 0;
  std::size_t static_sat_bypasses = 0;
  bool saturated = false;
};
struct CommonPointReuseResult {
  RepresentedIntervalResult result;
  CommonPointReuseCounters counters;
};
struct CommonPointReuseComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  ExactProjectionDomainReport domain;
  CommonPointReuseResult original, current;
};
// Private source-authenticated differential adapter. No physical receipt or
// caller-selected execution mode; both executors share the same exact body.
CommonPointReuseComparison CompareCommonPointReuse(const RepresentedTrianglePath&,
    const RepresentedTrianglePath&, RepresentedIntervalLimits) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing
