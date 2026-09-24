// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RootIntervalFilter.h"
#include "ExactProjectionDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct RootIntervalCounters { std::size_t attempts=0,shared_vertex_bypasses=0; };
struct RootIntervalComparison {
  RepresentedIntervalStatus status=RepresentedIntervalStatus::InvalidInput;
  ExactProjectionDomainReport domain;
  RootIntervalInput input;
  RootIntervalResult filter;
  RepresentedIntervalResult original,current;
  std::size_t original_exact_cells=0,current_exact_cells=0;
  RootIntervalCounters counters;
};
// Value-only differential oracle; production has no caller-selectable filter.
RootIntervalComparison CompareRootIntervalFilter(const RepresentedTrianglePath&,
    const RepresentedTrianglePath&,RepresentedIntervalLimits) noexcept;
} // namespace tlfea::contact::represented_interval_crossing
