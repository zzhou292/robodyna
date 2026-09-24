// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"

namespace tlfea::contact::represented_interval_crossing {

// Private value qualification only: no publication or physical receipt. The
// two fixed compile-time traversals share unchanged exact predicates and error
// policy. Production has no caller-selectable cache mode or cached-input flag.
struct NormalReuseCounters {
  std::size_t evaluated_cells = 0;
  std::size_t normal_evaluations = 0;
  std::size_t cache_hits = 0;
  bool saturated = false;
};
struct NormalReuseResult {
  RepresentedIntervalResult result;
  NormalReuseCounters counters;
};
struct NormalReuseComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  NormalReuseResult recomputed;
  NormalReuseResult memoized;
  std::size_t worker_exact_scratch_bytes = 0;
  std::size_t normal_storage_bytes = 0;
};
NormalReuseComparison CompareNormalReuse(
    const RepresentedTrianglePath&, const RepresentedTrianglePath&,
    RepresentedIntervalLimits) noexcept;

}  // namespace tlfea::contact::represented_interval_crossing
