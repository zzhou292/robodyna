// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "NativeStorageDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct NativeStorageCounters {
  std::size_t narrow_pairs = 0, wide_pairs = 0;
  bool saturated = false;
};
struct NativeStorageResult { RepresentedIntervalResult result; NativeStorageCounters counters; };
struct NativeStorageComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  NativeStorageDomainReport domain;
  std::size_t wide_scratch_bytes = 0, narrow_scratch_bytes = 0;
  NativeStorageResult original, current;
};
// Private fixed wide/adaptive comparison through the same arithmetic bodies.
// No public runtime override, resource cap or physical receipt is added.
NativeStorageComparison CompareNativeStorage(const RepresentedTrianglePath&,
    const RepresentedTrianglePath&, RepresentedIntervalLimits) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing
