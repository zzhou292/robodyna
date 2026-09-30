// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "NativeStorageDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct FixedPolicyComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  NativeStorageDomainReport domain;
  RepresentedIntervalResult original, current;
  bool fixed_executed = false;
  bool arithmetic_failed = false;
  std::size_t fixed_scratch_bytes = 0;
};
// Private host qualification only. The native source/identity checks and actual
// domain select fixed8 versus unchanged wide Boost before numerical execution.
FixedPolicyComparison CompareFixedIntegerPolicy(const RepresentedTrianglePath&,
    const RepresentedTrianglePath&, RepresentedIntervalLimits) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing
