// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "ExactProjectionDomain.h"

namespace tlfea::contact::represented_interval_crossing {
struct RelativeSeparationCounters {
  std::size_t eligible_cells = 0;
  std::size_t domain_fallback_cells = 0;
  std::size_t relative_aabb_separated = 0;
  std::size_t first_face_separated = 0;
  std::size_t second_face_separated = 0;
  bool saturated = false;
};
struct RelativeSeparationComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  ExactProjectionDomainReport domain;
  RepresentedIntervalResult legacy;
  RepresentedIntervalResult current;
  RelativeSeparationCounters counters;
};
// Private qualification, not a runtime profile selector or physical receipt.
// Both traversals use the same native executor; only the compile-time presence
// of the additional complete-cell certificate differs.
RelativeSeparationComparison CompareRelativeSeparation(
    const RepresentedTrianglePath&, const RepresentedTrianglePath&,
    RepresentedIntervalLimits) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing
