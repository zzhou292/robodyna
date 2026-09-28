// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeContactPublicationState.h"
namespace tl::fea::native_contact_publication {
// The force/search stage consumes activity at accepted X_n. Candidate source
// activity for X_(n+1) may reuse that slab or publish exactly one new generation.
// This is selector admission only; the transaction authenticates actual masks
// and completes every device operation before requesting common publication.
inline bool ValidActivityPlan(const NativeContactSelectors& accepted,
                              const NativeContactSelectors& next) noexcept {
  if (!accepted.activity_generation)
    return !accepted.activity && !accepted.reference_activity_generation &&
        !next.activity && !next.activity_generation && !next.reference_activity_generation;
  if (accepted.activity > 1 || next.activity > 1 ||
      next.reference_activity_generation != accepted.activity_generation)
    return false;
  const bool reused = next.activity == accepted.activity &&
      next.activity_generation == accepted.activity_generation;
  const bool changed = accepted.activity_generation != UINT64_MAX &&
      next.activity == (accepted.activity ^ 1u) &&
      next.activity_generation == accepted.activity_generation + 1;
  return reused || changed;
}
} // namespace tl::fea::native_contact_publication
