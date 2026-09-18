// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FixedTriangleFeatureTypes.h"
#include "../RepresentedIntervalCrossingTypes.h"

namespace tlfea::contact::self_contact_transaction {

// Native intersection publications have canonical producer identities. Keep
// the same first-match lookup for publication validation and normalization.
const FixedTriangleIntersection* FindPairIntersection(
    FixedTriangleIntersectionView intersections,
    const RepresentedIntervalPairKey& pair) noexcept;

enum class TranslatedLocalStatus { NotApplicable, Certified, InvalidInput };

// Internal transaction composition, not an independent physical authority.
// The caller must already have authenticated the complete accepted force/
// feature census, current activity and regularity, and the native prepared
// discovery/edge policy for this same attempt. Only the native exact-common-
// translation tag supplies whole-interval relative-geometry invariance.
// Together these premises preserve accepted finite-thickness obligations;
// this does not weaken LocalContact's standalone unowned-task separation rule.
// A local prepared intersection is then local throughout the interval.
// Ordinary first witnesses, nonlinear paths and nonlocal intersections gain
// no authority here. Unsuccessful calls leave the result unchanged.
TranslatedLocalStatus NormalizeExactTranslatedLocal(
    FixedTriangleIntersectionView intersections,
    RepresentedIntervalResult* result) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
