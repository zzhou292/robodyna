// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactCurrentRegularityTypes.h"

namespace tlfea::contact {

// Pure represented-triangle check used by the binding query and independent
// controls.  It certifies finite nondegeneracy, applies the fixed numerical
// quality boundary, and classifies chart orientation with an exact dyadic
// predicate.  Every failure preserves output.
SelfContactCurrentFacetStatus EvaluateCurrentFacetRegularity(
    const Vec3 (&vertices)[3], Vec3 chart_direction,
    SelfContactCurrentFacetWitness* output) noexcept;

}  // namespace tlfea::contact
