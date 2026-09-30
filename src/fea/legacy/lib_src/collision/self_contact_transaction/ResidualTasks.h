// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {

// Proves only that every unmasked directed VF/EE distance remains strictly
// beyond the summed physical half-thickness throughout the represented path.
// The caller supplies exact native discovery rows for this pair; source/task
// identity, complete unique task coverage and the local mask are checked here.
// This is NOT a triangle-intersection or local-topology certificate. A caller
// must independently certify continuous geometry before suppressing a pair.
// No task is omitted on account of coordinates, body membership or proximity.
LinearResidualSeparationResult CertifyQuadraticUnmaskedSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_quadratic,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_quadratic,
    double second_half_thickness_m, double duration,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleFeatureTaskMask authenticated_mask) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
