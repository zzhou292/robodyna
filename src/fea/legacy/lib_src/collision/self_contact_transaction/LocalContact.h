// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {

// Complete local exclusion: every unmasked thickness task is separated over
// the whole interval, and shared topology is its only geometric intersection.
// Inputs must come from authenticated source facets/member trajectories;
// coincident coordinates do not establish shared identity.
NonlinearSeparationResult CertifyQuadraticLocalContact(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_coefficients,
    double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_coefficients,
    double second_thickness, double duration,
    std::size_t max_work, unsigned max_depth) noexcept;

// Geometry component only: endpoint-local premises plus bounded whole-cell
// shared-edge/shared-vertex proofs. This alone does not admit thickness tasks.
NonlinearSeparationResult CertifyQuadraticLocalTopology(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_coefficients,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_coefficients,
    double duration, std::size_t max_work, unsigned max_depth) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
