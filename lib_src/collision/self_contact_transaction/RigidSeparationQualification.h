// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {
// Qualification only. Both sides run the same unchanged root/input checks.
// The current side may stop an impossible separation search at its actual
// root visit. No contact owner, intersection permission or trajectory is issued.
struct RigidSeparationComparison {
  NonlinearSeparationResult original, current;
};
RigidSeparationComparison CompareRigidSeparationEndpointWitness(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double first_thickness,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double second_thickness,
    double duration, std::size_t max_work, unsigned max_depth) noexcept;
}  // namespace tlfea::contact::self_contact_transaction
