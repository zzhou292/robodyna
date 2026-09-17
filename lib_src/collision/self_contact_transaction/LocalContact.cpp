// SPDX-License-Identifier: AGPL-3.0-or-later
#include "LocalContact.h"
#include "ResidualTasks.h"

namespace tlfea::contact::self_contact_transaction {

NonlinearSeparationResult CertifyQuadraticLocalContact(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_coefficients,
    double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_coefficients,
    double second_thickness, double duration,
    std::size_t max_work, unsigned max_depth) noexcept {
  NonlinearSeparationResult result;
  if (!max_work || max_depth > 52) return result;
  FixedTriangleFeatureTaskMask mask;
  if (BuildFixedTriangleFeatureTaskMask(
          first_prepared, second_prepared, &mask) !=
      FixedTriangleDiscoveryStatus::Ok)
    return result;
  result.status = NonlinearSeparationStatus::PotentialContact;
  if (!mask.local_tasks) return result;

  // Global discovery may choose a different seam producer; use the same
  // native bounded discovery for this exact pair, with no heap allocation.
  FixedTriangleFeatureCandidate features[15];
  fixed_triangle_features::PairFeatureResult discovery;
  if (fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
          first_prepared, second_prepared, mask, features, 15,
          &discovery) != FixedTriangleDiscoveryStatus::Ok) {
    result.status = NonlinearSeparationStatus::InvalidInput;
    return result;
  }
  const auto separation = CertifyQuadraticUnmaskedSeparation(
      first_accepted, first_prepared, first_coefficients, first_thickness,
      second_accepted, second_prepared, second_coefficients, second_thickness,
      duration, {features, discovery.feature_count, true}, mask);
  if (separation.status == LinearResidualSeparationStatus::InvalidInput) {
    result.status = NonlinearSeparationStatus::InvalidInput;
    return result;
  }
  if (separation.status !=
      LinearResidualSeparationStatus::CertifiedSeparated)
    return result;
  return CertifyQuadraticLocalTopology(
      first_accepted, first_prepared, first_coefficients,
      second_accepted, second_prepared, second_coefficients,
      duration, max_work, max_depth);
}

}  // namespace tlfea::contact::self_contact_transaction
