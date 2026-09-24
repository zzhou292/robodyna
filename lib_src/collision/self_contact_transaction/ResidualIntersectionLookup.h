// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "SortedIntersections.h"
namespace tlfea::contact::self_contact_transaction {
// Reuse only the checked intersection ordering of this immutable discovery
// cohort. A mismatched/unordered view keeps the original full scan. The borrow
// grants no geometry, ownership, local exclusion or force authority.
LinearResidualSeparationResult CertifyLinearResidualSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness_m,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleIntersectionView prepared_intersections,
    const SortedIntersections& intersections) noexcept;

// Extends the exact linear residual proof by the outward Bernstein deviation
// of each authenticated quadratic facet from its endpoint chord. The linear
// strict-gap lower bound must exceed both curvature upper bounds.
LinearResidualSeparationResult CertifyQuadraticResidualSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_quadratic,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_quadratic,
    double second_half_thickness_m, double duration,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleIntersectionView prepared_intersections,
    const SortedIntersections& intersections) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
