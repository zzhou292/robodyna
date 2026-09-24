// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {

// Private qualification diagnostics, never physical proof work or authority.
// Both traversals are fixed compile-time instantiations. Runtime callers cannot
// choose a proof order, and no observer, state, force, receipt or clock is changed.
struct SharedVertexProofCounters {
  std::size_t cells = 0;
  std::size_t endpoint_classifications = 0;
  std::size_t cone_calls = 0;
  std::size_t cone_axes = 0;
  std::size_t vertex_face_tasks = 0;
  std::size_t nonincident_edge_tasks = 0;
  std::size_t incident_edge_tasks = 0;
  std::size_t affine_searches = 0;
  std::size_t affine_directions = 0;
  bool saturated = false;
};

struct SharedVertexProofOrderResult {
  NonlinearSeparationResult report;
  SharedVertexProofCounters counters;
};

struct SharedVertexProofOrderComparison {
  SharedVertexProofOrderResult polynomial_first;
  SharedVertexProofOrderResult cone_first;
};

SharedVertexProofOrderComparison CompareSharedVertexTopologyOrders(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&,
    double duration, std::size_t max_work, unsigned max_depth) noexcept;

SharedVertexProofOrderComparison CompareSharedVertexCoverageOrders(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double first_thickness,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double second_thickness,
    double duration, const AcceptedEventCertificate*, std::size_t accepted_count,
    std::size_t max_work, unsigned max_depth) noexcept;

struct AffineConeSearchComparison {
  SharedVertexProofOrderResult original, current;
};
// Source/value qualification only, preserving the original 20-axis search as
// an independent disabled-extension reference. No runtime profile is exposed.
AffineConeSearchComparison CompareAffineConeSearch(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&,
    double duration, std::size_t max_work, unsigned max_depth) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
