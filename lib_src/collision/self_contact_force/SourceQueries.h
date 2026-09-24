// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../self_contact_active_use/Storage.h"

namespace tlfea::contact::self_contact_force {

// Private compile-time adapters for one shared event-validation body. The
// public-query adapter retains the original independent reference path for
// qualification. Production authenticates the complete owned batch scratch
// once, then uses exactly the same active-use classification implementation.
// Neither adapter publishes a classification or descriptor to a caller.
template<bool Batched>
struct SourceQueries {
  explicit SourceQueries(const SelfContactActiveUseBinding& source)
      : binding(source) {}

  SelfContactActiveUseReport ClassifyVertexFace(
      std::size_t vertex, std::size_t facet,
      const WeightedSurfacePoint& point,
      SelfContactActivityView activity) noexcept {
    if constexpr (Batched)
      return self_contact_transaction::ActiveUseQueryAccess::ClassifyVertexFace(
          binding, vertex, facet, point, activity, &classification);
    else
      return binding.ClassifyVertexFace(
          vertex, facet, point, activity, &classification);
  }

  SelfContactActiveUseReport ClassifyEdgeEdge(
      std::size_t first, const WeightedSurfacePoint& a,
      std::size_t second, const WeightedSurfacePoint& b,
      SelfContactEdgeEdgeCase edge_case,
      SelfContactActivityView activity) noexcept {
    if constexpr (Batched)
      return self_contact_transaction::ActiveUseQueryAccess::ClassifyEdgeEdge(
          binding, first, a, second, b, edge_case, activity, &classification);
    else
      return binding.ClassifyEdgeEdge(
          first, a, second, b, edge_case, activity, &classification);
  }

  FixedContactFacetReadResult Describe(
      std::size_t parent, unsigned local) noexcept {
    if constexpr (Batched)
      return cursor.Describe(parent, local);
    else {
      const auto report = binding.facets()->Describe(parent, local, &facet);
      return {report, report.status == FixedContactFacetStatus::Ok
                          ? &facet : nullptr};
    }
  }

  const SelfContactActiveUseBinding& binding;
  SelfContactPairClassification classification;
  FixedContactFacetReadCursor cursor;
  FixedContactFacet facet;
};

}  // namespace tlfea::contact::self_contact_force
