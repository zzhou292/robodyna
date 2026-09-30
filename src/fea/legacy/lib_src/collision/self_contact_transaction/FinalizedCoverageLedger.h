// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

#include <algorithm>
#include <array>

namespace tlfea::contact::self_contact_transaction {

// A lexical borrow of the transaction-owned, finalized accepted ledger. Only
// SealCandidate may construct it after authenticating the same owner/assembly/
// attempt. The ledger cannot change during that synchronous seal; nothing is
// retained across publication, discard, or another assembly. Raw-array coverage
// keeps the independent full scan and does not accept a caller's sorted flag.
class FinalizedCoverageLedger {
 public:
  struct Range { std::size_t begin = 0, end = 0; };
  struct Ranges {
    // Six VF source vertices and six possible first EE edges. Keys may repeat;
    // unioning ordinal intervals removes repeated visits without merging owners.
    std::array<Range, 12> values{};
    std::size_t count = 0;
    std::size_t comparisons = 0;  // Deterministic lookup cost, not proof work.
  };

  FinalizedCoverageLedger(const FinalizedCoverageLedger&) = delete;
  FinalizedCoverageLedger& operator=(const FinalizedCoverageLedger&) = delete;
  FinalizedCoverageLedger(FinalizedCoverageLedger&&) = delete;
  FinalizedCoverageLedger& operator=(FinalizedCoverageLedger&&) = delete;

  const AcceptedEventCertificate* data() const noexcept { return data_; }
  std::size_t count() const noexcept { return count_; }

  Ranges ForPair(const CurrentFixedTriangle (&triangles)[2]) const noexcept {
    Ranges result;
    for (const auto& triangle : triangles) {
      for (unsigned local = 0; local < 3; ++local) {
        AppendPrefix(FixedTriangleCandidateKind::VertexFace,
                     &triangle.vertex_keys[local], nullptr, &result);
        AppendPrefix(FixedTriangleCandidateKind::EdgeEdge,
                     nullptr, &triangle.edge_keys[local], &result);
      }
    }
    std::sort(result.values.begin(), result.values.begin() + result.count,
              [](Range a, Range b) {
                return a.begin < b.begin ||
                    (a.begin == b.begin && a.end < b.end);
              });
    std::size_t written = 0;
    for (std::size_t i = 0; i < result.count; ++i) {
      const auto range = result.values[i];
      if (written && range.begin <= result.values[written - 1].end)
        result.values[written - 1].end =
            std::max(result.values[written - 1].end, range.end);
      else
        result.values[written++] = range;
    }
    result.count = written;
    return result;
  }

 private:
  friend class ::tlfea::contact::SelfContactTransaction;
  friend struct FinalizedCoverageLedgerTestAccess;

  FinalizedCoverageLedger(const AcceptedEventCertificate* data,
                          std::size_t count) noexcept
      : data_(data), count_(count) {}

  void AppendPrefix(FixedTriangleCandidateKind kind,
                    const FacetVertexKey* vertex, const FacetEdgeKey* edge,
                    Ranges* output) const noexcept {
    // FinalizeAcceptedEventLedger orders by full geometric feature before
    // active parent ownership. These shorter prefixes therefore form complete
    // contiguous ranges, including all distinct physical owners and seams.
    const auto compare = [&](std::size_t index) {
      ++output->comparisons;
      const auto& feature = data_[index].event.feature;
      if (feature.kind != kind)
        return static_cast<unsigned>(feature.kind) < static_cast<unsigned>(kind)
            ? -1 : 1;
      return kind == FixedTriangleCandidateKind::VertexFace
          ? fixed_triangle_features::Compare(feature.vertex_face.vertex, *vertex)
          : fixed_triangle_features::Compare(feature.edge_edge.edges[0], *edge);
    };
    std::size_t lower = 0, upper = count_;
    while (lower < upper) {
      const auto middle = lower + (upper - lower) / 2;
      if (compare(middle) < 0) lower = middle + 1;
      else upper = middle;
    }
    const auto begin = lower;
    upper = count_;
    while (lower < upper) {
      const auto middle = lower + (upper - lower) / 2;
      if (compare(middle) <= 0) lower = middle + 1;
      else upper = middle;
    }
    if (begin != lower) output->values[output->count++] = {begin, lower};
  }

  const AcceptedEventCertificate* const data_;
  const std::size_t count_;
};

// The indexed overloads differ only in which certificate ordinals are offered
// to the unchanged exact owner builder. All geometry/work/error policy is shared.
NonlinearSeparationResult CertifyQuadraticFacetCoverage(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double, double,
    const FinalizedCoverageLedger&, std::size_t, unsigned) noexcept;

NonlinearSeparationResult CertifyQuadraticFacetPolicyCoverage(
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double,
    const CurrentFixedTriangle&, const CurrentFixedTriangle&,
    const FacetQuadraticCoefficients&, double, double,
    const FinalizedCoverageLedger&,
    const AcceptedFeatureExclusionCertificate*, std::size_t,
    std::size_t, unsigned, PolicyExclusionSource* = nullptr) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
