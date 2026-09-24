// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FixedTriangleFeatureDiscovery.h"
#include "../RepresentedIntervalCrossingTypes.h"

namespace tlfea::contact { class SelfContactTransaction; }
namespace tlfea::contact::self_contact_transaction {

struct IntersectionLookupCounts {
  std::size_t rows = 0;
  bool saturated = false;
};

// Lexical sorted-order borrow, not a geometry/ownership receipt. The transaction
// constructs it after successful native discovery, uses it only in that cohort,
// and destroys it before the next Discover call can expire the publication.
// Canonical strict ordering is checked once; malformed/foreign views retain the
// raw first-match path. No caller can supply an unchecked already-sorted flag.
class SortedIntersections {
 public:
  SortedIntersections(const SortedIntersections&) = delete;
  SortedIntersections& operator=(const SortedIntersections&) = delete;
  SortedIntersections(SortedIntersections&&) = delete;
  SortedIntersections& operator=(SortedIntersections&&) = delete;

  bool matches(FixedTriangleIntersectionView) const noexcept;
  bool ordered() const noexcept { return ordered_; }
  const FixedTriangleIntersection* Find(FixedTriangleIntersectionView,
      const RepresentedIntervalPairKey&, IntersectionLookupCounts* = nullptr) const noexcept;

 private:
  friend class ::tlfea::contact::SelfContactTransaction;
  friend struct SortedIntersectionsTestAccess;
  explicit SortedIntersections(const FixedTriangleFeatureDiscovery&) noexcept;
  explicit SortedIntersections(FixedTriangleIntersectionView) noexcept;
  const FixedTriangleIntersectionView view_;
  bool ordered_ = false;
};

struct IntersectionLookupComparison {
  std::size_t original = SIZE_MAX, indexed = SIZE_MAX;
  IntersectionLookupCounts original_counts, indexed_counts;
  bool ordered = false;
};
// Value-only operation-count qualification; no mutable state or receipt.
IntersectionLookupComparison CompareIntersectionLookup(FixedTriangleIntersectionView,
    const RepresentedIntervalPairKey&, const SortedIntersections&) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
