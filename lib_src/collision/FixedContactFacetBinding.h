// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FixedContactFacetTypes.h"
#include "SelfContactSurfaceBinding.h"

namespace tlfea::contact {
class FixedContactFacetReadCursor;
// Immutable geometry profile, independent of rendering. Retains the complete
// native S0 authority and one small shared template; virtual vertices add no
// physical DOFs. Descriptors are values, not owner/contact admission receipts.
class FixedContactFacetBinding {
 public:
  FixedContactFacetBinding() = default;
  FixedContactFacetBinding(const FixedContactFacetBinding&) noexcept = default;
  FixedContactFacetBinding& operator=(const FixedContactFacetBinding&) = delete;
  static FixedContactFacetPreflight Preflight(const SelfContactSurfaceBinding&,
      FixedContactFacetConfig = {}, FixedContactFacetLimits = {}) noexcept;
  FixedContactFacetReport Initialize(const SelfContactSurfaceBinding&,
      FixedContactFacetConfig = {}, FixedContactFacetLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const SelfContactSurfaceBinding* surface() const noexcept;
  FixedContactFacetConfig config() const noexcept;
  FixedContactFacetForecast forecast() const noexcept;
  bool SharesStorage(const FixedContactFacetBinding&) const noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  std::size_t facet_count(std::size_t parent) const noexcept;
  FixedContactFacetReport Describe(std::size_t parent, unsigned local_facet,
      FixedContactFacet*) const noexcept;
  // Current x only. No velocity, thickness/gap modification or current-geometry
  // admission is implied. A degenerate native surface can have zero error.
  Status Approximation(std::size_t parent, VectorView positions,
      FacetApproximationBound*) const noexcept;
  // Complete parent inventory with one source/output alias preflight. This is
  // startup/current-geometry evidence, not a regularity or contact receipt.
  Status SummarizeApproximation(VectorView positions,
      FacetApproximationSummary*) const noexcept;
 private:
  FixedContactFacetReport DescribeDisjoint(std::size_t, unsigned,
      FixedContactFacet*) const noexcept;
  friend class FixedContactFacetReadCursor;
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

// Startup-only borrowed descriptor cursor. Initialize authenticates its one
// fixed output address against every retained source range once; subsequent
// Describe calls cannot redirect writes and allocate no storage.
class FixedContactFacetReadCursor {
 public:
  FixedContactFacetReport Initialize(const FixedContactFacetBinding&,
      FixedContactFacet*) noexcept;
  FixedContactFacetReport Describe(std::size_t parent,
      unsigned local_facet) const noexcept;
 private:
  const FixedContactFacetBinding* binding_ = nullptr;
  FixedContactFacet* output_ = nullptr;
};
} // namespace tlfea::contact
