// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FixedContactFacetTypes.h"
#include "SelfContactSurfaceBinding.h"
#include <optional>

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
  Status ApproximationDisjoint(std::size_t, VectorView,
      FacetApproximationBound*) const noexcept;
  friend class FixedContactFacetReadCursor;
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

struct FixedContactFacetReadResult {
  FixedContactFacetReport report;
  // Borrowed from the cursor. Null on failure and expires when Describe is
  // called again or the cursor is destroyed.
  const FixedContactFacet* facet = nullptr;
};

struct FixedContactFacetApproximationReadResult {
  Status status = Status::kInvalidArgument;
  FacetApproximationBound approximation;
};

// Authenticated descriptor/current-geometry cursor. Initialize authenticates
// its owned descriptor address once and retains the immutable source handle.
// Reads cannot redirect writes and allocate no storage.
class FixedContactFacetReadCursor {
 public:
  FixedContactFacetReadCursor() = default;
  FixedContactFacetReadCursor(const FixedContactFacetReadCursor&) = delete;
  FixedContactFacetReadCursor(FixedContactFacetReadCursor&&) = delete;
  FixedContactFacetReadCursor& operator=(
      const FixedContactFacetReadCursor&) = delete;
  FixedContactFacetReadCursor& operator=(
      FixedContactFacetReadCursor&&) = delete;
  FixedContactFacetReport Initialize(
      const FixedContactFacetBinding&) noexcept;
  FixedContactFacetReadResult Describe(std::size_t parent,
      unsigned local_facet) noexcept;
  FixedContactFacetApproximationReadResult Approximation(
      std::size_t parent, VectorView positions) noexcept;
 private:
  std::optional<FixedContactFacetBinding> binding_;
  FixedContactFacet current_;
};
} // namespace tlfea::contact
