// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "RepresentedIntervalCrossingTypes.h"

#include <memory>

namespace tlfea::contact {

// Bounded host certificate for pairs of fixed physical triangles.  For the
// explicitly declared LinearNodalV1 path, every vertex follows the exact real
// line between its represented endpoint values over t in [0,1].
//
// A crossing/contact certificate contains an exact dyadic time at which exact
// predicates prove the two closed triangles intersect.  A separation
// certificate covers the complete interval with iteratively partitioned,
// linear-path endpoint enclosures and a nondegeneracy proof.  Those enclosures
// are separation-only evidence: swept AABB overlap is never called a crossing.
//
// Unsupported motion, unresolved degeneracy and bounded-work exhaustion are
// successful, explicit Unresolved records.  Malformed identity/input and total
// capacity exhaustion fail the whole call.  Failed calls preserve the previous
// immutable complete publication; a successful call replaces it atomically.
// results() is borrowed: its view expires on the next successful Certify, move,
// or destruction.  Every failed Certify preserves the prior view's address,
// count, completeness and bytes.
class RepresentedIntervalCrossing {
 public:
  RepresentedIntervalCrossing() noexcept;
  ~RepresentedIntervalCrossing();
  RepresentedIntervalCrossing(const RepresentedIntervalCrossing&) = delete;
  RepresentedIntervalCrossing& operator=(
      const RepresentedIntervalCrossing&) = delete;
  RepresentedIntervalCrossing(RepresentedIntervalCrossing&&) noexcept;
  RepresentedIntervalCrossing& operator=(
      RepresentedIntervalCrossing&&) noexcept;

  static RepresentedIntervalPreflight Preflight(
      RepresentedIntervalLimits limits = {}) noexcept;
  RepresentedIntervalReport Initialize(
      RepresentedIntervalLimits limits = {}) noexcept;
  RepresentedIntervalReport Certify(
      const RepresentedTrianglePath* paths, std::size_t path_count,
      const RepresentedTrianglePair* pairs, std::size_t pair_count) noexcept;

  bool initialized() const noexcept { return bool(impl_); }
  RepresentedIntervalForecast forecast() const noexcept;
  RepresentedIntervalResultView results() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
