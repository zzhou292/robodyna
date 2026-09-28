// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/search/Types.h"
#include <memory>
// CUDA defines cudaStream_t as this opaque pointer. Forecast consumers need no
// runtime headers; GPU callers pass their ordinary explicit cudaStream_t.
struct CUstream_st;
namespace tlfea::contact::radioss_type25::search {
class Maintenance;
class ReferenceToken {
 public:
  ReferenceToken() = default;
  std::uint64_t generation() const noexcept { return generation_; }
 private:
  friend class Maintenance;
  std::uint64_t owner_ = 0, sequence_ = 0, generation_ = 0;
};
// Owns numerical reference snapshots only. No solver clock, candidate list,
// contact-history row or permission to reuse a physical contact inventory.
// Operations on one instance require external serialization.
class Maintenance {
 public:
  Maintenance();
  ~Maintenance();
  Maintenance(const Maintenance&) = delete;
  Maintenance& operator=(const Maintenance&) = delete;
  static Status Preflight(const Source&, Limits, Forecast&) noexcept;
  Status Initialize(const Source&, Limits, CUstream_st*) noexcept;
  // Failure invalidates pending staging while preserving the published reference.
  // Current velocities are required only for Evaluate, not reference capture.
  Status StageReference(const Current&, ReferenceToken&) noexcept;
  // Explicit reference replacement after source-authorized role removal.
  // Requires Source::MonotoneRetirement and a complete current main-node mask.
  // Published evaluation still rejects changed masks; reactivation and changed
  // source/topology stamps reject. Native empty extrema are supported only in
  // the explicit retirement profile, without granting candidate reuse authority.
  Status StageReference(const Current&, ReferenceCapturePolicy, ReferenceToken&) noexcept;
  Status PublishReference(const ReferenceToken&) noexcept;
  void DiscardReference() noexcept;
  // previous_dt follows input_units; snapshots/scalar results always use native units.
  // Legacy empty active sides reject. The explicit retirement profile follows
  // native empty extrema; success remains numerical maintenance, not reuse authority.
  Status Evaluate(const Current&, double previous_dt, bool force_sort, Report&) noexcept;
  // Last Stage/Evaluate/Publish result, copied independently of success output.
  // A device-reduced row is a flattened role ordinal, then a main-gap ordinal.
  // Discard preserves this diagnostic; the next operation replaces it.
  FailureInfo last_failure() const noexcept;
  std::uint64_t reference_generation() const noexcept;
  Forecast allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25::search
