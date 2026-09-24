// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include <cuda_runtime_api.h>
#include <memory>

namespace tlfea::contact::self_contact_filters {
// Standalone bounded numerical adapter, not yet wired into transactions.
// Calls on one owner are externally serialized; borrowed buffers must not be
// concurrently mutated. All work uses the nondefault stream supplied at Initialize. Upload and queries
// synchronize before returning; callers retain borrowed inputs until then.
// No query allocates or grows storage. Each successful query returns one result
// per input pair in original order, without compaction, exclusions or atomics.
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;
  static Preflight PreflightLimits(Limits) noexcept;
  Report Initialize(Limits, cudaStream_t) noexcept;
  Report Upload(SceneView, cudaStream_t) noexcept;
  Report Accepted(PairView, cudaStream_t) noexcept;
  Report Linear(PairView, SelfContactFacetPrismAxisLimit, cudaStream_t) noexcept;
  Forecast forecast() const noexcept;
  // Revoke scene/results without freeing capacity or recovering a CUDA error.
  // This is host-only because Upload/query always drained their borrowed inputs.
  void DiscardScene() noexcept;
  // Enclosing owners must reject aliased output before writing a receipt.
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  // Upload invocation also revokes the old scene, including rejected uploads.
  // Every Upload/query invocation revokes the prior borrowed publication,
  // including invalid-input failures. Values may remain in storage but are not
  // a current result. Device errors poison the owner; CPU fallback belongs to
  // the future compositor and must never hide a CUDA failure.
  ResultView results() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Report Evaluate(PairView, bool accepted, SelfContactFacetPrismAxisLimit,
                  cudaStream_t) noexcept;
};
}  // namespace tlfea::contact::self_contact_filters
