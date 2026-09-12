#pragma once
#include "SelfContactBroadphaseTypes.h"
#include "SelfContactSurfaceBinding.h"
#include <cuda_runtime_api.h>
#include <memory>

namespace tlfea::contact {
// Bounded adapter to the existing HydroelasticBroadphase SAP traversal.
// Retains S0 maps; borrows caller coordinates and stream only until Evaluate
// returns. Every query synchronizes that stream before publishing its complete
// pair view. Null/default streams are rejected. No default-stream work, per-query allocation or host pair copy.
// All selected parents participate, including shared-node/same-body parents.
// Activity, adjacency and body exclusions belong to a later authenticated policy.
class SelfContactBroadphase {
 public:
  SelfContactBroadphase();
  ~SelfContactBroadphase();
  SelfContactBroadphase(const SelfContactBroadphase&) = delete;
  SelfContactBroadphase& operator=(const SelfContactBroadphase&) = delete;
  static SelfContactBroadphasePreflight Preflight(const SelfContactSurfaceBinding&,
      SelfContactBroadphaseLimits = {}) noexcept;
  SelfContactBroadphaseReport Initialize(const SelfContactSurfaceBinding&,
      SelfContactBroadphaseLimits, cudaStream_t) noexcept;
  SelfContactBroadphaseReport Evaluate(const SelfContactBroadphaseInput&, cudaStream_t) noexcept;
  bool initialized() const noexcept;
  const SelfContactSurfaceBinding* source() const noexcept;
  SelfContactBroadphaseForecast forecast() const noexcept;
  // A saved borrowed view expires on the next Evaluate or destruction.
  // Evaluate failure revokes the current view; raw copies cannot authenticate a query.
  SelfContactBroadphasePairs pairs() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact
