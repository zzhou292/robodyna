#pragma once
#include "FENodalState.h"
#include "lib_src/math/Fixed3.h"

namespace tl::fea {
struct NodalUniformMotionLimits {
  std::size_t max_nodes = MaxActiveNodalStateNodes;
  std::size_t max_device_bytes = std::size_t{32} << 20;
  std::size_t max_host_bytes = 65536;
};
struct NodalUniformMotionForecast {
  NodalReport report;
  std::size_t device_bytes = 0, host_bytes = 0;
  std::size_t blocks = 0;
};
struct NodalUniformMotionSummary {
  std::size_t nodes = 0;
  double maximum_position_error = 0, maximum_velocity_error = 0;
  double maximum_orientation_error = 0, maximum_spin = 0;
};
struct NodalUniformMotionObservation {
  // Same borrowed metadata/lifetime as FENodalState::BorrowPrepared; never a
  // publication receipt. Scalar summary remains an observation after commit.
  NodalPreparedView prepared;
  NodalUniformMotionSummary motion;
};
// Optional read-only observer. Initialization requires a fresh idle owner and
// captures its actual original positions once. The owner and all calls are
// externally serialized. No second state/clock, history or acceptance authority.
// Observation checks the COMPLETE active owner slab (including unrequested
// reactions, rigid and CIN tails), then nodal unit quaternions, then component
// differences from original X + uniform_velocity * proposed_time, identity Q
// and zero spin. It does not prescribe motion or validate physical accuracy.
// Full snapshots/archive readback remain independent and unchanged.
class NodalUniformMotionObserver {
 public:
  NodalUniformMotionObserver();
  ~NodalUniformMotionObserver();
  NodalUniformMotionObserver(const NodalUniformMotionObserver&) = delete;
  NodalUniformMotionObserver& operator=(const NodalUniformMotionObserver&) = delete;
  static NodalUniformMotionForecast Preflight(std::size_t nodes, NodalUniformMotionLimits = {}) noexcept;
  NodalReport Initialize(FENodalState&, NodalUniformMotionLimits = {});
  // No allocation per call. Wrong output/capability, numerical failure and CUDA
  // failure leave output unchanged. As with CopyPrepared, stale/numerical owner
  // rejection discards the attempt; CUDA failure poisons the owner. Invalid
  // host argument preflight leaves a valid prepared attempt usable.
  NodalReport ObservePrepared(FENodalState&, const NodalTrialToken&,
      tl::math::Vec3 uniform_velocity, NodalUniformMotionObservation*);
  NodalAllocationInfo allocations() const noexcept;
  const NodalUniformMotionForecast& forecast() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea
