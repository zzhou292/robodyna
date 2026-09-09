#pragma once

#include "FENodalStateView.h"
#include "ExplicitStepStability.h"
#include <cuda_runtime_api.h>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace tl::fea {
constexpr std::size_t MaxTranslationNodes = 64;
constexpr std::size_t MaxTranslationDeviceBytes = 1024 * 1024;

enum class NodalStatus {
  Ok, InvalidInput, ResourceLimit, NotInitialized, WrongPhase, StaleTrial,
  ContributorFailure, InvalidOutput, UnsupportedRotation, StepTooLarge,
  HistoryLimit, DeviceFailure
};
struct NodalReport {
  NodalStatus status = NodalStatus::InvalidInput;
  const char* message = "Invalid request";  // Static, allocation-free diagnostic.
  std::uint32_t node = UINT32_MAX;
  double stable_dt = 0;
};
struct NodalStateConfig {
  std::size_t node_count = 0;
  std::size_t max_device_bytes = MaxTranslationDeviceBytes;
  double fixed_dt = 1e-3, minimum_dt = 1e-12, timestep_safety = .8;
};
struct NodalStamp {
  std::uint64_t owner_id = 0, epoch = 0;
  std::size_t node_count = 0;
  double time = 0, fixed_dt = 0;
};
struct NodalAllocationInfo {
  // Explicit module-owned cudaMalloc buffers; excludes CUDA runtime/driver
  // internal allocations and pageable-copy staging outside this module.
  std::size_t device_bytes = 0, device_allocations = 0;
};
struct NodalSnapshotBuffer {
  double* position_xyz = nullptr;
  double* velocity_xyz = nullptr;
  std::size_t capacity_nodes = 0;
};

// Sticky per-attempt failure reporting by one serialized device writer. The
// application must report EVERY failed contributor; omission cannot be inferred
// from a force vector. Provenance checks do not authenticate raw device writes.
struct NodalAssemblyResult {
  tlfea::contact::Status status = tlfea::contact::Status::kOk;
  std::uint32_t node = UINT32_MAX;
  std::uint64_t base_epoch = 0, attempt = 0;
};
struct NodalAssemblyView {
  DeviceNodalKinematicsView accepted;
  tlfea::contact::LumpedTranslationMassView mass;
  DeviceNodalForceView forces;
  stability::RowBounds* bounds = nullptr;
  NodalAssemblyResult* result = nullptr;
  cudaStream_t stream = nullptr;
  std::uint64_t attempt = 0;
  std::uint64_t owner_id = 0;  // Source association, not authentication of raw writes.
};
// Read-only completed candidate for module admission checks before commit.
// kinematics.base_epoch remains the ACCEPTED base epoch of this attempt.
// This is never an accepted snapshot or authority to publish output.
struct NodalPreparedView {
  DeviceNodalKinematicsView kinematics;
  cudaStream_t stream = nullptr;
  std::uint64_t owner_id = 0, attempt = 0;
  double proposed_time = 0;
};
TL_SURFACE_HD inline void RecordNodalAssemblyFailure(
    const NodalAssemblyView& view, tlfea::contact::Status status,
    std::uint32_t node = UINT32_MAX) {
  if (!view.result || status == tlfea::contact::Status::kOk) return;
  if (view.result->status == tlfea::contact::Status::kOk) {
    view.result->status = status;
    view.result->node = node;
  }
  stability::InvalidateRows(view.bounds);
}

class FENodalState;
class NodalTrialToken {
  // Value authorization with no lifetime/storage ownership. Retained tokens
  // cannot authorize work after destruction, even if a new owner occupies the
  // same host address. Presenting such a stale value is a supported rejection.
  friend class FENodalState;
  friend NodalReport AdvanceTranslations(FENodalState&, const NodalTrialToken&);
  std::uint64_t owner_id_ = 0, base_epoch_ = 0, attempt_ = 0;
};

// Sole owner of one shared PHYSICAL-node translation space. This is a TL state
// component, not another model/solver hierarchy or an ANCF coefficient adapter.
// Startup copies immutable masses/constraints and allocates all module buffers.
// Initial angular velocity may be absent or identically zero. Free nodes require
// positive finite inverse mass; fixed nodes require zero inverse mass/velocity.
//
// Order: BeginTrial -> all additive contributors -> SealAssembly ->
// AdvanceTranslations -> Commit. Forces and bounds are trial SCRATCH, never
// accepted force diagnostics. No external history participant is committed here.
// Calls and assembly writes are serialized; use only the returned stream. Views
// expire at SealAssembly/Discard/the next BeginTrial and cannot outlive the owner.
// No mutation through retained device views is allowed after sealing. Recoverable
// failures discard the trial. After successful initialization, any detected CUDA
// error poisons the owner; only host accepted metadata survives as a guarantee,
// not readable device memory/context recovery. Failed initialization publishes
// no owner and releases temporary buffers; device health must be established
// before retrying initialization after a runtime failure.
class FENodalState {
 public:
  FENodalState();
  ~FENodalState();
  FENodalState(const FENodalState&) = delete;
  FENodalState& operator=(const FENodalState&) = delete;
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const std::uint8_t* fixed);
  NodalReport BeginTrial(NodalTrialToken*, NodalAssemblyView*);
  NodalReport SealAssembly(const NodalTrialToken&);
  // Only after AdvanceTranslations succeeds. Validators use the returned stream
  // and finish before Commit; the coordinator must discard any rejected trial.
  // Views expire on Commit/Discard/next BeginTrial or owner destruction.
  NodalReport BorrowPrepared(const NodalTrialToken&, NodalPreparedView*);
  NodalReport Commit(const NodalTrialToken&) noexcept;
  void Discard() noexcept;
  NodalStamp accepted() const noexcept;
  NodalAllocationInfo allocations() const noexcept;
  // Output ranges must be host-writable, sized and nonoverlapping. All validation
  // and device readback finish in preallocated private staging before either
  // output range or stamp changes. Reading during a trial still exports accepted
  // x/v only. The application associates owner_id with its run/topology identity.
  NodalReport CopyAccepted(NodalSnapshotBuffer, NodalStamp*);
 private:
  friend NodalReport AdvanceTranslations(FENodalState&, const NodalTrialToken&);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tl::fea
