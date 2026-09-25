#pragma once

#include "FENodalStateView.h"
#include "ExplicitStepStability.h"
#include "NodalForceStageSnapshot.h"
#include "NodalStateLimits.h"
#include "NodalRigidOwnerLimits.h"
#include "../constraints/NodalRigidGroupState.h"
#include <cuda_runtime_api.h>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace tl::fea {
struct NodalCinStartup;
struct NodalCinWitnessSource;
struct NodalCinAdmission;
struct NodalCinAssemblyView;
struct NodalCinSnapshotBuffer;
struct NodalCinPhysicalMainBuffer;
struct NodalCinPhysicalMainStamp;
struct NodalCinStructuralLimit;
enum class NodalStatus {
  Ok, InvalidInput, ResourceLimit, NotInitialized, WrongPhase, StaleTrial,
  ContributorFailure, InvalidOutput, UnsupportedRotation, StepTooLarge,
  HistoryLimit, DeviceFailure, MissingStepAdmission, MissingCandidateValidation,
  UnsupportedTemporalScheme
};
enum class NodalTemporalScheme { VelocityFirst, StaggeredHalfKickStart };
enum class NodalVelocityPhase { Collocated, PreviousMidpoint };
// Continuous contact interpretation authenticated by BorrowPrepared.  The
// discrete owner still advances only endpoints; None grants no interval claim.
enum class NodalRigidMemberTrajectory : std::uint8_t {
  None,
  // For u in [0,1], h=proposed_time-base_time, accepted/prepared member
  // endpoints x0/x1 and group centers c0/c1, and the prepared primary spin w:
  //
  // x(u)=c(u)+(1-u)(x0-c0)+u(x1-c1)
  //      -.5*u*(1-u)*h^2*w x (w x (x0-c0)),
  // c(u)=(1-u)c0+u*c1.
  //
  // Thus both binary64 endpoints are exact.  In exact arithmetic the ordinary
  // (>2 member) cross-product branch reduces to its constant-spin,
  // second-order drift recurrence.  Any two-member finite-velocity or
  // binary64 endpoint residual remains in the authenticated linear endpoint
  // term; it is not silently replaced by an endpoint chord.
  EndpointCorrectedSecondOrderDriftV1,
};
constexpr bool IsCollocatedNodalTiming(NodalTemporalScheme scheme,NodalVelocityPhase phase) noexcept {
  return scheme==NodalTemporalScheme::VelocityFirst && phase==NodalVelocityPhase::Collocated;
}
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
  // Both schemes initialize from physical, collocated v0/omega0. The staggered
  // scheme takes a half kick on its first accepted interval, then full kicks.
  // Separate operations require prescribed-load or case-qualified history
  // admission; neither operation commits external material history.
  NodalTemporalScheme temporal_scheme = NodalTemporalScheme::VelocityFirst;
  // Optional transient A/AR capture. Requires fresh extended staggered startup
  // with attached prepared rigid groups. Disabled preserves legacy allocations.
  bool capture_force_stage_accelerations = false;
  // Appended for aggregate-initialization compatibility. Larger owner counts
  // require explicit count AND sufficient byte limits; defaults remain 2048
  // nodes/1 MiB. All buffers remain active-sized with no per-step growth.
  std::size_t max_nodes = MaxNodalStateNodes;
  // Independent opt-in group scope; default preserves the64-group owner.
  NodalRigidOwnerLimits rigid_limits{};
};
// Optional conventional-node degrees of freedom. Masks are immutable WORLD
// constraints: bits 1/2/4 fix x/y/z translation, and rotation_fixed is 0 or 1.
// Independent free rotations require positive isotropic inverse inertia; fixed rotations
// require zero inverse inertia and initial angular velocity. This is an input
// inertia declaration, not a shell mass formula or drilling-inertia choice.
// The typed PART assembly overload separately admits exact zero M/J members
// with primary-driven, present kinematic DOFs.
struct NodalDofConfig {
  const std::uint8_t* translation_fixed_bits = nullptr;
  const std::uint8_t* rotation_fixed = nullptr;
  const double* inverse_inertia = nullptr;
  // Immutable 0/1 rotational DOF presence. Null preserves all-present legacy
  // behavior. Absent rotations require zero inverse J, zero initial spin and
  // rotation_fixed=0. They carry no reaction; an applied couple rejects the
  // trial. CIN dependents/masters and rigid members must remain present.
  const std::uint8_t* rotation_present = nullptr;
};
struct NodalStamp {
  std::uint64_t owner_id = 0, epoch = 0;
  std::size_t node_count = 0;
  double time = 0, fixed_dt = 0;
  bool has_rotations = false;
  // Constraint reactions balance forces evaluated at this previous accepted
  // state. They accompany the new endpoint; they are not endpoint force data.
  bool reactions_valid = false;
  std::uint64_t reaction_base_epoch = 0;
  double reaction_time = 0;
  NodalTemporalScheme temporal_scheme = NodalTemporalScheme::VelocityFirst;
  NodalVelocityPhase velocity_phase = NodalVelocityPhase::Collocated;
  double velocity_time = 0;
  // Momentum changes span this kick duration, which is h/2 for the first
  // staggered kick. It is distinct from the full physical interval duration h.
  double reaction_kick_dt = 0;
  NodalRigidGroupInfo rigid_groups{}; // Immutable source association, no extra state owner.
  bool has_rotation_presence = false;
};
struct NodalAllocationInfo {
  // Explicit module-owned cudaMalloc buffers; excludes CUDA runtime/driver
  // internal allocations and pageable-copy staging outside this module.
  std::size_t device_bytes = 0, device_allocations = 0;
};
// Read-only combined-owner capacity query. No raw input arrays are read, no
// owner identity is created, and no kinematic/coefficient/DOF admission occurs.
struct NodalAssemblyCinForecast {
  NodalReport report;
  std::size_t device_bytes = 0;           // Exact explicit allocation sum.
  std::size_t source_host_bytes = 0;      // Retained CIN model/domain/classification.
  std::size_t owner_host_bytes = 0;       // Incremental retained payload upper bound.
  std::size_t startup_scratch_bytes = 0;  // Temporary witness identity index.
  std::size_t startup_host_bytes = 0;     // Existing complete owner admission bound.
};
struct NodalSnapshotBuffer {
  double* position_xyz = nullptr;
  double* velocity_xyz = nullptr;
  std::size_t capacity_nodes = 0;
  // Optional extended-owner readback. Each requested range is independently
  // validated and staged. Legacy x/v-only output remains supported.
  double* orientation_wxyz = nullptr;
  double* angular_velocity_xyz = nullptr;
  double* reaction_force_xyz = nullptr;
  double* reaction_couple_xyz = nullptr;
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
  const double* inverse_inertia = nullptr;
  const std::uint8_t* translation_fixed_bits = nullptr;
  const std::uint8_t* rotation_fixed = nullptr;
  NodalTemporalScheme temporal_scheme = NodalTemporalScheme::VelocityFirst;
  NodalVelocityPhase velocity_phase = NodalVelocityPhase::Collocated;
  double position_time = 0, velocity_time = 0;
  NodalRigidGroupInfo rigid_groups{};
  const std::uint8_t* rotation_present = nullptr;
};
// Read-only completed candidate for module admission checks before commit.
// kinematics.base_epoch remains the ACCEPTED base epoch of this attempt.
// This is never an accepted snapshot or authority to publish output.
struct NodalPreparedView {
  DeviceNodalKinematicsView kinematics;
  cudaStream_t stream = nullptr;
  std::uint64_t owner_id = 0, attempt = 0;
  double proposed_time = 0;
  // Accepted base of this interval, borrowed with the same lifetime as the
  // candidate. Enables work/energy validators without duplicating nodal state.
  DeviceNodalKinematicsView base_kinematics;
  NodalTemporalScheme temporal_scheme = NodalTemporalScheme::VelocityFirst;
  NodalVelocityPhase velocity_phase = NodalVelocityPhase::Collocated;
  NodalVelocityPhase base_velocity_phase = NodalVelocityPhase::Collocated;
  double base_time = 0, velocity_time = 0, base_velocity_time = 0, kick_dt = 0;
  NodalRigidGroupInfo rigid_groups{};
  NodalRigidMemberTrajectory rigid_member_trajectory =
      NodalRigidMemberTrajectory::None;
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
class NodalRigidGroupModel;
class NodalRigidAssemblyBinding;
struct NodalStepAdmission;
struct NodalValidationReceipt;
struct NodalStaggeredPrescribedAdmission;
struct NodalStaggeredHistoryAdmission;
class NodalTrialToken {
  // Value authorization with no lifetime/storage ownership. Retained tokens
  // cannot authorize work after destruction, even if a new owner occupies the
  // same host address. Presenting such a stale value is a supported rejection.
  friend class FENodalState;
  friend NodalReport AdvanceStaggeredCin(FENodalState&, const NodalTrialToken&, const NodalCinAdmission&);
  friend NodalReport AdvanceTranslations(FENodalState&, const NodalTrialToken&);
  friend NodalReport AdvanceNodal(FENodalState&, const NodalTrialToken&, const NodalStepAdmission&);
  friend NodalReport AdvanceStaggeredPrescribed(FENodalState&, const NodalTrialToken&, const NodalStaggeredPrescribedAdmission&);
  friend NodalReport AdvanceStaggeredHistory(FENodalState&, const NodalTrialToken&, const NodalStaggeredHistoryAdmission&);
  friend NodalReport AdvanceStaggeredRigidGroups(FENodalState&, const NodalTrialToken&, const NodalStaggeredHistoryAdmission&);
  friend NodalReport CompleteNodalValidation(FENodalState&, const NodalTrialToken&, const NodalValidationReceipt&);
  std::uint64_t owner_id_ = 0, base_epoch_ = 0, attempt_ = 0;
};

// Sole owner of one shared PHYSICAL-node space. This is a TL state
// component, not another model/solver hierarchy or an ANCF coefficient adapter.
// Startup copies immutable masses/constraints and allocates all module buffers.
// The legacy overload permits absent/zero angular velocity. The optional DOF
// overload adds unit orientation, isotropic spin and world component constraints
// in the SAME accepted/trial state; it does not qualify nonlinear shell dynamics.
// Initialization inputs remain readable and stable until the call returns.
//
// Order: BeginTrial -> all additive contributors -> SealAssembly ->
// AdvanceTranslations (legacy) or separately admitted AdvanceNodal -> Commit.
// StaggeredHalfKickStart requires extended initialization and the distinct
// AdvanceStaggeredPrescribed or restricted AdvanceStaggeredHistory operation.
// Attached rigid groups instead require AdvanceStaggeredRigidGroups and its
// validation receipt; their state shares this owner's slab and publication.
// CopyAccepted exports the stored velocity
// with its explicit phase/time; it never reconstructs a collocated velocity.
// Forces and bounds are trial SCRATCH, never
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
  static NodalAssemblyCinForecast ForecastAssemblyCin(const NodalStateConfig&,
      const NodalRigidAssemblyBinding&,const NodalCinStartup&,
      bool rotation_presence = true) noexcept;
  FENodalState();
  ~FENodalState();
  FENodalState(const FENodalState&) = delete;
  FENodalState& operator=(const FENodalState&) = delete;
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const std::uint8_t* fixed);
  // Requires orientation_wxyz. All translation bits set require zero inverse
  // mass; every other node requires positive inverse mass. Fixed components
  // require zero initial velocity. Angular velocity may be absent (zero).
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const NodalDofConfig&);
  // Optional plain rigid groups: exact reference/mass/J association, free
  // member DOFs, uniform member translation and zero initial spin. Copies the
  // immutable model and appends group history to the same accepted/trial slabs.
  // Requires the dedicated staggered rigid-group advance and validation receipt.
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const NodalDofConfig&,
                         const NodalRigidGroupModel&);
  // Explicit CIN phase, with optional disjoint existing rigid groups. Complete
  // current coefficients and source witnesses are supplied by the caller.
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const NodalDofConfig&,
                         const NodalCinStartup&, const NodalRigidGroupModel* = nullptr);
  // Prepared PART and plain groups in one owner. Requires VehicleAssembly
  // limits, exact source coefficients and kinematically present member rotation.
  // Explicit empty rigid bindings require staggered startup and the physical
  // CIN coefficient store (possibly empty); raw fixed-node M/J remain proved.
  NodalReport Initialize(const NodalStateConfig&, HostNodalKinematicsView,
                         const double* inverse_mass, const NodalDofConfig&,
                         const NodalRigidAssemblyBinding&, const NodalCinStartup* = nullptr);
  NodalReport BorrowCinAssembly(const NodalTrialToken&, NodalCinAssemblyView*);
  NodalReport ValidateCinWitnessSource(const NodalCinWitnessSource&) const noexcept;
  NodalReport CopyAcceptedCin(NodalCinSnapshotBuffer, NodalStamp*);
  NodalReport CopyPreparedCin(const NodalTrialToken&, NodalCinSnapshotBuffer, NodalPreparedView*);
  // TT0 only: complete post-CIN virtual main coefficients, no physical node or clock.
  NodalReport CopyPreparedCinPhysicalMains(const NodalTrialToken&, NodalCinPhysicalMainBuffer,
                                          NodalCinPhysicalMainStamp*);
  // Host-only, no extra CUDA copy/sync: requires capture_limiter on this successful
  // prepared CIN attempt. Unchanged output on error; no commit authority.
  NodalReport CopyPreparedCinStructuralLimit(const NodalTrialToken&, NodalCinStructuralLimit*) const;
  NodalReport BeginTrial(NodalTrialToken*, NodalAssemblyView*);
  // Host-only exact authentication of the currently open assembly capability.
  // The token, owner phase/attempt, accepted sources, every force destination,
  // bounds/result control address, count/stamp, masks and stream must equal this
  // owner's private live view. No CUDA call/error consumption or phase change.
  NodalReport AuthenticateAssemblyView(
      const NodalTrialToken&, const NodalAssemblyView&) const noexcept;
  // Exact live-view authentication plus a range predicate over this owner,
  // its host staging, all nodal allocations, rigid metadata and the complete
  // CIN device arena. This compares addresses only and never dereferences the
  // supplied range.
  bool AssemblyRangeDisjoint(const NodalTrialToken&,
      const NodalAssemblyView&, const void*, std::size_t) const noexcept;
  // Host-only comparison of a retained assembly SOURCE identity with this
  // owner's current accepted buffers and immutable mass/constraint storage.
  // It may inspect pointer values after that view expires, but never reads
  // through them or restores access to the expired view. The source epoch and
  // timing must still match the current accepted state. Attempts, force
  // destinations and their contents are deliberately not validated. This
  // neither grants assembly/commit authority nor consumes pending CUDA errors.
  NodalReport ValidateAcceptedAssemblySources(const NodalAssemblyView&) const noexcept;
  // Startup-only identity check for a nondefault stream already owned by this
  // state. It grants no trial, device-pointer, assembly, or publication access.
  // The caller retains no right to use the stream after owner destruction.
  NodalReport ValidateOwnerStream(cudaStream_t) const noexcept;
  // Narrow startup accessor. The returned stream remains owner-borrowed and is
  // authenticated again by consumers before they retain it.
  NodalReport BorrowOwnerStream(cudaStream_t*) const noexcept;
  // Read-only startup predicate over this actual owner's immutable membership.
  // Count/range validation precedes member lookup. No CUDA call, allocation,
  // phase change or force/publication authority; queries may repeat indices.
  NodalReport ValidateNonRigidNodes(const std::size_t*,std::size_t count) const noexcept;
  // Read-only comparison with the complete immutable PART/plain metadata
  // retained at startup. Valid at any accepted epoch; no device read/allocation.
  // Complete non-rigid coefficients and current CIN values remain separate.
  NodalReport ValidateRigidAssemblyBinding(const NodalRigidAssemblyBinding&) const noexcept;
  // Immutable source-role query only: present rotations and unfixed world DOFs.
  // Authentic PART/CIN zero inverses are allowed; no coefficients are inferred.
  NodalReport ValidateFreeRotationalNodes(const std::size_t*,std::size_t count) const noexcept;
  // Read-only role query allowing actual fixed world components; rotations must
  // remain present. No allocation, device operation or physical authority.
  NodalReport ValidatePresentRotationalNodes(const std::size_t*,std::size_t count) const noexcept;
  // Supplied-readback predicate, requiring the caller's preceding authenticated
  // CopyAccepted at this fresh accepted stamp; not an independent device read.
  // It compares exact velocity bits against THIS owner's immutable fixed masks;
  // it does not certify arbitrary caller data as an owner readback.
  NodalReport ValidateInitialConstrainedTranslation(const NodalStamp&,const double* velocity_xyz,
      std::size_t nodes,tl::math::Vec3 common_velocity) const noexcept;
  NodalReport SealAssembly(const NodalTrialToken&);
  // Only after the applicable advance succeeds. Validators use the returned stream
  // and finish before Commit; the coordinator must discard any rejected trial.
  // Views expire on Commit/Discard/next BeginTrial or owner destruction.
  NodalReport BorrowPrepared(const NodalTrialToken&, NodalPreparedView*);
  // Drains the owner stream and checks pending CUDA errors before publication.
  // The coordinator must still report/discard numerical validator rejections.
  NodalReport Commit(const NodalTrialToken&) noexcept;
  void Discard() noexcept;
  NodalStamp accepted() const noexcept;
  NodalAllocationInfo allocations() const noexcept;
  // Output ranges must be host-writable, sized and nonoverlapping. All validation
  // and device readback finish in preallocated private staging before any
  // output range or stamp changes. Reading during a trial still exports accepted
  // state only. The application associates owner_id with its run/topology identity.
  NodalReport CopyAccepted(NodalSnapshotBuffer, NodalStamp*);
  // Candidate motion and actual constraint reaction F/C, for validation before
  // publication. Uses the same optional fields, active capacity and private
  // staging as CopyAccepted. Every output, including the prepared identity, is
  // unchanged on failure and must be disjoint from the input token. No mutable
  // device storage is exposed. Ready and AwaitingValidation phases are allowed;
  // this readback does not validate or commit the candidate. Reaction F/C belong
  // to prepared.base_time/base_kinematics.base_epoch and prepared.kick_dt; x/q
  // are at proposed_time, v/omega at velocity_time. The returned prepared view
  // has BorrowPrepared's lifetime. Calls are serialized with owner operations.
  NodalReport CopyPrepared(const NodalTrialToken&, NodalSnapshotBuffer, NodalPreparedView*);
  // Actual force-stage A/AR from the same completed prepared kick. All node
  // arrays and source-associated group rows are mandatory and publish together
  // with prepared identity, only after authentication, readback and finite checks.
  // Outputs must be disjoint from one another and token. Fixed ordinary DOFs
  // have zero A/AR. The force stage is prepared.base_time, not proposed_time;
  // combine with pre-kick motion and the SAME prepared group's updated axes.
  // Unavailable without startup opt-in, or after commit/discard/new attempt.
  // Does not return accepted data, advance state, or define an energy tolerance.
  NodalReport CopyPreparedForceStage(const NodalTrialToken&,NodalForceStageSnapshotBuffer,NodalPreparedView*);
  NodalRigidGroupInfo rigid_groups() const noexcept;
  // Failure-atomic readback, with the SAME accepted owner stamp. The frame's
  // force-stage time is stamp.reaction_time after a step, stamp.time at startup.
  NodalReport CopyAcceptedRigidGroups(NodalRigidGroupSnapshotBuffer, NodalStamp*);
  // Candidate readback for validators; buffers and prepared timing publish
  // together only after all validation/readback succeeds. Output ranges must
  // also be disjoint from the token. No mutable group view is exported.
  NodalReport CopyPreparedRigidGroups(const NodalTrialToken&,NodalRigidGroupSnapshotBuffer,NodalPreparedView*);
 private:
  friend NodalReport AdvanceStaggeredCin(FENodalState&, const NodalTrialToken&, const NodalCinAdmission&);
  friend NodalReport AdvanceTranslations(FENodalState&, const NodalTrialToken&);
  friend NodalReport AdvanceNodal(FENodalState&, const NodalTrialToken&, const NodalStepAdmission&);
  friend NodalReport AdvanceStaggeredPrescribed(FENodalState&, const NodalTrialToken&, const NodalStaggeredPrescribedAdmission&);
  friend NodalReport AdvanceStaggeredHistory(FENodalState&, const NodalTrialToken&, const NodalStaggeredHistoryAdmission&);
  friend NodalReport AdvanceStaggeredRigidGroups(FENodalState&, const NodalTrialToken&, const NodalStaggeredHistoryAdmission&);
  friend NodalReport CompleteNodalValidation(FENodalState&, const NodalTrialToken&, const NodalValidationReceipt&);
  NodalReport InitializeImpl(const NodalStateConfig&, HostNodalKinematicsView,
                             const double* inverse_mass, const std::uint8_t* fixed,
                             const NodalDofConfig*, const NodalRigidGroupModel* = nullptr,
                             const NodalCinStartup* = nullptr,const NodalRigidAssemblyBinding* = nullptr);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tl::fea
