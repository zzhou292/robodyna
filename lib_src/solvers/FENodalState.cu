#include "FENodalStateStorage.h"
#include "NodalRotation.h"
#include "NodalTrialIdentity.h"
#include "NodalRigidGroupStorage.h"
#include "NodalForceStageCaptureLayout.h"
#include "NodalStateLayout.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <utility>

namespace tl::fea {
namespace {
using nodal_detail::Phase;
using nodal_detail::Control;
namespace sc = tlfea::contact;
NodalReport Ok() { return {NodalStatus::Ok, "OK"}; }
std::atomic<std::uint64_t> next_owner{1};
std::uint64_t NewOwner() {
  auto value = next_owner.load(std::memory_order_relaxed);
  while (value != UINT64_MAX) {
    if (next_owner.compare_exchange_weak(value, value + 1, std::memory_order_relaxed)) return value;
  }
  return 0;
}
__global__ void ResetTrial(Control* c, std::uint64_t epoch, std::uint64_t attempt) {
  c->assembly = {};
  c->assembly.base_epoch = epoch; c->assembly.attempt = attempt;
  c->limit = {}; c->node = UINT32_MAX; c->status = NodalStatus::Ok;
  if (stability::ResetRows(&c->rows, epoch, attempt) != sc::Status::kOk)
    c->status = NodalStatus::InvalidOutput;
}
__global__ void SealTrial(Control* c, const double* scratch, std::uint32_t n,
                          std::uint64_t epoch, std::uint64_t attempt,
                          double safety, double minimum_dt, double h, bool rotations) {
  if (c->status != NodalStatus::Ok) return;
  if (c->assembly.base_epoch != epoch || c->assembly.attempt != attempt ||
      c->rows.base_epoch != epoch || c->rows.attempt != attempt) {
    c->status = NodalStatus::StaleTrial; return;
  }
  if (c->assembly.status != sc::Status::kOk) {
    c->status = NodalStatus::ContributorFailure; c->node = c->assembly.node; return;
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    for (unsigned axis = 0; axis < 6; ++axis) {
      const double value = scratch[axis*n+i];
      if (!sc::IsFinite(value)) { c->status = NodalStatus::InvalidOutput; c->node = i; return; }
      if (!rotations && axis >= 3 && value != 0) { c->status = NodalStatus::UnsupportedRotation; c->node = i; return; }
    }
  }
  const auto status = stability::FinalizeRows(&c->rows, safety, minimum_dt, h, &c->limit);
  if (status != sc::Status::kOk) {
    c->status = status == sc::Status::kOutOfRange ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput;
    return;
  }
  if (!stability::IsCurrentLimit(c->rows, c->limit)) { c->status = NodalStatus::StaleTrial; return; }
  if (c->limit.dt < h) {
    c->status = NodalStatus::StepTooLarge;
    c->node = c->limit.stiffness_bound > 0 ? c->limit.stiffness_node : c->limit.damping_node;
  }
}
}  // namespace

FENodalState::FENodalState() = default;
FENodalState::~FENodalState() = default;
FENodalState::Impl::~Impl() {
  if (stream) cudaStreamSynchronize(stream);
  if (control) cudaFree(control);
  if (fixed) cudaFree(fixed);
  if (inverse) cudaFree(inverse);
  if (scratch) cudaFree(scratch);
  if (trial) cudaFree(trial);
  if (accepted) cudaFree(accepted);
  if (stream) cudaStreamDestroy(stream);
}
NodalReport FENodalState::Impl::Reject(NodalStatus status, const char* message, std::uint32_t node) {
  phase = Phase::Idle;
  const double bound = status == NodalStatus::StepTooLarge &&
      host_control.limit.base_epoch == stamp.epoch && host_control.limit.attempt == attempt ? host_control.limit.dt : 0;
  return {status, message, node, bound};
}
NodalReport FENodalState::Impl::Check(cudaError_t error) {
  if (error == cudaSuccess) return Ok();
  usable = false; return Reject(NodalStatus::DeviceFailure, cudaGetErrorString(error));
}
NodalReport FENodalState::Impl::SynchronizeControl() {
  auto report = Check(cudaMemcpyAsync(&host_control, control, sizeof(Control), cudaMemcpyDeviceToHost, stream));
  if (report.status != NodalStatus::Ok) return report;
  report = Check(cudaStreamSynchronize(stream));
  if (report.status != NodalStatus::Ok) return report;
  if (host_control.status != NodalStatus::Ok)
    return Reject(host_control.status, "Trial validation failed", host_control.node);
  return Ok();
}
bool FENodalState::Impl::Matches(std::uint64_t owner, std::uint64_t epoch, std::uint64_t trial_id) const {
  return owner == stamp.owner_id && epoch == stamp.epoch && trial_id == attempt && trial_id != 0;
}

NodalReport FENodalState::Initialize(const NodalStateConfig& c, HostNodalKinematicsView in,
                                    const double* inverse_mass, const std::uint8_t* fixed) {
  return InitializeImpl(c, in, inverse_mass, fixed, nullptr);
}

NodalReport FENodalState::Initialize(const NodalStateConfig& c, HostNodalKinematicsView in,
                                    const double* inverse_mass, const NodalDofConfig& dofs) {
  return InitializeImpl(c, in, inverse_mass, nullptr, &dofs);
}

NodalReport FENodalState::Initialize(const NodalStateConfig& c, HostNodalKinematicsView in,
    const double* inverse_mass, const NodalDofConfig& dofs, const NodalRigidGroupModel& groups) {
  return InitializeImpl(c,in,inverse_mass,nullptr,&dofs,&groups);
}

NodalReport FENodalState::InitializeImpl(const NodalStateConfig& c, HostNodalKinematicsView in,
                                        const double* inverse_mass, const std::uint8_t* fixed,
                                        const NodalDofConfig* dofs, const NodalRigidGroupModel* groups) {
  if (impl_) return {NodalStatus::InvalidInput, "Owner already initialized"};
  if (!c.max_nodes || c.max_nodes > MaxActiveNodalStateNodes ||
      !c.node_count || c.node_count > c.max_nodes || !c.max_device_bytes ||
      c.max_device_bytes > MaxActiveNodalStateDeviceBytes)
    return {NodalStatus::ResourceLimit, "Nodal capacity exceeds admitted limits"};
  const bool rotations = dofs != nullptr;
  const bool staggered = c.temporal_scheme == NodalTemporalScheme::StaggeredHalfKickStart;
  if (c.temporal_scheme != NodalTemporalScheme::VelocityFirst && !staggered)
    return {NodalStatus::UnsupportedTemporalScheme, "Unknown nodal temporal scheme"};
  if (staggered && !rotations)
    return {NodalStatus::UnsupportedTemporalScheme, "Staggered stepping requires extended nodal initialization"};
  if(c.capture_force_stage_accelerations&&(!staggered||!rotations||!groups))
    return {NodalStatus::UnsupportedTemporalScheme,"Force-stage capture requires fresh staggered rigid-group startup"};
  if (in.node_count != c.node_count || !in.position_xyz || !in.velocity_xyz || !inverse_mass ||
      (rotations ? (!in.orientation_wxyz || !dofs->translation_fixed_bits || !dofs->rotation_fixed || !dofs->inverse_inertia) : !fixed) ||
      !std::isfinite(c.fixed_dt) || !std::isfinite(c.minimum_dt) || c.minimum_dt <= 0 ||
      c.fixed_dt < c.minimum_dt || !std::isfinite(c.timestep_safety) || c.timestep_safety <= 0 || c.timestep_safety >= 1)
    return {NodalStatus::InvalidInput, "Invalid kinematics, mass, or fixed-step configuration"};
  if (staggered && (!(.5*c.fixed_dt > 0) || !(.5*c.fixed_dt < c.fixed_dt)))
    return {NodalStatus::InvalidInput, "Initial half step is not representable"};
  if (!rotations && in.orientation_wxyz)
    return {NodalStatus::UnsupportedRotation, "Orientations require extended nodal initialization"};
  bool component_constraints = false;
  for (std::size_t i = 0; i < c.node_count; ++i) {
    const unsigned bits = rotations ? dofs->translation_fixed_bits[i] : (fixed[i] ? 7 : 0);
    if ((rotations ? bits > 7 : fixed[i] > 1) || !std::isfinite(inverse_mass[i]) ||
        (bits == 7 ? inverse_mass[i] != 0 : inverse_mass[i] <= 0))
      return {NodalStatus::InvalidInput, "Invalid explicit mass or fixed mask", static_cast<std::uint32_t>(i)};
    component_constraints |= bits != 0 && bits != 7;
    if (rotations && (dofs->rotation_fixed[i] > 1 || !std::isfinite(dofs->inverse_inertia[i]) ||
        (dofs->rotation_fixed[i] ? dofs->inverse_inertia[i] != 0 : dofs->inverse_inertia[i] <= 0) ||
        !nodal_detail::UnitQuaternion(nodal_detail::ReadQuaternion(in.orientation_wxyz + 4*i))))
      return {NodalStatus::InvalidInput, "Invalid isotropic inertia, rotation mask, or unit quaternion", static_cast<std::uint32_t>(i)};
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto j = 3*i+axis;
      if (!std::isfinite(in.position_xyz[j]) || !std::isfinite(in.velocity_xyz[j]) || ((bits & (1u << axis)) && in.velocity_xyz[j] != 0))
        return {NodalStatus::InvalidInput, "Invalid position or fixed-node velocity", static_cast<std::uint32_t>(i)};
      if (!rotations && in.angular_velocity_xyz && in.angular_velocity_xyz[j] != 0)
        return {NodalStatus::UnsupportedRotation, "Angular motion is not admitted", static_cast<std::uint32_t>(i)};
      if (rotations && in.angular_velocity_xyz && (!std::isfinite(in.angular_velocity_xyz[j]) ||
          (dofs->rotation_fixed[i] && in.angular_velocity_xyz[j] != 0)))
        return {NodalStatus::InvalidInput, "Invalid angular or fixed-rotation velocity", static_cast<std::uint32_t>(i)};
    }
  }
  const auto n = c.node_count;
  std::unique_ptr<nodal_detail::RigidStorage> rigid_groups;
  if(groups) {
    if(!rotations) return {NodalStatus::UnsupportedRotation,"Rigid groups require extended nodal state"};
    try {
      auto report=nodal_detail::PrepareRigidStorage(*groups,c,in,inverse_mass,*dofs,rigid_groups);
      if(report.status!=NodalStatus::Ok) return report;
    } catch(const std::bad_alloc&) { return {NodalStatus::ResourceLimit,"Rigid host storage allocation failed"}; }
  }
  const std::size_t group_values=rigid_groups?rigid::GroupStateValues*rigid_groups->info.group_count:0;
  const nodal_detail::ForceStageCaptureLayout capture{n,rigid_groups?rigid_groups->info.group_count:0};
  const std::size_t capture_values=c.capture_force_stage_accelerations?capture.values():0;
  nodal_detail::StateLayout layout;
  if(!layout.Initialize(n,rotations,group_values,rigid_groups?rigid_groups->immutable_bytes:0,
      capture_values,sizeof(Control),c.max_device_bytes))
    return {NodalStatus::ResourceLimit,"Device byte budget or host staging extent is insufficient"};
  const auto state_values=layout.accepted.count,mask_bytes=layout.fixed.bytes;
  try {
    auto next = std::make_unique<Impl>();
    next->rigid_groups=std::move(rigid_groups);
    next->staging.resize(state_values,0.);
    next->constraint_staging.resize(mask_bytes,0);
    next->config = c; next->stamp = {NewOwner(), 0, n, 0, c.fixed_dt};
    if(next->rigid_groups) next->stamp.rigid_groups=next->rigid_groups->info;
    next->stamp.temporal_scheme = c.temporal_scheme;
    next->has_rotations = rotations; next->has_component_constraints = component_constraints;
    next->stamp.has_rotations = rotations; next->state_values = state_values;
    if (!next->stamp.owner_id) return {NodalStatus::HistoryLimit, "Owner identities exhausted"};
    auto report = next->Check(cudaStreamCreateWithFlags(&next->stream, cudaStreamNonBlocking));
    if (report.status != NodalStatus::Ok) return report;
    auto allocate = [&](auto** p, std::size_t size) {
      auto r = next->Check(cudaMalloc(reinterpret_cast<void**>(p), size));
      if (r.status == NodalStatus::Ok) { next->allocation.device_bytes += size; ++next->allocation.device_allocations; }
      return r;
    };
    report = allocate(&next->accepted, layout.accepted.bytes); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->trial, layout.trial.bytes); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->scratch, layout.scratch.bytes); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->inverse, layout.inverse.bytes); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->fixed, mask_bytes); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->control, sizeof(Control)); if (report.status != NodalStatus::Ok) return report;
    if(next->rigid_groups) {
      report=next->Check(next->rigid_groups->Upload(next->stream));
      if(report.status!=NodalStatus::Ok) return report;
      next->allocation.device_bytes+=next->rigid_groups->immutable_bytes;
      ++next->allocation.device_allocations;
    }
    std::memcpy(next->staging.data(), in.position_xyz, 3*n*sizeof(double));
    std::memcpy(next->staging.data()+3*n, in.velocity_xyz, 3*n*sizeof(double));
    if (rotations) {
      if (in.angular_velocity_xyz) std::memcpy(next->staging.data()+6*n, in.angular_velocity_xyz, 3*n*sizeof(double));
      std::memcpy(next->staging.data()+9*n, in.orientation_wxyz, 4*n*sizeof(double));
    }
    if(next->rigid_groups) next->rigid_groups->InitializeState(next->staging.data()+19*n);
    report = next->Check(cudaMemcpyAsync(next->accepted, next->staging.data(), state_values*sizeof(double), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaMemcpyAsync(next->inverse, inverse_mass, n*sizeof(double), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    if (rotations) {
      report = next->Check(cudaMemcpyAsync(next->inverse+n, dofs->inverse_inertia, n*sizeof(double), cudaMemcpyHostToDevice, next->stream));
      if (report.status != NodalStatus::Ok) return report;
    }
    for (std::size_t i = 0; i < n; ++i) {
      next->constraint_staging[i] = rotations ? (dofs->translation_fixed_bits[i] == 7) : fixed[i];
      if (rotations) {
        next->constraint_staging[n+i] = dofs->translation_fixed_bits[i];
        next->constraint_staging[2*n+i] = dofs->rotation_fixed[i];
      }
    }
    report = next->Check(cudaMemcpyAsync(next->fixed, next->constraint_staging.data(), mask_bytes, cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaMemsetAsync(next->scratch, 0, layout.scratch.bytes, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    next->host_control.rows = {next->scratch+6*n, next->scratch+7*n, static_cast<std::uint32_t>(n), static_cast<std::uint32_t>(n)};
    report = next->Check(cudaMemcpyAsync(next->control, &next->host_control, sizeof(Control), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaStreamSynchronize(next->stream));
    if (report.status != NodalStatus::Ok) return report;
    impl_ = std::move(next); return Ok();
  } catch (const std::bad_alloc&) { return {NodalStatus::ResourceLimit, "Host state allocation failed"}; }
}

NodalAssemblyView FENodalState::Impl::AcceptedAssemblySources() const noexcept {
  const auto n = config.node_count;
  NodalAssemblyView view;
  view.accepted = {accepted, accepted+3*n, has_rotations ? accepted+6*n : scratch+8*n, n, stamp.epoch,
                   has_rotations ? accepted+9*n : nullptr};
  view.mass = {inverse, fixed, static_cast<std::uint32_t>(n), stamp.epoch,
               has_component_constraints || rigid_groups ? sc::TranslationMassModel::kUnspecified : sc::TranslationMassModel::kIsotropicLumped};
  view.stream = stream; view.owner_id = stamp.owner_id;
  view.temporal_scheme = stamp.temporal_scheme; view.velocity_phase = stamp.velocity_phase;
  view.position_time = stamp.time; view.velocity_time = stamp.velocity_time;
  view.rigid_groups=stamp.rigid_groups;
  if (has_rotations) {
    view.inverse_inertia = inverse+n; view.translation_fixed_bits = fixed+n; view.rotation_fixed = fixed+2*n;
  }
  return view;
}
NodalReport FENodalState::ValidateAcceptedAssemblySources(const NodalAssemblyView& retained) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!trial_identity::SameAssemblySources(retained, s.AcceptedAssemblySources()))
    return {NodalStatus::StaleTrial, "Assembly source identity differs from the actual accepted owner"};
  return Ok();
}

NodalReport FENodalState::BeginTrial(NodalTrialToken* token, NodalAssemblyView* view) {
  if (token) *token = {}; if (view) *view = {};
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_; s.phase = Phase::Idle; s.pending_qualification = 0;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!token || !view) return {NodalStatus::InvalidInput, "Missing trial output"};
  if (s.attempt == UINT64_MAX || s.stamp.epoch == UINT64_MAX)
    return {NodalStatus::HistoryLimit, "Step provenance exhausted"};
  ++s.attempt;
  s.candidate_time = s.stamp.time + s.config.fixed_dt;
  if (!std::isfinite(s.candidate_time) || s.candidate_time <= s.stamp.time)
    return {NodalStatus::HistoryLimit, "Accepted clock cannot represent another step"};
  const bool staggered = s.config.temporal_scheme == NodalTemporalScheme::StaggeredHalfKickStart;
  s.candidate_velocity_time = staggered ? s.stamp.time+.5*s.config.fixed_dt : s.candidate_time;
  s.candidate_kick_dt = staggered && s.stamp.epoch == 0 ? .5*s.config.fixed_dt : s.config.fixed_dt;
  if (staggered && (!std::isfinite(s.candidate_velocity_time) ||
      !(s.candidate_velocity_time > s.stamp.time) || !(s.candidate_velocity_time < s.candidate_time)))
    return {NodalStatus::HistoryLimit, "Accepted clock cannot represent the next midpoint"};
  const auto n = s.config.node_count;
  auto report = s.Check(cudaMemcpyAsync(s.trial, s.accepted, s.state_values*sizeof(double), cudaMemcpyDeviceToDevice, s.stream));
  if (report.status != NodalStatus::Ok) return report;
  report = s.Check(cudaMemsetAsync(s.scratch, 0, 11*n*sizeof(double), s.stream));
  if (report.status != NodalStatus::Ok) return report;
  ResetTrial<<<1,1,0,s.stream>>>(s.control, s.stamp.epoch, s.attempt);
  report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  token->owner_id_ = s.stamp.owner_id; token->base_epoch_ = s.stamp.epoch; token->attempt_ = s.attempt;
  *view = s.AcceptedAssemblySources();
  view->forces = {s.scratch, s.scratch+n, s.scratch+2*n, s.scratch+3*n, s.scratch+4*n, s.scratch+5*n, n, s.stamp.epoch};
  view->bounds = &s.control->rows; view->result = &s.control->assembly;
  view->attempt = s.attempt;
  s.phase = Phase::Assembling; return Ok();
}

NodalReport FENodalState::SealAssembly(const NodalTrialToken& token) {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return s.Reject(NodalStatus::StaleTrial, "Trial token belongs to another owner or attempt");
  if (s.phase != Phase::Assembling) return s.Reject(NodalStatus::WrongPhase, "Assembly is not open");
  auto report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  SealTrial<<<1,1,0,s.stream>>>(s.control, s.scratch, static_cast<std::uint32_t>(s.config.node_count),
      s.stamp.epoch, s.attempt, s.config.timestep_safety, s.config.minimum_dt, s.config.fixed_dt, s.has_rotations);
  report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  s.phase = Phase::Sealed; return Ok();
}
NodalReport FENodalState::BorrowPrepared(const NodalTrialToken& token, NodalPreparedView* out) {
  if (out) *out = {};
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!out) return s.Reject(NodalStatus::InvalidInput, "Missing prepared view output");
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return s.Reject(NodalStatus::StaleTrial, "Prepared token belongs to another owner or attempt");
  if (s.phase != Phase::Ready && s.phase != Phase::AwaitingValidation)
    return s.Reject(NodalStatus::WrongPhase, "No completed valid advance");
  const auto n = s.config.node_count;
  out->kinematics = {s.trial, s.trial+3*n, s.has_rotations ? s.trial+6*n : s.scratch+8*n, n, s.stamp.epoch,
                     s.has_rotations ? s.trial+9*n : nullptr};
  out->stream = s.stream; out->owner_id = s.stamp.owner_id;
  out->attempt = s.attempt; out->proposed_time = s.candidate_time;
  out->temporal_scheme = s.stamp.temporal_scheme;
  out->velocity_phase = s.config.temporal_scheme == NodalTemporalScheme::StaggeredHalfKickStart ?
      NodalVelocityPhase::PreviousMidpoint : NodalVelocityPhase::Collocated;
  out->base_velocity_phase = s.stamp.velocity_phase;
  out->base_time = s.stamp.time; out->base_velocity_time = s.stamp.velocity_time;
  out->velocity_time = s.candidate_velocity_time; out->kick_dt = s.candidate_kick_dt;
  out->rigid_groups=s.stamp.rigid_groups;
  out->base_kinematics = {s.accepted, s.accepted+3*n,
                          s.has_rotations ? s.accepted+6*n : s.scratch+8*n,
                          n, s.stamp.epoch, s.has_rotations ? s.accepted+9*n : nullptr};
  return Ok();
}

NodalReport FENodalState::Commit(const NodalTrialToken& token) noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return s.Reject(NodalStatus::StaleTrial, "Trial token belongs to another owner or attempt");
  if (s.phase == Phase::AwaitingValidation)
    return s.Reject(NodalStatus::MissingCandidateValidation, "Restricted candidate requires completed validation");
  if (s.phase != Phase::Ready) return s.Reject(NodalStatus::WrongPhase, "No completed valid advance");
  // A prepared-state validator may have queued work after the advance. Detect
  // its CUDA failure before publishing ANY reaction metadata or accepted slab.
  // Numerical rejections still require the coordinator to discard explicitly.
  auto report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.Check(cudaStreamSynchronize(s.stream)); if (report.status != NodalStatus::Ok) return report;
  if (s.has_rotations) {
    s.stamp.reactions_valid = true; s.stamp.reaction_base_epoch = s.stamp.epoch; s.stamp.reaction_time = s.stamp.time;
    s.stamp.reaction_kick_dt = s.candidate_kick_dt;
  }
  std::swap(s.accepted, s.trial); ++s.stamp.epoch; s.stamp.time = s.candidate_time;
  s.stamp.velocity_time = s.candidate_velocity_time;
  s.stamp.velocity_phase = s.config.temporal_scheme == NodalTemporalScheme::StaggeredHalfKickStart ?
      NodalVelocityPhase::PreviousMidpoint : NodalVelocityPhase::Collocated;
  s.phase = Phase::Idle; return Ok();
}
void FENodalState::Discard() noexcept { if (impl_) impl_->phase = Phase::Idle; }
NodalStamp FENodalState::accepted() const noexcept { return impl_ ? impl_->stamp : NodalStamp{}; }
NodalAllocationInfo FENodalState::allocations() const noexcept { return impl_ ? impl_->allocation : NodalAllocationInfo{}; }

}  // namespace tl::fea
