#include "FENodalStateStorage.h"
#include "NodalRotation.h"
#include "NodalTrialIdentity.h"
#include "NodalRigidGroupStorage.h"
#include "NodalForceStageCaptureLayout.h"
#include "NodalStateLayout.h"
#include "NodalCinStorage.h"
#include "nodal_seal/Validation.cuh"
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

NodalReport FENodalState::Initialize(const NodalStateConfig& c, HostNodalKinematicsView in,
    const double* inverse_mass, const NodalDofConfig& dofs, const NodalCinStartup& cin,
    const NodalRigidGroupModel* groups) {
  return InitializeImpl(c, in, inverse_mass, nullptr, &dofs, groups, &cin);
}

NodalReport FENodalState::Initialize(const NodalStateConfig& c, HostNodalKinematicsView in,
    const double* inverse_mass, const NodalDofConfig& dofs,
    const NodalRigidAssemblyBinding& binding, const NodalCinStartup* cin) {
  return InitializeImpl(c, in, inverse_mass, nullptr, &dofs, nullptr, cin, &binding);
}

NodalReport FENodalState::InitializeImpl(const NodalStateConfig& c, HostNodalKinematicsView in,
                                        const double* inverse_mass, const std::uint8_t* fixed,
                                        const NodalDofConfig* dofs, const NodalRigidGroupModel* groups,
                                        const NodalCinStartup* cin, const NodalRigidAssemblyBinding* binding) {
  if (impl_) return {NodalStatus::InvalidInput, "Owner already initialized"};
  if (!c.max_nodes || c.max_nodes > MaxActiveNodalStateNodes ||
      !c.node_count || c.node_count > c.max_nodes || !c.max_device_bytes ||
      c.max_device_bytes > MaxActiveNodalStateDeviceBytes)
    return {NodalStatus::ResourceLimit, "Nodal capacity exceeds admitted limits"};
  const bool rotations = dofs != nullptr;
  const bool rotation_presence = rotations && dofs->rotation_present;
  const bool staggered = c.temporal_scheme == NodalTemporalScheme::StaggeredHalfKickStart;
  if (c.temporal_scheme != NodalTemporalScheme::VelocityFirst && !staggered)
    return {NodalStatus::UnsupportedTemporalScheme, "Unknown nodal temporal scheme"};
  if (staggered && !rotations)
    return {NodalStatus::UnsupportedTemporalScheme, "Staggered stepping requires extended nodal initialization"};
  if(c.capture_force_stage_accelerations&&(!staggered||!rotations||(!groups&&!binding)))
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
  // Count and whole-owner byte admission precede borrowed nodal array reads.
  // Source model access here is immutable metadata only; physical association
  // and ordered per-node validity checks below retain their existing order.
  const auto n = c.node_count;
  nodal_detail::RigidStorageLayout rigid_layout;
  if(groups) {
    const auto report=nodal_detail::ForecastRigidStorage(*groups,c,rigid_layout);
    if(report.status!=NodalStatus::Ok)return report;
  }
  if (binding) {
    const auto report = nodal_detail::ForecastRigidStorage(*binding, c, rigid_layout);
    if (report.status != NodalStatus::Ok) return report;
  }
  const auto group_count = binding ? binding->groups().size() : (groups ? groups->group_count() : 0);
  const std::size_t group_values = rigid::GroupStateValues*group_count;
  nodal_detail::CinLayout cin_layout;
  if (cin) {
    if (!rotations) return {NodalStatus::UnsupportedRotation, "CIN requires extended nodal state"};
    const auto report = nodal_detail::ForecastCinStorage(*cin, c, cin_layout);
    if (report.status != NodalStatus::Ok) return report;
  }
  const nodal_detail::ForceStageCaptureLayout capture{n,group_count};
  const std::size_t capture_values=c.capture_force_stage_accelerations?capture.values():0;
  nodal_detail::StateLayout layout;
  if(!layout.Initialize(n,rotations,group_values,rigid_layout.device_bytes,
      capture_values,sizeof(Control),c.max_device_bytes,cin_layout.state_values,cin_layout.device_bytes,
      rotation_presence))
    return {NodalStatus::ResourceLimit,"Device byte budget or host staging extent is insufficient"};
  if (cin) {
    // Simultaneous retained source/optional arrays, complete existing owner
    // staging/constraints, and optional rigid arrays. No stale startup RSS is
    // treated as retained source memory.
    if (!nodal_detail::CinOwnerHostFits(cin_layout.host_bytes, rigid_layout.host_bytes,
        layout.accepted.count, layout.fixed.count, sizeof(Impl), cin->limits.max_host_bytes)) {
      return {NodalStatus::ResourceLimit, "Complete CIN owner host payload exceeds limits"};
    }
  }
  bool component_constraints = false;
  for (std::size_t i = 0; i < c.node_count; ++i) {
    const unsigned bits = rotations ? dofs->translation_fixed_bits[i] : (fixed[i] ? 7 : 0);
    const bool absent_rotation = rotation_presence && !dofs->rotation_present[i];
    // Exact compact source association below checks the coefficient and role.
    // Ordinary nodes retain their positive independent inverse requirements.
    const bool dependent_rigid = binding && binding->FindMember(i);
    const bool dependent_inverse = cin || dependent_rigid;
    if (rotation_presence && (dofs->rotation_present[i] > 1 ||
        (absent_rotation && (dofs->rotation_fixed[i] || dofs->inverse_inertia[i] != 0))))
      return {NodalStatus::InvalidInput, "Absent rotation requires zero J inverse and no fixed reaction", static_cast<std::uint32_t>(i)};
    if ((rotations ? bits > 7 : fixed[i] > 1) || !std::isfinite(inverse_mass[i]) ||
        (bits == 7 ? inverse_mass[i] != 0 : (dependent_inverse ? inverse_mass[i] < 0 : inverse_mass[i] <= 0)))
      return {NodalStatus::InvalidInput, "Invalid explicit mass or fixed mask", static_cast<std::uint32_t>(i)};
    component_constraints |= bits != 0 && bits != 7;
    if (rotations && (dofs->rotation_fixed[i] > 1 || !std::isfinite(dofs->inverse_inertia[i]) ||
        ((dofs->rotation_fixed[i] || absent_rotation) ? dofs->inverse_inertia[i] != 0 :
          (dependent_inverse ? dofs->inverse_inertia[i] < 0 : dofs->inverse_inertia[i] <= 0)) ||
        !nodal_detail::UnitQuaternion(nodal_detail::ReadQuaternion(in.orientation_wxyz + 4*i))))
      return {NodalStatus::InvalidInput, "Invalid isotropic inertia, rotation mask, or unit quaternion", static_cast<std::uint32_t>(i)};
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto j = 3*i+axis;
      if (!std::isfinite(in.position_xyz[j]) || !std::isfinite(in.velocity_xyz[j]) || ((bits & (1u << axis)) && in.velocity_xyz[j] != 0))
        return {NodalStatus::InvalidInput, "Invalid position or fixed-node velocity", static_cast<std::uint32_t>(i)};
      if (!rotations && in.angular_velocity_xyz && in.angular_velocity_xyz[j] != 0)
        return {NodalStatus::UnsupportedRotation, "Angular motion is not admitted", static_cast<std::uint32_t>(i)};
      if (rotations && in.angular_velocity_xyz && (!std::isfinite(in.angular_velocity_xyz[j]) ||
          ((dofs->rotation_fixed[i] || absent_rotation) && in.angular_velocity_xyz[j] != 0)))
        return {NodalStatus::InvalidInput, "Invalid angular or fixed-rotation velocity", static_cast<std::uint32_t>(i)};
    }
  }
  if (binding) {
    const auto report = nodal_detail::ValidateRigidAssemblyOwner(*binding,in,inverse_mass,*dofs,cin != nullptr);
    if (report.status != NodalStatus::Ok) return report;
  }
  std::unique_ptr<nodal_detail::RigidStorage> rigid_groups;
  if(groups || binding) {
    if(!rotations) return {NodalStatus::UnsupportedRotation,"Rigid groups require extended nodal state"};
    try {
      const auto report = binding
        ? nodal_detail::PrepareRigidStorage(*binding,c,in,inverse_mass,*dofs,rigid_layout,rigid_groups)
        : nodal_detail::PrepareRigidStorage(*groups,c,in,inverse_mass,*dofs,rigid_layout,rigid_groups);
      if(report.status!=NodalStatus::Ok) return report;
    } catch(const std::bad_alloc&) { return {NodalStatus::ResourceLimit,"Rigid host storage allocation failed"}; }
  }
  const auto state_values=layout.accepted.count,mask_bytes=layout.fixed.bytes;
  std::unique_ptr<nodal_detail::CinStorage> cin_storage;
  if (cin) {
    try {
      const auto report = nodal_detail::PrepareCinStorage(*cin, c, in, inverse_mass,
          *dofs, groups, cin_layout, cin_storage, binding);
      if (report.status != NodalStatus::Ok) return report;
      cin_storage->state_offset = 19*n+group_values;
    } catch (const std::bad_alloc&) {
      return {NodalStatus::ResourceLimit, "CIN host storage allocation failed"};
    }
  }
  try {
    auto next = std::make_unique<Impl>();
    next->rigid_groups=std::move(rigid_groups);
    next->cin=std::move(cin_storage);
    next->staging.resize(state_values,0.);
    next->constraint_staging.resize(mask_bytes,0);
    if (next->cin && !nodal_detail::CinOwnerHostFits(next->cin->layout.host_bytes,
        rigid_layout.host_bytes, next->staging.capacity(), next->constraint_staging.capacity(),
        sizeof(Impl), cin->limits.max_host_bytes)) {
      return {NodalStatus::ResourceLimit, "Actual complete CIN owner host capacities exceed limits"};
    }
    next->config = c; next->stamp = {NewOwner(), 0, n, 0, c.fixed_dt};
    if(next->rigid_groups) next->stamp.rigid_groups=next->rigid_groups->info;
    next->stamp.temporal_scheme = c.temporal_scheme;
    next->has_rotations = rotations; next->has_component_constraints = component_constraints;
    next->stamp.has_rotations = rotations; next->state_values = state_values;
    next->stamp.has_rotation_presence = rotation_presence;
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
    if (next->cin) {
      report = next->Check(next->cin->Upload(next->stream));
      if (report.status != NodalStatus::Ok) return report;
      next->allocation.device_bytes += next->cin->layout.device_bytes;
      ++next->allocation.device_allocations;
    }
    std::memcpy(next->staging.data(), in.position_xyz, 3*n*sizeof(double));
    std::memcpy(next->staging.data()+3*n, in.velocity_xyz, 3*n*sizeof(double));
    if (rotations) {
      if (in.angular_velocity_xyz) std::memcpy(next->staging.data()+6*n, in.angular_velocity_xyz, 3*n*sizeof(double));
      std::memcpy(next->staging.data()+9*n, in.orientation_wxyz, 4*n*sizeof(double));
    }
    if(next->rigid_groups) next->rigid_groups->InitializeState(next->staging.data()+19*n);
    if(next->cin) next->cin->InitializeState(next->staging.data(), *cin, inverse_mass, *dofs);
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
        if (rotation_presence) next->constraint_staging[3*n+i] = dofs->rotation_present[i];
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
  const double* current_inverse = cin ? accepted+cin->state_offset+2*n : inverse;
  view.mass = {current_inverse, fixed, static_cast<std::uint32_t>(n), stamp.epoch,
               has_component_constraints || rigid_groups || cin ? sc::TranslationMassModel::kUnspecified : sc::TranslationMassModel::kIsotropicLumped};
  view.stream = stream; view.owner_id = stamp.owner_id;
  view.temporal_scheme = stamp.temporal_scheme; view.velocity_phase = stamp.velocity_phase;
  view.position_time = stamp.time; view.velocity_time = stamp.velocity_time;
  view.rigid_groups=stamp.rigid_groups;
  if (has_rotations) {
    view.inverse_inertia = current_inverse+n; view.translation_fixed_bits = fixed+n; view.rotation_fixed = fixed+2*n;
    view.rotation_present = stamp.has_rotation_presence ? fixed+3*n : nullptr;
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
  if (s.cin) {
    report = s.Check(s.cin->ResetTrial(s.stream));
    if (report.status != NodalStatus::Ok) return report;
  }
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
  nodal_seal::Launch(s.control, s.scratch, static_cast<std::uint32_t>(s.config.node_count),
      s.stamp.epoch, s.attempt, s.config.timestep_safety, s.config.minimum_dt, s.config.fixed_dt, s.has_rotations, s.stream);
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
