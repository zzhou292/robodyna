#include "FENodalStateStorage.h"
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
bool Overlap(const void* a, std::size_t an, const void* b, std::size_t bn) {
  const auto x = reinterpret_cast<std::uintptr_t>(a), y = reinterpret_cast<std::uintptr_t>(b);
  if (an > UINTPTR_MAX - x || bn > UINTPTR_MAX - y) return true;
  return x < y + bn && y < x + an;
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
                          double safety, double minimum_dt, double h) {
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
      if (axis >= 3 && value != 0) { c->status = NodalStatus::UnsupportedRotation; c->node = i; return; }
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
  if (impl_) return {NodalStatus::InvalidInput, "Owner already initialized"};
  if (!c.node_count || c.node_count > MaxTranslationNodes || !c.max_device_bytes ||
      c.max_device_bytes > MaxTranslationDeviceBytes)
    return {NodalStatus::ResourceLimit, "Translation capacity exceeds admitted limits"};
  if (in.node_count != c.node_count || !in.position_xyz || !in.velocity_xyz || !inverse_mass || !fixed ||
      !std::isfinite(c.fixed_dt) || !std::isfinite(c.minimum_dt) || c.minimum_dt <= 0 ||
      c.fixed_dt < c.minimum_dt || !std::isfinite(c.timestep_safety) || c.timestep_safety <= 0 || c.timestep_safety >= 1)
    return {NodalStatus::InvalidInput, "Invalid kinematics, mass, or fixed-step configuration"};
  for (std::size_t i = 0; i < c.node_count; ++i) {
    if (fixed[i] > 1 || !std::isfinite(inverse_mass[i]) || (fixed[i] ? inverse_mass[i] != 0 : inverse_mass[i] <= 0))
      return {NodalStatus::InvalidInput, "Invalid explicit mass or fixed mask", static_cast<std::uint32_t>(i)};
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto j = 3*i+axis;
      if (!std::isfinite(in.position_xyz[j]) || !std::isfinite(in.velocity_xyz[j]) || (fixed[i] && in.velocity_xyz[j] != 0))
        return {NodalStatus::InvalidInput, "Invalid position or fixed-node velocity", static_cast<std::uint32_t>(i)};
      if (in.angular_velocity_xyz && in.angular_velocity_xyz[j] != 0)
        return {NodalStatus::UnsupportedRotation, "Angular motion is not admitted", static_cast<std::uint32_t>(i)};
    }
  }
  const auto n = c.node_count;
  const std::size_t bytes = 24*n*sizeof(double) + n*sizeof(std::uint8_t) + sizeof(Control);
  if (bytes > c.max_device_bytes) return {NodalStatus::ResourceLimit, "Device byte budget is insufficient"};
  try {
    auto next = std::make_unique<Impl>();
    next->config = c; next->stamp = {NewOwner(), 0, n, 0, c.fixed_dt};
    if (!next->stamp.owner_id) return {NodalStatus::HistoryLimit, "Owner identities exhausted"};
    auto report = next->Check(cudaStreamCreateWithFlags(&next->stream, cudaStreamNonBlocking));
    if (report.status != NodalStatus::Ok) return report;
    auto allocate = [&](auto** p, std::size_t size) {
      auto r = next->Check(cudaMalloc(reinterpret_cast<void**>(p), size));
      if (r.status == NodalStatus::Ok) { next->allocation.device_bytes += size; ++next->allocation.device_allocations; }
      return r;
    };
    report = allocate(&next->accepted, 6*n*sizeof(double)); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->trial, 6*n*sizeof(double)); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->scratch, 11*n*sizeof(double)); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->inverse, n*sizeof(double)); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->fixed, n); if (report.status != NodalStatus::Ok) return report;
    report = allocate(&next->control, sizeof(Control)); if (report.status != NodalStatus::Ok) return report;
    std::memcpy(next->staging.data(), in.position_xyz, 3*n*sizeof(double));
    std::memcpy(next->staging.data()+3*n, in.velocity_xyz, 3*n*sizeof(double));
    report = next->Check(cudaMemcpyAsync(next->accepted, next->staging.data(), 6*n*sizeof(double), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaMemcpyAsync(next->inverse, inverse_mass, n*sizeof(double), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaMemcpyAsync(next->fixed, fixed, n, cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaMemsetAsync(next->scratch, 0, 11*n*sizeof(double), next->stream));
    if (report.status != NodalStatus::Ok) return report;
    next->host_control.rows = {next->scratch+6*n, next->scratch+7*n, static_cast<std::uint32_t>(n), static_cast<std::uint32_t>(n)};
    report = next->Check(cudaMemcpyAsync(next->control, &next->host_control, sizeof(Control), cudaMemcpyHostToDevice, next->stream));
    if (report.status != NodalStatus::Ok) return report;
    report = next->Check(cudaStreamSynchronize(next->stream));
    if (report.status != NodalStatus::Ok) return report;
    impl_ = std::move(next); return Ok();
  } catch (const std::bad_alloc&) { return {NodalStatus::ResourceLimit, "Host state allocation failed"}; }
}

NodalReport FENodalState::BeginTrial(NodalTrialToken* token, NodalAssemblyView* view) {
  if (token) *token = {}; if (view) *view = {};
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_; s.phase = Phase::Idle;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!token || !view) return {NodalStatus::InvalidInput, "Missing trial output"};
  if (s.attempt == UINT64_MAX || s.stamp.epoch == UINT64_MAX)
    return {NodalStatus::HistoryLimit, "Step provenance exhausted"};
  ++s.attempt;
  s.candidate_time = s.stamp.time + s.config.fixed_dt;
  if (!std::isfinite(s.candidate_time) || s.candidate_time <= s.stamp.time)
    return {NodalStatus::HistoryLimit, "Accepted clock cannot represent another step"};
  const auto n = s.config.node_count;
  auto report = s.Check(cudaMemcpyAsync(s.trial, s.accepted, 6*n*sizeof(double), cudaMemcpyDeviceToDevice, s.stream));
  if (report.status != NodalStatus::Ok) return report;
  report = s.Check(cudaMemsetAsync(s.scratch, 0, 11*n*sizeof(double), s.stream));
  if (report.status != NodalStatus::Ok) return report;
  ResetTrial<<<1,1,0,s.stream>>>(s.control, s.stamp.epoch, s.attempt);
  report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  token->owner_id_ = s.stamp.owner_id; token->base_epoch_ = s.stamp.epoch; token->attempt_ = s.attempt;
  view->accepted = {s.accepted, s.accepted+3*n, s.scratch+8*n, n, s.stamp.epoch};
  view->mass = {s.inverse, s.fixed, static_cast<std::uint32_t>(n), s.stamp.epoch, sc::TranslationMassModel::kIsotropicLumped};
  view->forces = {s.scratch, s.scratch+n, s.scratch+2*n, s.scratch+3*n, s.scratch+4*n, s.scratch+5*n, n, s.stamp.epoch};
  view->bounds = &s.control->rows; view->result = &s.control->assembly;
  view->stream = s.stream; view->attempt = s.attempt;
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
      s.stamp.epoch, s.attempt, s.config.timestep_safety, s.config.minimum_dt, s.config.fixed_dt);
  report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  s.phase = Phase::Sealed; return Ok();
}
NodalReport FENodalState::Commit(const NodalTrialToken& token) noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return s.Reject(NodalStatus::StaleTrial, "Trial token belongs to another owner or attempt");
  if (s.phase != Phase::Ready) return s.Reject(NodalStatus::WrongPhase, "No completed valid advance");
  std::swap(s.accepted, s.trial); ++s.stamp.epoch; s.stamp.time = s.candidate_time;
  s.phase = Phase::Idle; return Ok();
}
void FENodalState::Discard() noexcept { if (impl_) impl_->phase = Phase::Idle; }
NodalStamp FENodalState::accepted() const noexcept { return impl_ ? impl_->stamp : NodalStamp{}; }
NodalAllocationInfo FENodalState::allocations() const noexcept { return impl_ ? impl_->allocation : NodalAllocationInfo{}; }

NodalReport FENodalState::CopyAccepted(NodalSnapshotBuffer out, NodalStamp* stamp) {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!stamp || !out.position_xyz || !out.velocity_xyz)
    return {NodalStatus::InvalidInput, "Missing host snapshot output"};
  if (out.capacity_nodes < s.config.node_count) return {NodalStatus::ResourceLimit, "Snapshot capacity is insufficient"};
  const auto bytes = 3*s.config.node_count*sizeof(double);
  if (Overlap(out.position_xyz, bytes, out.velocity_xyz, bytes) ||
      Overlap(out.position_xyz, bytes, stamp, sizeof(*stamp)) || Overlap(out.velocity_xyz, bytes, stamp, sizeof(*stamp)))
    return {NodalStatus::InvalidInput, "Snapshot outputs overlap or overflow their address range"};
  auto report = s.Check(cudaMemcpyAsync(s.staging.data(), s.accepted, 2*bytes, cudaMemcpyDeviceToHost, s.stream));
  if (report.status != NodalStatus::Ok) return report;
  report = s.Check(cudaStreamSynchronize(s.stream)); if (report.status != NodalStatus::Ok) return report;
  for (std::size_t i = 0; i < 6*s.config.node_count; ++i)
    if (!std::isfinite(s.staging[i])) return s.Reject(NodalStatus::InvalidOutput, "Accepted readback is nonfinite");
  std::memcpy(out.position_xyz, s.staging.data(), bytes);
  std::memcpy(out.velocity_xyz, s.staging.data()+3*s.config.node_count, bytes);
  *stamp = s.stamp; return Ok();
}
}  // namespace tl::fea
