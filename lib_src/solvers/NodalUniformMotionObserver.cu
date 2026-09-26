#include "NodalUniformMotionObserver.h"
#include "FENodalStateStorage.h"
#include "NodalTrialIdentity.h"
#include "nodal_motion/Values.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <cmath>
#include <new>

namespace tl::fea {
namespace {
constexpr unsigned Threads = 128;
constexpr std::size_t MaxBlocks = 1024;
using Summary = nodal_motion::Summary;
struct Layout {
  util::ArenaRegion reference, partial, result;
  std::size_t bytes = 0, blocks = 0;
  bool Prepare(std::size_t nodes, NodalUniformMotionLimits limits) {
    if (!nodes || !limits.max_nodes || limits.max_nodes > MaxActiveNodalStateNodes ||
        nodes > limits.max_nodes || !limits.max_device_bytes ||
        limits.max_device_bytes > (std::size_t{32} << 20) || !limits.max_host_bytes)
      return false;
    blocks = std::min(MaxBlocks, (nodes + Threads - 1) / Threads);
    util::BoundedArenaLayout budget(limits.max_device_bytes);
    if (!budget.Append<double>(3 * nodes, reference) ||
        !budget.Append<Summary>(blocks, partial) || !budget.Append<Summary>(1, result))
      return false;
    bytes = budget.bytes();
    return true;
  }
};
__global__ void InspectMotion(const double* state, std::size_t values,
    std::size_t nodes, bool rotations, const double* reference,
    tl::math::Vec3 velocity, double time, bool observe, Summary* partial) {
  Summary local = nodal_motion::Empty();
  const auto first = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto stride = std::size_t(gridDim.x) * blockDim.x;
  for (auto i = first; i < values; i += stride)
    if (!tl::math::Finite(state[i])) nodal_motion::Minimum(local.nonfinite, i);
  for (auto node = first; node < nodes; node += stride) {
    if (rotations && !nodal_detail::UnitQuaternion(
            nodal_detail::ReadQuaternion(state + 9 * nodes + 4 * node)))
      nodal_motion::Minimum(local.quaternion, node);
    if (!observe) continue;
    const double speed[]{velocity.x, velocity.y, velocity.z};
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto at = 3 * node + axis;
      // The owned target disables FMA. Zero velocity preserves the original
      // coordinate directly, matching the existing X-only app observer.
      const double increment = speed[axis] * time;
      const double expected = speed[axis] == 0 ? reference[at] : reference[at] + increment;
      nodal_motion::Difference(local, state[at], expected, local.position, 13 * node + 3 * axis);
      nodal_motion::Difference(local, state[3 * nodes + at], speed[axis], local.velocity, 13 * node + 3 * axis + 1);
      if (rotations)
        nodal_motion::Difference(local, state[6 * nodes + at], 0, local.spin, 13 * node + 3 * axis + 2);
    }
    if (rotations)
      for (unsigned axis = 0; axis < 4; ++axis)
        nodal_motion::Difference(local, state[9 * nodes + 4 * node + axis],
            axis == 0 ? 1 : 0, local.orientation, 13 * node + 9 + axis);
  }
  __shared__ Summary shared[Threads];
  shared[threadIdx.x] = local;
  __syncthreads();
  for (unsigned offset = Threads / 2; offset; offset /= 2) {
    if (threadIdx.x < offset) nodal_motion::Merge(shared[threadIdx.x], shared[threadIdx.x + offset]);
    __syncthreads();
  }
  if (!threadIdx.x) partial[blockIdx.x] = shared[0];
}
__global__ void FinishMotion(const Summary* partial, std::size_t blocks, Summary* result) {
  Summary local = nodal_motion::Empty();
  for (std::size_t i = threadIdx.x; i < blocks; i += blockDim.x) nodal_motion::Merge(local, partial[i]);
  __shared__ Summary shared[Threads];
  shared[threadIdx.x] = local;
  __syncthreads();
  for (unsigned offset = Threads / 2; offset; offset /= 2) {
    if (threadIdx.x < offset) nodal_motion::Merge(shared[threadIdx.x], shared[threadIdx.x + offset]);
    __syncthreads();
  }
  if (!threadIdx.x) *result = shared[0];
}
bool Range(const void* pointer, std::size_t bytes) {
  return pointer && bytes <= UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(pointer);
}
}
struct NodalUniformMotionObserver::Impl {
  ~Impl() { if (arena) cudaFree(arena); }
  FENodalState* owner = nullptr;
  std::uint64_t owner_id = 0;
  std::size_t nodes = 0;
  bool rotations = false;
  void* arena = nullptr;
  double* reference = nullptr;
  Summary* partial = nullptr;
  Summary* result = nullptr;
  Summary host;
  NodalUniformMotionForecast forecast;
  cudaError_t Run(const double* state, std::size_t values, cudaStream_t stream,
      tl::math::Vec3 velocity, double time, bool observe) {
    InspectMotion<<<unsigned(forecast.blocks), Threads, 0, stream>>>(
        state, values, nodes, rotations, reference, velocity, time, observe, partial);
    auto error = cudaPeekAtLastError();
    if (error == cudaSuccess) {
      FinishMotion<<<1, Threads, 0, stream>>>(partial, forecast.blocks, result);
      error = cudaPeekAtLastError();
    }
    if (error == cudaSuccess)
      error = cudaMemcpyAsync(&host, result, sizeof(host), cudaMemcpyDeviceToHost, stream);
    // Always drain before private storage can retire, including an earlier
    // launch/copy failure. Both checks use the same physical owner's policy.
    const auto drained = cudaStreamSynchronize(stream);
    return error == cudaSuccess ? drained : error;
  }
};
NodalUniformMotionObserver::NodalUniformMotionObserver() = default;
NodalUniformMotionObserver::~NodalUniformMotionObserver() = default;
NodalUniformMotionForecast NodalUniformMotionObserver::Preflight(
    std::size_t nodes, NodalUniformMotionLimits limits) noexcept {
  NodalUniformMotionForecast out;
  Layout layout;
  if (!layout.Prepare(nodes, limits) || sizeof(Impl) > limits.max_host_bytes) {
    out.report = {NodalStatus::ResourceLimit, "Motion observer capacity is insufficient"};
    return out;
  }
  out.device_bytes = layout.bytes;
  out.host_bytes = sizeof(Impl);
  out.blocks = layout.blocks;
  out.report = {NodalStatus::Ok, "OK"};
  return out;
}
NodalReport NodalUniformMotionObserver::Initialize(FENodalState& owner, NodalUniformMotionLimits limits) {
  if (impl_) return {NodalStatus::WrongPhase, "Motion observer is already initialized"};
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& state = *owner.impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (state.stamp.epoch || state.stamp.time != 0 || state.phase != nodal_detail::Phase::Idle)
    return {NodalStatus::WrongPhase, "Motion reference requires a fresh idle owner"};
  const auto forecast = Preflight(state.config.node_count, limits);
  if (forecast.report.status != NodalStatus::Ok) return forecast.report;
  std::unique_ptr<Impl> next;
  try { next = std::make_unique<Impl>(); }
  catch (const std::bad_alloc&) { return {NodalStatus::ResourceLimit, "Motion observer host allocation failed"}; }
  next->owner = &owner;
  next->owner_id = state.stamp.owner_id;
  next->nodes = state.config.node_count;
  next->rotations = state.has_rotations;
  next->forecast = forecast;
  Layout layout;
  layout.Prepare(next->nodes, limits);
  auto report = state.Check(cudaMalloc(&next->arena, forecast.device_bytes));
  if (report.status != NodalStatus::Ok) return report;
  next->reference = util::ArenaPointer<double>(next->arena, layout.reference);
  next->partial = util::ArenaPointer<Summary>(next->arena, layout.partial);
  next->result = util::ArenaPointer<Summary>(next->arena, layout.result);
  auto error = cudaMemcpyAsync(next->reference, state.accepted, layout.reference.bytes,
      cudaMemcpyDeviceToDevice, state.stream);
  if (error == cudaSuccess) error = next->Run(state.accepted, state.state_values, state.stream, {}, 0, false);
  else {
    const auto drained = cudaStreamSynchronize(state.stream);
    (void)drained;
  }
  report = state.Check(error);
  if (report.status != NodalStatus::Ok) return report;
  if (next->host.nonfinite != UINT64_MAX)
    return state.Reject(NodalStatus::InvalidOutput, "Accepted readback is nonfinite");
  if (next->host.quaternion != UINT64_MAX)
    return state.Reject(NodalStatus::InvalidOutput, "Accepted quaternion readback is not unit",
        std::uint32_t(next->host.quaternion));
  impl_ = std::move(next);
  return {NodalStatus::Ok, "OK"};
}
NodalReport NodalUniformMotionObserver::ObservePrepared(FENodalState& owner,
    const NodalTrialToken& token, tl::math::Vec3 velocity, NodalUniformMotionObservation* output) {
  if (!impl_) return {NodalStatus::NotInitialized, "Motion observer is not initialized"};
  auto& observed = *impl_;
  if (!Range(output, sizeof(*output)) ||
      !trial_identity::Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !trial_identity::Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !trial_identity::Disjoint(output, sizeof(*output), &observed, sizeof(observed)) ||
      !trial_identity::Disjoint(output, sizeof(*output), observed.arena, observed.forecast.device_bytes) ||
      !trial_identity::Disjoint(output, sizeof(*output), &owner, sizeof(owner)) ||
      !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z))
    return {NodalStatus::InvalidInput, "Motion observation output or velocity is invalid"};
  if (&owner != observed.owner || !owner.impl_ || owner.accepted().owner_id != observed.owner_id)
    return {NodalStatus::StaleTrial, "Motion reference belongs to another owner"};
  auto& state = *owner.impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  NodalPreparedView prepared;
  auto report = owner.BorrowPrepared(token, &prepared);
  if (report.status != NodalStatus::Ok) return report;
  if (!std::isfinite(prepared.proposed_time) || prepared.proposed_time < 0)
    return state.Reject(NodalStatus::InvalidOutput, "Motion time is invalid");
  report = state.Check(observed.Run(state.trial, state.state_values, state.stream,
      velocity, prepared.proposed_time, true));
  if (report.status != NodalStatus::Ok) return report;
  const auto& result = observed.host;
  // Same priority as CopyPrepared followed by the CPU motion observer.
  if (result.nonfinite != UINT64_MAX)
    return state.Reject(NodalStatus::InvalidOutput, "Prepared readback is nonfinite");
  if (result.quaternion != UINT64_MAX)
    return state.Reject(NodalStatus::InvalidOutput, "Prepared quaternion readback is not unit",
        std::uint32_t(result.quaternion));
  if (result.motion != UINT64_MAX)
    return state.Reject(NodalStatus::InvalidOutput, "Nonfinite motion field difference",
        std::uint32_t(result.motion / 13));
  NodalUniformMotionObservation next;
  next.prepared = prepared;
  next.motion = {observed.nodes, result.position, result.velocity, result.orientation, result.spin};
  *output = next;
  return {NodalStatus::Ok, "OK"};
}
NodalAllocationInfo NodalUniformMotionObserver::allocations() const noexcept {
  return impl_ ? NodalAllocationInfo{impl_->forecast.device_bytes, 1} : NodalAllocationInfo{};
}
const NodalUniformMotionForecast& NodalUniformMotionObserver::forecast() const noexcept {
  static const NodalUniformMotionForecast empty;
  return impl_ ? impl_->forecast : empty;
}
} // namespace tl::fea
