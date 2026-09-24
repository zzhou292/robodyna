// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Environment.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>
#include <utility>

namespace tlfea::contact::self_contact_filters {
namespace {
Report DeviceError(cudaError_t error) noexcept {
  return {Status::DeviceFailure, cudaGetErrorString(error)};
}
bool ExplicitStream(cudaStream_t stream) noexcept {
  return stream && stream != cudaStreamLegacy && stream != cudaStreamPerThread;
}
template <class T>
bool Range(const T* data, std::size_t count) noexcept {
  const auto address = reinterpret_cast<std::uintptr_t>(data);
  return !count || (data && address % alignof(T) == 0 &&
      count <= (UINTPTR_MAX - address) / sizeof(T));
}
bool Outside(const void* input, std::size_t bytes, const void* owner,
             std::size_t owner_bytes, const void* impl, std::size_t impl_bytes,
             const tl::util::HostArena& host) noexcept {
  using tl::fea::trial_identity::Disjoint;
  return !bytes || (Disjoint(input, bytes, owner, owner_bytes) &&
      Disjoint(input, bytes, impl, impl_bytes) &&
      Disjoint(input, bytes, host.data(), host.bytes()));
}
}  // namespace

Batch::Impl::~Impl() { if (device) cudaFree(device); }
Batch::Batch() = default;
Batch::~Batch() = default;
Preflight Batch::PreflightLimits(Limits limits) noexcept {
  Layout layout;
  Preflight result;
  result.report = MakeLayout(limits, sizeof(Batch) + sizeof(Impl), layout);
  if (result.report.status == Status::Ok) result.forecast = layout.forecast;
  return result;
}
Report Batch::Initialize(Limits limits, cudaStream_t stream) noexcept try {
  if (impl_) return {Status::AlreadyInitialized, "Filter batch is already initialized"};
  if (!ExplicitStream(stream)) return {Status::InvalidInput, "Filter batch requires an explicit stream"};
  Layout layout;
  const auto report = MakeLayout(limits, sizeof(Batch) + sizeof(Impl), layout);
  if (report.status != Status::Ok) return report;
  unsigned flags = 0;
  auto error = cudaStreamGetFlags(stream, &flags);
  if (error != cudaSuccess) return DeviceError(error);
  auto next = std::make_unique<Impl>();
  next->layout = layout;
  next->stream = stream;
  if (!next->host.Initialize(layout.host_results.bytes) ||
      !(next->staging = next->host.Construct<PairResult>(layout.host_results)))
    return {Status::ResourceLimit, "Filter batch host allocation failed"};
  error = cudaMalloc(&next->device, layout.forecast.device_bytes);
  if (error != cudaSuccess) return DeviceError(error);
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {Status::ResourceLimit, "Filter batch owner allocation failed"};
}

Report Batch::Upload(SceneView scene, cudaStream_t stream) noexcept {
  if (!impl_) return {Status::NotInitialized, "Filter batch is not initialized"};
  auto& state = *impl_;
  state.complete = false;
  state.scene_ready = false;
  state.result_count = state.scene_count = 0;
  if (!state.usable) return {Status::DeviceFailure, "Filter batch CUDA storage is poisoned"};
  if (stream != state.stream || !scene.count || scene.count > state.layout.forecast.facets ||
      !Range(scene.accepted, scene.count) || !Range(scene.prepared, scene.count) ||
      !Range(scene.properties, scene.count))
    return {Status::InvalidInput, "Filter batch scene range or stream is invalid"};
  const auto geometry_bytes = scene.count * sizeof(TriangleGeometry);
  const auto property_bytes = scene.count * sizeof(FacetProperties);
  const auto outside = [&](const void* input, std::size_t bytes) {
    return Outside(input, bytes, this, sizeof(*this), &state, sizeof(state), state.host);
  };
  if (!outside(scene.accepted, geometry_bytes) || !outside(scene.prepared, geometry_bytes) ||
      !outside(scene.properties, property_bytes))
    return {Status::InvalidInput, "Filter batch scene aliases owned state"};
  if (state.generation == UINT64_MAX)
    return {Status::ResourceLimit, "Filter batch scene generation exhausted"};
  auto error = cudaMemcpyAsync(tl::util::ArenaPointer<TriangleGeometry>(state.device, state.layout.accepted),
      scene.accepted, geometry_bytes, cudaMemcpyHostToDevice, stream);
  if (error == cudaSuccess)
    error = cudaMemcpyAsync(tl::util::ArenaPointer<TriangleGeometry>(state.device, state.layout.prepared),
        scene.prepared, geometry_bytes, cudaMemcpyHostToDevice, stream);
  if (error == cudaSuccess)
    error = cudaMemcpyAsync(tl::util::ArenaPointer<FacetProperties>(state.device, state.layout.properties),
        scene.properties, property_bytes, cudaMemcpyHostToDevice, stream);
  // Always drain previously enqueued borrowed reads, even after a later copy
  // fails. Preserve the first CUDA error; no borrowed pointer survives return.
  const auto synchronized = cudaStreamSynchronize(stream);
  if (error == cudaSuccess) error = synchronized;
  if (error != cudaSuccess) {
    state.usable = false;
    return DeviceError(error);
  }
  ++state.generation;
  state.scene_count = scene.count;
  state.scene_ready = true;
  return {};
}

Report Batch::Accepted(PairView pairs, cudaStream_t stream) noexcept {
  return Evaluate(pairs, true, SelfContactFacetPrismAxisLimit::VertexVertex, stream);
}
Report Batch::Linear(PairView pairs, SelfContactFacetPrismAxisLimit limit,
                     cudaStream_t stream) noexcept {
  return Evaluate(pairs, false, limit, stream);
}
Report Batch::Evaluate(PairView pairs, bool accepted,
                      SelfContactFacetPrismAxisLimit limit, cudaStream_t stream) noexcept {
  if (!impl_) return {Status::NotInitialized, "Filter batch is not initialized"};
  auto& state = *impl_;
  state.complete = false;
  state.result_count = 0;
  if (!state.usable) return {Status::DeviceFailure, "Filter batch CUDA storage is poisoned"};
  if (stream != state.stream || pairs.count > state.layout.forecast.pairs ||
      !Range(pairs.data, pairs.count) ||
      static_cast<std::uint8_t>(limit) > static_cast<std::uint8_t>(SelfContactFacetPrismAxisLimit::VertexVertex))
    return {Status::InvalidInput, "Filter batch pair range, axis limit or stream is invalid"};
  if (!state.scene_ready) return {Status::NoScene, "Filter batch has no complete scene"};
  if (!CompatibleHostArithmetic())
    return {Status::UnsupportedEnvironment, "Filter batch requires host RN, gradual underflow and masked traps"};
  const auto bytes = pairs.count * sizeof(FixedTrianglePair);
  if (!Outside(pairs.data, bytes, this, sizeof(*this), &state, sizeof(state), state.host))
    return {Status::InvalidInput, "Filter batch pairs alias owned state"};
  // Canonical first malformed pair, before any device read. Input order,
  // repetitions and both orientations are retained; no sorting/truncation.
  for (std::size_t i = 0; i < pairs.count; ++i)
    if (pairs.data[i].first >= state.scene_count || pairs.data[i].second >= state.scene_count ||
        pairs.data[i].first == pairs.data[i].second)
      return {Status::InvalidInput, "Filter batch facet ordinal is invalid", i};
  if (!pairs.count) {
    state.complete = true;
    return {};
  }
  auto error = cudaMemcpyAsync(tl::util::ArenaPointer<FixedTrianglePair>(state.device, state.layout.pairs),
      pairs.data, bytes, cudaMemcpyHostToDevice, stream);
  if (error == cudaSuccess) error = Launch(state.device, state.layout, pairs.count, accepted, limit, stream);
  if (error == cudaSuccess)
    error = cudaMemcpyAsync(state.staging,
        tl::util::ArenaPointer<PairResult>(state.device, state.layout.results),
        pairs.count * sizeof(PairResult), cudaMemcpyDeviceToHost, stream);
  const auto synchronized = cudaStreamSynchronize(stream);
  if (error == cudaSuccess) error = synchronized;
  if (error != cudaSuccess) {
    state.usable = state.scene_ready = false;
    return DeviceError(error);
  }
  state.result_count = pairs.count;
  state.complete = true;
  return {};
}
Forecast Batch::forecast() const noexcept { return impl_ ? impl_->layout.forecast : Forecast{}; }
ResultView Batch::results() const noexcept {
  if (!impl_ || !impl_->complete || !impl_->scene_ready || !impl_->usable) return {};
  return {impl_->staging, impl_->result_count, impl_->generation, true};
}
}  // namespace tlfea::contact::self_contact_filters
