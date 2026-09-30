#include "self_contact_broadphase/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>

namespace tlfea::contact {
namespace bp = self_contact_broadphase;
using S = SelfContactBroadphaseStatus;
namespace {
SelfContactBroadphaseReport DeviceError(cudaError_t error) {
  return {S::DeviceFailure, cudaGetErrorString(error)};
}
bool ExplicitStream(cudaStream_t stream) {
  return stream && stream != cudaStreamLegacy && stream != cudaStreamPerThread;
}
bool InputDisjoint(VectorView view, std::size_t nodes, const void* device, std::size_t bytes) {
  if (!view.valid() || view.node_count != nodes) return false;
  const auto values = (view.node_count - 1) * view.node_stride + 2 * view.component_stride + 1;
  if (values > SIZE_MAX / sizeof(double)) return false;
  return tl::fea::trial_identity::Disjoint(view.data, values * sizeof(double), device, bytes);
}
bool Empty(VectorView view) {
  return !view.data && !view.node_count && !view.node_stride &&
      !view.component_stride;
}
}
SelfContactBroadphase::SelfContactBroadphase() = default;
SelfContactBroadphase::~SelfContactBroadphase() = default;
SelfContactBroadphase::Impl::~Impl() { if (device) cudaFree(device); }
SelfContactBroadphasePreflight SelfContactBroadphase::Preflight(const SelfContactSurfaceBinding& source,
    SelfContactBroadphaseLimits limits) noexcept {
  const auto checked = bp::CheckSource(source, limits);
  if (checked.status != S::Ok) return {checked, {}};
  bp::ScratchRequirements scratch;
  const auto error = bp::QueryScratch(source.parents().size(), limits.max_pairs, scratch);
  if (error != cudaSuccess) return {DeviceError(error), {}};
  bp::Layout layout;
  const auto report = bp::MakeLayout(source, limits, scratch, sizeof(Impl), layout);
  return {report, report.status == S::Ok ? layout.forecast : SelfContactBroadphaseForecast{}};
}
SelfContactBroadphaseReport SelfContactBroadphase::Initialize(const SelfContactSurfaceBinding& source,
    SelfContactBroadphaseLimits limits, cudaStream_t stream) noexcept try {
  if (impl_) return {S::AlreadyInitialized, "Broadphase already initialized"};
  if (!ExplicitStream(stream) || !source.OutputDisjoint(this, sizeof(*this)))
    return {S::InvalidInput, "Explicit stream and disjoint destination required"};
  const auto checked = bp::CheckSource(source, limits);
  if (checked.status != S::Ok) return checked;
  bp::ScratchRequirements scratch;
  auto error = bp::QueryScratch(source.parents().size(), limits.max_pairs, scratch);
  if (error != cudaSuccess) return DeviceError(error);
  bp::Layout layout;
  const auto report = bp::MakeLayout(source, limits, scratch, sizeof(Impl), layout);
  if (report.status != S::Ok) return report;
  auto next = std::make_unique<Impl>(source);
  next->layout = layout;
  const tl::util::ArenaRegion bounds_region{
      0, layout.forecast.parents,
      layout.forecast.parents * sizeof(AABB)};
  if (!next->conservative_bounds.Initialize(bounds_region.bytes) ||
      !(next->staged_bounds =
            next->conservative_bounds.Construct<AABB>(bounds_region)))
    return {S::ResourceLimit,
            "Conservative parent-bound staging allocation failed"};
  tl::util::HostArena upload;
  if (!upload.Initialize(layout.parents.bytes)) return {S::ResourceLimit, "Parent upload allocation failed"};
  const tl::util::ArenaRegion region{0, layout.parents.count, layout.parents.bytes};
  auto* parents = upload.Construct<bp::Parent>(region);
  if (!bp::CopyParents(source, parents, layout.parents.count))
    return {S::InvalidInput, "Immutable typed parent map is inconsistent"};
  error = cudaMalloc(&next->device, layout.forecast.device_bytes);
  if (error != cudaSuccess) return DeviceError(error);
  error = cudaMemcpyAsync(tl::util::ArenaPointer<bp::Parent>(next->device, layout.parents), parents,
      layout.parents.bytes, cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return DeviceError(error);
  error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) return DeviceError(error);
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {S::ResourceLimit, "Broadphase host storage allocation failed"};
}
SelfContactBroadphaseReport SelfContactBroadphase::Evaluate(const SelfContactBroadphaseInput& input,
    cudaStream_t stream) noexcept {
  if (!impl_) return {S::NotInitialized, "Broadphase is not initialized"};
  auto& s = *impl_;
  s.complete = false; s.pair_count = 0;
  if (!s.usable) return {S::DeviceFailure, "Broadphase CUDA storage is poisoned"};
  const auto& f = s.layout.forecast;
  const bool swept = input.motion == SelfContactBoundsMotion::LinearNodalEndpoints;
  const bool conservative =
      input.motion == SelfContactBoundsMotion::ConservativeSweptParentBounds;
  const bool current = input.motion == SelfContactBoundsMotion::Current;
  const std::size_t bounds_bytes =
      f.parents * sizeof(SelfContactSweptParentBounds);
  const bool conservative_input =
      conservative && Empty(input.current) && Empty(input.endpoint) &&
      input.swept_parent_bounds &&
      input.swept_parent_count == f.parents &&
      s.source.OutputDisjoint(input.swept_parent_bounds, bounds_bytes) &&
      tl::fea::trial_identity::Disjoint(
          input.swept_parent_bounds, bounds_bytes,
          this, sizeof(*this)) &&
      tl::fea::trial_identity::Disjoint(
          input.swept_parent_bounds, bounds_bytes,
          &s, sizeof(s)) &&
      tl::fea::trial_identity::Disjoint(
          input.swept_parent_bounds, bounds_bytes,
          s.conservative_bounds.data(), s.conservative_bounds.bytes());
  const bool nodal_input =
      !conservative && !input.swept_parent_bounds &&
      !input.swept_parent_count &&
      InputDisjoint(input.current, f.nodes, s.device, f.device_bytes) &&
      (swept ? InputDisjoint(
                    input.endpoint, f.nodes, s.device, f.device_bytes)
             : Empty(input.endpoint));
  if (!ExplicitStream(stream) || input.axis > 2 ||
      (!current && !swept && !conservative) ||
      (!conservative_input && !nodal_input))
    return {S::InvalidInput, "Invalid owner-layout positions, motion mode, stream or alias"};
  const auto check = [&](cudaError_t error) {
    if (error != cudaSuccess) s.usable = false;
    return error;
  };
  auto error = check(cudaPeekAtLastError());
  if (error != cudaSuccess) return DeviceError(error);
  if (conservative) {
    for (std::size_t parent = 0; parent < f.parents; ++parent) {
      const auto& value = input.swept_parent_bounds[parent];
      if (!IsFinite(value.lower) || !IsFinite(value.upper) ||
          value.lower.x > value.upper.x ||
          value.lower.y > value.upper.y ||
          value.lower.z > value.upper.z)
        return {S::InvalidBounds,
                "Nonfinite or inverted conservative parent bounds",
                static_cast<std::uint32_t>(parent)};
      s.staged_bounds[parent] = {
          make_double3(value.lower.x, value.lower.y, value.lower.z),
          make_double3(value.upper.x, value.upper.y, value.upper.z),
          static_cast<int>(parent)};
    }
    error = check(cudaMemcpyAsync(
        tl::util::ArenaPointer<AABB>(s.device, s.layout.boxes),
        s.staged_bounds, s.layout.boxes.bytes,
        cudaMemcpyHostToDevice, stream));
    if (error != cudaSuccess) return DeviceError(error);
  } else {
    error = check(bp::Bounds(s.device, s.layout, input, stream));
    if (error != cudaSuccess) return DeviceError(error);
    error = check(cudaMemcpyAsync(&s.host_control,
        tl::util::ArenaPointer<bp::Control>(s.device, s.layout.control),
        sizeof(s.host_control), cudaMemcpyDeviceToHost, stream));
    if (error != cudaSuccess) return DeviceError(error);
    error = check(cudaStreamSynchronize(stream));
    if (error != cudaSuccess) return DeviceError(error);
    if (s.host_control.invalid_parent != UINT32_MAX)
      return {S::InvalidBounds, "Nonfinite or unrepresentable parent bounds", s.host_control.invalid_parent};
  }
  error = check(bp::SortBoxes(s.device, s.layout, input.axis, stream));
  if (error != cudaSuccess) return DeviceError(error);
  error = check(bp::Count(s.device, s.layout, input.axis, stream));
  if (error != cudaSuccess) return DeviceError(error);
  error = check(bp::ScanCounts(s.device, s.layout, stream));
  if (error != cudaSuccess) return DeviceError(error);
  const auto* total = tl::util::ArenaPointer<unsigned long long>(s.device, s.layout.offsets) + f.parents;
  error = check(cudaMemcpyAsync(&s.host_control.count, total, sizeof(s.host_control.count),
      cudaMemcpyDeviceToHost, stream));
  if (error != cudaSuccess) return DeviceError(error);
  error = check(cudaStreamSynchronize(stream));
  if (error != cudaSuccess) return DeviceError(error);
  const auto count = s.host_control.count;
  if (count > f.pair_capacity)
    return {S::PairCapacity, "Complete pair set exceeds preallocated capacity", UINT32_MAX, count};
  if (count) {
    error = check(bp::Fill(s.device, s.layout, input.axis, stream));
    if (error != cudaSuccess) return DeviceError(error);
    error = check(bp::SortPairs(s.device, s.layout, static_cast<int>(count), stream));
    if (error != cudaSuccess) return DeviceError(error);
    error = check(cudaStreamSynchronize(stream));
    if (error != cudaSuccess) return DeviceError(error);
  }
  s.pair_count = count; s.complete = true;
  return {};
}
bool SelfContactBroadphase::initialized() const noexcept { return bool(impl_); }
const SelfContactSurfaceBinding* SelfContactBroadphase::source() const noexcept {
  return impl_ ? &impl_->source : nullptr;
}
SelfContactBroadphaseForecast SelfContactBroadphase::forecast() const noexcept {
  return impl_ ? impl_->layout.forecast : SelfContactBroadphaseForecast{};
}
SelfContactBroadphasePairs SelfContactBroadphase::pairs() const noexcept {
  if (!impl_ || !impl_->complete) return {};
  const auto& s = *impl_;
  return {s.pair_count ? tl::util::ArenaPointer<SelfContactPairKey>(s.device, s.layout.sorted_pair_keys) : nullptr,
          s.pair_count, true};
}
} // namespace tlfea::contact
