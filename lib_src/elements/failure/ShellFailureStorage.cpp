#include "ShellFailureStorage.h"
#include "ShellFailureValues.h"
#include <algorithm>

namespace tl::fea::shell_batch_plasticity_detail {
FailureHostStorage::~FailureHostStorage() {
  if (device_) cudaFree(device_);
}

bool FailureHostStorage::Forecast(std::size_t count, std::size_t binding_bytes,
    std::size_t device_cap, const ShellBatchFailureLimits& limits,
    FailureLayout& output, std::size_t& host_bytes) noexcept {
  if (!limits.max_parents || limits.max_parents > 524288 || count > limits.max_parents ||
      !limits.max_device_bytes || limits.max_device_bytes > 256ULL * 1024 * 1024 ||
      !limits.max_host_bytes || limits.max_host_bytes > 512ULL * 1024 * 1024 ||
      binding_bytes < sizeof(ShellBatchFailureBinding)) {
    return false;
  }
  FailureLayout layout;
  util::BoundedArenaLayout host(limits.max_host_bytes);
  util::ArenaRegion ignored;
  // The immutable binding's reported payload already includes its handle,
  // which is embedded in FailureHostStorage. Charge that handle only once.
  const auto object_bytes = sizeof(FailureHostStorage) - sizeof(ShellBatchFailureBinding);
  if (!layout.Initialize(count, std::min(device_cap, limits.max_device_bytes)) ||
      !host.Append<unsigned char>(object_bytes, ignored) ||
      !host.Append<unsigned char>(binding_bytes, ignored) ||
      !host.Append<unsigned char>(layout.bytes, ignored) ||
      !host.Append<ShellBatchFailureState>(count, ignored) ||
      !host.Append<unsigned char>(64, ignored)) {
    return false;
  }
  output = layout;
  host_bytes = host.bytes();
  return true;
}

SetupReport FailureHostStorage::Initialize(const ShellBatchFailureBinding& binding,
    ShellBindingFamily family, std::size_t count, const FailureLayout& layout) {
  if (device_ || !binding.prepared() || count != layout.policy.count ||
      (family != ShellBindingFamily::Qeph && family != ShellBindingFamily::T3)) {
    return {SetupStatus::InvalidInput, "Invalid failure sidecar source/layout"};
  }
  util::HostArena arena;
  if (!arena.Initialize(layout.bytes)) {
    return {SetupStatus::ResourceLimit, "Failure sidecar host allocation failed"};
  }
  auto* initial = layout.Construct(arena);
  if (!initial) {
    return {SetupStatus::ResourceLimit, "Failure sidecar layout construction failed"};
  }
  staging_.Resize(count);
  for (std::size_t e = 0; e < count; ++e) {
    const auto* source = binding.parent(family, e);
    if (!source) {
      return {SetupStatus::InvalidInput, "Failure sidecar family source is incomplete"};
    }
    initial->policy[e] = source->policy;
    initial->parameters[e] = source->constant;
    initial->tab1_parameters[e] = source->tab1;
    for (auto* slab : initial->state) {
      if (source->policy == ShellFailurePolicy::ConstantAllPoints) {
        slab[e] = ShellBatchFailureState::Constant();
      } else if (source->policy == ShellFailurePolicy::Tab1AnyPoint) {
        slab[e] = ShellBatchFailureState::Tab1();
      }
    }
    // Establish the selected union member before subsequent CUDA byte readback.
    staging_[e] = initial->state[0][e];
  }
  FailureDeviceStorage* candidate = nullptr;
  auto error = cudaMalloc(reinterpret_cast<void**>(&candidate), layout.bytes);
  if (error != cudaSuccess) {
    return {SetupStatus::DeviceFailure, "Failure sidecar device allocation failed", error};
  }
  const auto header = layout.Rebase(candidate);
  *initial = header;
  error = cudaMemcpy(candidate, arena.data(), layout.bytes, cudaMemcpyHostToDevice);
  if (error != cudaSuccess) {
    cudaFree(candidate);
    return {SetupStatus::DeviceFailure, "Failure sidecar initialization failed", error};
  }
  binding_ = binding;
  family_ = family;
  count_ = count;
  layout_ = layout;
  header_ = header;
  device_ = candidate;
  return {SetupStatus::Success, "OK"};
}

SetupReport FailureHostStorage::Read(unsigned slab, std::size_t count,
    cudaStream_t stream, double time, const ShellBatchLayeredSection* sections) noexcept {
  if (!device_ || slab > 1 || count != count_ || !sections) {
    return {SetupStatus::InvalidInput, "Invalid failure readback shape"};
  }
  // A rejected/injected read may have changed a staged tag. Restore the exact
  // active member before raw CUDA writes; do not rely on C++20 implicit lifetime.
  for (std::size_t e = 0; e < count; ++e) {
    const auto* source = binding_.parent(family_, e);
    if (!source) return {SetupStatus::InvalidInput, "Incomplete failure readback source"};
    if (source->policy == ShellFailurePolicy::ConstantAllPoints) {
      staging_[e] = ShellBatchFailureState::Constant();
    } else if (source->policy == ShellFailurePolicy::Tab1AnyPoint) {
      staging_[e] = ShellBatchFailureState::Tab1();
    } else {
      staging_[e] = ShellBatchFailureState{};
    }
  }
  auto error = cudaMemcpyAsync(staging_.data(), header_.state[slab],
      count * sizeof(ShellBatchFailureState), cudaMemcpyDeviceToHost, stream);
  if (error == cudaSuccess) error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) {
    return {SetupStatus::DeviceFailure, "Failure sidecar readback failed", error};
  }
  for (std::size_t e = 0; e < count; ++e) {
    const auto* source = binding_.parent(family_, e);
    if (!source || staging_[e].policy() != source->policy || !ValidFailureEncoding(staging_[e]) ||
        !ValidFailureState(staging_[e], source->policy, sections[e].plastic(), time)) {
      return {SetupStatus::NonfiniteResult,
              "Failure sidecar state disagrees with its declared policy/saved section"};
    }
  }
  return {SetupStatus::Success, "OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
