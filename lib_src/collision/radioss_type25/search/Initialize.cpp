// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Launch.h"
#include <atomic>
#include <new>
namespace tlfea::contact::radioss_type25::search {
namespace {
// Same bounded identity pattern as SelfContactForceAssembly. Cache tokens only;
// this sequence is not a mechanics clock, owner ID or physical receipt.
std::atomic<std::uint64_t> next_identity{1};
std::uint64_t Identity() noexcept {
  auto value = next_identity.load(std::memory_order_relaxed);
  while (value && value != UINT64_MAX) {
    if (next_identity.compare_exchange_weak(value, value + 1, std::memory_order_relaxed))
      return value;
  }
  return 0;
}
bool Explicit(cudaStream_t stream) noexcept {
  return stream && stream != cudaStreamLegacy && stream != cudaStreamPerThread;
}
}
Maintenance::Maintenance() = default;
Maintenance::~Maintenance() = default;
Maintenance::Impl::~Impl() {
  if (stream) cudaStreamSynchronize(stream);
  if (arena) cudaFree(arena);
}
Status Maintenance::Initialize(const Source& source, Limits limits,
    cudaStream_t stream) noexcept try {
  if (impl_) return Status::AlreadyInitialized;
  if (!Explicit(stream)) return Status::InvalidInput;
  detail::Layout layout;
  const auto status = detail::MakeLayout(source, limits, sizeof(Maintenance) + sizeof(Impl), layout);
  if (status != Status::Ok) return status;
  auto next = std::make_unique<Impl>();
  next->identity = Identity();
  if (!next->identity) return Status::ResourceLimit;
  next->source = source;
  next->limits = limits;
  next->layout = layout;
  next->stream = stream;
  if (!units_detail::Make(source.units, next->factors)) return Status::InvalidInput;
  tl::util::HostArena upload;
  if (!upload.Initialize(layout.roles.bytes)) return Status::ResourceLimit;
  const tl::util::ArenaRegion region{0, layout.roles.count, layout.roles.bytes};
  auto* roles = upload.Construct<std::uint32_t>(region);
  if (!roles) return Status::ResourceLimit;
  std::size_t k = 0;
  for (std::size_t i = 0; i < source.secondaries; ++i) roles[k++] = source.secondary_nodes[i];
  for (std::size_t i = 0; i < source.mains; ++i) roles[k++] = source.main_nodes[i];
  for (std::size_t i = 0; i < source.main_1d; ++i) roles[k++] = source.main_1d_nodes[i];
  auto error = cudaMalloc(&next->arena, layout.forecast.device_bytes);
  if (error != cudaSuccess) return Status::DeviceFailure;
  next->device = detail::Bind(next->arena, layout, source, next->factors);
  error = cudaMemcpyAsync(next->device.roles, roles, layout.roles.bytes, cudaMemcpyHostToDevice, stream);
  const auto drained = cudaStreamSynchronize(stream);
  if (error != cudaSuccess || drained != cudaSuccess) return Status::DeviceFailure;
  // Startup maps are owned on device; do not retain borrowed host addresses.
  next->source.secondary_nodes = nullptr;
  next->source.main_nodes = nullptr;
  next->source.main_1d_nodes = nullptr;
  impl_ = std::move(next);
  return Status::Ok;
} catch (const std::bad_alloc&) {
  return Status::ResourceLimit;
}
} // namespace tlfea::contact::radioss_type25::search
