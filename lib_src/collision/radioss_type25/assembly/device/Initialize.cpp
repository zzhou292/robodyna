// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <atomic>
#include <new>
namespace tlfea::contact::radioss_type25::assembly {
namespace {
std::atomic<std::uint64_t> next_identity{1};
std::uint64_t Identity() noexcept {
  auto value = next_identity.load(std::memory_order_relaxed);
  while (value && value != UINT64_MAX)
    if (next_identity.compare_exchange_weak(value,value+1,std::memory_order_relaxed)) return value;
  return 0;
}
}
DeviceIncidenceBuilder::DeviceIncidenceBuilder() = default;
DeviceIncidenceBuilder::~DeviceIncidenceBuilder() = default;
DeviceIncidenceBuilder::Impl::~Impl() {
  if (stream) cudaStreamSynchronize(stream);
  if (arena) cudaFree(arena);
}
IncidenceStatus DeviceIncidenceBuilder::Preflight(IncidenceLimits limits,
    IncidenceForecast& output) noexcept {
  auto status = device_detail::CheckLimits(limits);
  if (status != IncidenceStatus::Ok) return status;
  std::size_t bytes = 0;
  if (device_detail::QueryScratch(limits,bytes) != cudaSuccess) return IncidenceStatus::DeviceFailure;
  device_detail::Layout layout;
  status = device_detail::MakeLayout(limits,bytes,sizeof(DeviceIncidenceBuilder)+sizeof(Impl),layout);
  if (status == IncidenceStatus::Ok) output = layout.forecast;
  return status;
}
IncidenceStatus DeviceIncidenceBuilder::Initialize(IncidenceLimits limits,
    cudaStream_t stream) noexcept try {
  if (impl_) return IncidenceStatus::AlreadyInitialized;
  if (!stream || stream == cudaStreamLegacy || stream == cudaStreamPerThread)
    return IncidenceStatus::InvalidInput;
  auto status = device_detail::CheckLimits(limits);
  if (status != IncidenceStatus::Ok) return status;
  std::size_t bytes = 0;
  if (cudaGetLastError() != cudaSuccess || device_detail::QueryScratch(limits,bytes) != cudaSuccess)
    return IncidenceStatus::DeviceFailure;
  auto next = std::make_unique<Impl>();
  status = device_detail::MakeLayout(limits,bytes,sizeof(DeviceIncidenceBuilder)+sizeof(Impl),next->layout);
  if (status != IncidenceStatus::Ok) return status;
  next->identity = Identity();
  if (!next->identity) return IncidenceStatus::ResourceLimit;
  next->limits = limits; next->stream = stream;
  if (cudaMalloc(&next->arena,next->layout.forecast.device_bytes) != cudaSuccess)
    return IncidenceStatus::DeviceFailure;
  next->device = device_detail::Bind(next->arena,next->layout);
  impl_ = std::move(next);
  return IncidenceStatus::Ok;
} catch (const std::bad_alloc&) { return IncidenceStatus::ResourceLimit; }
} // namespace tlfea::contact::radioss_type25::assembly
