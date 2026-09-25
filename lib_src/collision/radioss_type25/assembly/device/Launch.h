// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include <cuda_runtime.h>
namespace tlfea::contact::radioss_type25::assembly::device_detail {
struct Device {
  std::uint64_t* keys = nullptr;
  std::uint64_t* sorted_keys = nullptr;
  std::uint32_t* occurrences = nullptr;
  std::uint32_t* offsets = nullptr;
  unsigned long long* failure = nullptr;
  void* cub = nullptr;
  std::size_t cub_bytes = 0;
};
inline Device Bind(void* arena, const Layout& layout) noexcept {
  using tl::util::ArenaPointer;
  return {ArenaPointer<std::uint64_t>(arena,layout.keys),
      ArenaPointer<std::uint64_t>(arena,layout.sorted_keys),
      ArenaPointer<std::uint32_t>(arena,layout.occurrences),
      ArenaPointer<std::uint32_t>(arena,layout.offsets),
      ArenaPointer<unsigned long long>(arena,layout.failure),
      ArenaPointer<std::byte>(arena,layout.cub),layout.forecast.cub_bytes};
}
cudaError_t QueryScratch(IncidenceLimits, std::size_t&) noexcept;
cudaError_t Build(Device, const DeviceConnectivity&, cudaStream_t, IncidenceReport&) noexcept;
} // namespace tlfea::contact::radioss_type25::assembly::device_detail
