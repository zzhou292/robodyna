// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include <algorithm>

namespace tl::fea::nodal_detail {
cudaError_t CinStorage::Upload(cudaStream_t stream) {
  auto error = cudaMalloc(&arena, layout.device_bytes);
  if (error != cudaSuccess) return error;
  auto* bytes = static_cast<std::byte*>(arena);
  auto* device_rows = reinterpret_cast<constraints::tied_shell::cin::StageRow*>(bytes+layout.rows.offset);
  auto* device_dependent = reinterpret_cast<std::uint8_t*>(bytes+layout.dependent.offset);
  auto* device_first = reinterpret_cast<std::uint32_t*>(bytes+layout.first_witness.offset);
  activity = reinterpret_cast<std::uint8_t*>(bytes+layout.activity.offset);
  patches = reinterpret_cast<constraints::tied_shell::Patch*>(bytes+layout.patches.offset);
  work = reinterpret_cast<double*>(bytes+layout.work.offset);
  device = {device_rows, device_dependent, std::uint32_t(layout.nodes),
            std::uint32_t(layout.attachments), std::uint32_t(layout.witnesses), device_first};
  error = cudaMemcpyAsync(device_rows, rows.data(), layout.rows.bytes, cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  error = cudaMemcpyAsync(device_dependent, dependent.data(), layout.dependent.bytes, cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  error = cudaMemcpyAsync(device_first, first_witness.data(), layout.first_witness.bytes,
      cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  return ResetTrial(stream);
}
cudaError_t CinStorage::ResetTrial(cudaStream_t stream) {
  auto error = cudaMemsetAsync(activity, 0, layout.activity.bytes, stream);
  if (error != cudaSuccess) return error;
  return cudaMemsetAsync(work, 0, layout.work.bytes, stream);
}
} // namespace tl::fea::nodal_detail
