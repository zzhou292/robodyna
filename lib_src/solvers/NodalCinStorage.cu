// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include <algorithm>

namespace tl::fea::nodal_detail {
CinStorage::~CinStorage() { if (arena) cudaFree(arena); }
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
void CinStorage::InitializeState(double* state, const NodalCinStartup& input,
    const NodalDofConfig& dofs) const noexcept {
  auto* tail = state+state_offset;
  const auto n = layout.nodes;
  std::copy(input.mass, input.mass+n, tail);
  std::copy(input.inertia, input.inertia+n, tail+n);
  for (std::size_t i = 0; i < n; ++i) {
    tail[2*n+i] = dependent[i] || dofs.translation_fixed_bits[i] == 7 ? 0 : 1/input.mass[i];
    // Startup already validated this exact inverse, including dependent/fixed
    // and explicitly absent rotations. Do not reconstruct 1/0 for a solid node.
    tail[3*n+i] = dofs.inverse_inertia[i];
  }
  // INIEND initializes ILEV28 SMAS/SINER from literal secondary coefficients.
  // Later force stages update only a nonzero current coefficient.
  for (std::size_t r = 0; r < rows.size(); ++r) {
    tail[4*n+r] = input.mass[rows[r].secondary];
    tail[4*n+rows.size()+r] = input.inertia[rows[r].secondary];
  }
}
} // namespace tl::fea::nodal_detail
