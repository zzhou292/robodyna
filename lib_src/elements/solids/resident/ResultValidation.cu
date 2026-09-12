// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ResultValidation.h"

namespace tl::fea::solids::batch_detail {
namespace {
template<class Traits> __global__ void ValidateResults(Storage* storage, unsigned trial,
    double time, std::uint64_t epoch) {
  auto& family = FamilyStorage<Traits>(*storage);
  const auto first = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto stride = std::size_t(gridDim.x) * blockDim.x;
  for (std::size_t parent = first; parent < family.count; parent += stride) {
    // Every active row is overwritten on every operation, including errors.
    family.result_valid[parent] = CheckParentResult<Traits>(*storage, trial, parent, time, epoch);
  }
}
} // namespace

void LaunchResultValidation(Storage* storage, unsigned trial, double time,
    std::uint64_t epoch, cudaStream_t stream) {
  ValidateResults<Traits18><<<candidate_blocks, candidate_threads, 0, stream>>>(storage, trial, time, epoch);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  ValidateResults<Traits24><<<candidate_blocks, candidate_threads, 0, stream>>>(storage, trial, time, epoch);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  ValidateResults<Traits6z><<<candidate_blocks, candidate_threads, 0, stream>>>(storage, trial, time, epoch);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  ValidateResults<Traits18Law44><<<candidate_blocks, candidate_threads, 0, stream>>>(storage, trial, time, epoch);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  ValidateResults<Traits18Law90><<<candidate_blocks, candidate_threads, 0, stream>>>(storage, trial, time, epoch);
}
} // namespace tl::fea::solids::batch_detail
