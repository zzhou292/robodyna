// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ForceTransfers.h"

namespace tl::fea::cin_advance::force_transfers {
namespace {
__global__ void Prepare(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  if (!blockIdx.x && !threadIdx.x && force_gather::Eligible(input.model, input.force_gather))
    *input.force_gather.summary = {};
  const auto force = force_inputs::ForceView(input);
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x) {
    Row next;
    next.report = cin::detail::PrepareForceRow(input.model, force, row, next);
    input.prepared_transfers[row] = next;
  }
}
} // namespace
cudaError_t Launch(const Input& input, cudaStream_t stream) {
  if (!input.model.row_count) return cudaSuccess;
  constexpr unsigned threads = 128;
  const auto blocks = 1+(input.model.row_count-1)/threads;
  Prepare<<<blocks, threads, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::force_transfers
