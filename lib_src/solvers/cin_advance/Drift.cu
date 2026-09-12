// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DriftValues.h"

namespace tl::fea::cin_advance::drift {
namespace {
__global__ void Begin(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  // The recovery phase has finished and its successful key is now dead.
  *input.recovery_failure = recovery::NoFailure;
}
__global__ void PrepareRows(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x) {
    const auto result = Prepare(input, row);
    input.prepared_drift[row] = result;
    if (result.status != NodalStatus::Ok) atomicMin(input.recovery_failure, row);
  }
}
__global__ void PublishRows(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto first_failure = *input.recovery_failure;
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x) {
    Publish(input, row, first_failure);
  }
}
__global__ void CompleteRows(Input input) {
  Complete(input);
}
} // namespace

cudaError_t Launch(const Input& input, cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  constexpr unsigned threads = 128;
  const auto blocks = 1+(input.model.row_count-1)/threads;
  PrepareRows<<<blocks, threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  PublishRows<<<blocks, threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CompleteRows<<<1, 1, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::drift
