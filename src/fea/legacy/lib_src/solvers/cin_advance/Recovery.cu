// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Recovery.h"
#include "RecoveryDrift.h"
#include "DriftValues.h"

namespace tl::fea::cin_advance::recovery {
namespace {
__global__ void Begin(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  if (!cin::detail::MotionPointersValid(input.model, MotionView(input))) {
    Fail(input, NodalStatus::InvalidOutput, UINT32_MAX);
    return;
  }
  *input.recovery_failure = NoFailure;
}
__global__ void PrepareRows(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x) {
    const auto result = Prepare(input, row);
    input.prepared_recovery[row] = result;
    if (!result.valid) atomicMin(input.recovery_failure, row);
  }
}
__global__ void PublishRows(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto first_failure = *input.recovery_failure;
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x)
    Publish(input, row, first_failure);
}
__global__ void Complete(Input input, bool defer_drift) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto failed = *input.recovery_failure;
  if (failed != NoFailure) {
    Fail(input, NodalStatus::InvalidOutput, input.model.rows[failed].secondary);
    return;
  }
  if (!defer_drift) Drift(input);
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
  const bool parallel_drift = input.prepared_drift != nullptr;
  Complete<<<1, 1, 0, stream>>>(input, parallel_drift);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  return parallel_drift ? drift::Launch(input, stream) : cudaSuccess;
}
} // namespace tl::fea::cin_advance::recovery
