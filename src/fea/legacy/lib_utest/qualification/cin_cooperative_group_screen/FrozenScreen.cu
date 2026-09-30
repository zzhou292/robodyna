// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FrozenSummary.h"
#include "FrozenGroups.h"
#include "FrozenCapture.h"

namespace tl::fea::cooperative_test::frozen_screen {
using cin_advance::NoFailure;
namespace {
__global__ void Begin(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  if (!frozen::CheckSources(Sources(input), input.structural.factor)) {
    input.control->status = NodalStatus::InvalidOutput;
    input.control->node = UINT32_MAX;
  }
}
__device__ void Reduce(Summary* values) {
  for (unsigned offset = Threads/2; offset; offset /= 2) {
    __syncthreads();
    if (threadIdx.x < offset) frozen_screen::Merge(values[threadIdx.x], values[threadIdx.x+offset]);
  }
  __syncthreads();
}
__global__ void Nodes(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  __shared__ Summary values[Threads];
  auto local = Empty();
  const auto source = Sources(input);
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < source.nodes; node += gridDim.x*blockDim.x) {
    frozen_screen::Observe(source, input.structural.factor, node, local);
  }
  values[threadIdx.x] = local;
  Reduce(values);
  if (!threadIdx.x) input.screen[blockIdx.x] = values[0];
}
__device__ void Publish(Input input, const Summary& ordinary, bool parallel_groups) {
  cin_timestep::Result result;
  std::uint32_t invalid_node = UINT32_MAX;
  const auto source = Sources(input);
  const bool valid = parallel_groups
      ? frozen_groups::CompleteScreen(source, ordinary, input.group_reports, result, invalid_node)
      : frozen_screen::Complete(source, input.structural.factor, ordinary, result, invalid_node);
  if (!valid) {
    input.control->status = NodalStatus::InvalidOutput;
    input.control->node = invalid_node;
    return;
  }
  input.control->limit.dt = result.minimum_dt;
  if (input.structural.capture_limiter)
    input.control->structural_limiter = frozen_limiter::Capture(source, input.structural.factor,
        result, input.epoch, input.attempt);
  if (input.durations.drift_dt > result.minimum_dt) {
    input.control->status = NodalStatus::StepTooLarge;
    input.control->node = result.limiting_node;
    return;
  }
  *input.failure = NoFailure;
}
__global__ void Finish(Input input, bool parallel_groups) {
  if (input.control->status != NodalStatus::Ok) return;
  __shared__ Summary values[Threads];
  auto local = Empty();
  const auto blocks = Blocks(input.model.node_count);
  for (unsigned block = threadIdx.x; block < blocks; block += blockDim.x)
    frozen_screen::Merge(local, input.screen[block]);
  values[threadIdx.x] = local;
  Reduce(values);
  if (threadIdx.x) return;
  // Retained private first record makes the executed reduction observable in
  // owning CUDA packets; it is not a public clock or mechanics authority.
  input.screen[0] = values[0];
  if (!parallel_groups) Publish(input, values[0], false);
}
__global__ void EvaluateGroups(Input input) {
  if (input.control->status != NodalStatus::Ok || input.screen[0].invalid_node != UINT32_MAX) return;
  const auto source = Sources(input);
  for (std::uint32_t group = blockIdx.x*blockDim.x+threadIdx.x;
       group < input.groups.group_count; group += gridDim.x*blockDim.x) {
    input.group_reports[group] = frozen_groups::ScreenGroup(source, input.structural.factor, group);
  }
}
__global__ void FinishGroups(Input input) {
  if (input.control->status == NodalStatus::Ok) Publish(input, input.screen[0], true);
}
} // namespace
cudaError_t Launch(const Input& input, cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Nodes<<<Blocks(input.model.node_count), Threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  const bool parallel_groups = input.group_reports && input.groups.group_count;
  Finish<<<1, Threads, 0, stream>>>(input, parallel_groups);
  error = cudaGetLastError();
  if (error != cudaSuccess || !parallel_groups) return error;
  const auto blocks = 1+(input.groups.group_count-1)/frozen_groups::Threads;
  EvaluateGroups<<<blocks, frozen_groups::Threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  FinishGroups<<<1, 1, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::screen
