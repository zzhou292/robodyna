// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Groups.h"
#include "group_motion/Motion.cuh"

namespace tl::fea::cin_advance::groups {
namespace {
__device__ void Fail(Input input, NodalStatus status, std::uint32_t node) {
  input.control->status = status;
  input.control->node = node;
  if (status == NodalStatus::StepTooLarge) input.control->limit.dt = 0;
}
__global__ void Begin(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto failure = *input.failure;
  if (failure != NoFailure) Fail(input, FailureStatus(failure), FailureNode(failure));
}
__global__ void Advance(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  __shared__ group_motion::Tile tile;
  const auto group=blockIdx.x;
  if (input.capture.node) group_motion::Advance<true>(input,group,tile);
  else group_motion::Advance<false>(input,group,tile);
}
__global__ void Complete(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  for (std::uint32_t group = 0; group < input.groups.group_count; ++group) {
    const auto& report = input.group_reports[group];
    if (report.status == NodalStatus::Ok) continue;
    Fail(input, report.status, report.last_node);
    return;
  }
}
} // namespace
cudaError_t LaunchMotion(const Input& input, cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Advance<<<input.groups.group_count, group_motion::Threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Complete<<<1, 1, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::groups
