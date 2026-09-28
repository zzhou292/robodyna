// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ForceGather.h"
#include "NumericalMassRead.cuh"

namespace tl::fea::cin_advance::force_gather {
namespace {
__global__ void Gather(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto view = input.force_gather;
  const auto force = force_inputs::ForceView(input);
  for (std::uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;
       index < view.master_count; index += blockDim.x * gridDim.x) {
    Master value;
    if (GatherMaster(input.model, force, input.prepared_transfers, view, index, value))
      view.values[index] = value;
    else
      atomicExch(&view.summary->mode, static_cast<unsigned>(Mode::NeedsSerial));
  }
}
__global__ void Complete(Input input) {
  if(input.control->status!=NodalStatus::Ok)return;
  __shared__ mass_read::Tile tile;
  auto& summary=*input.force_gather.summary;
  double numerical_mass=0;
  const bool ready=summary.mode==Mode::Staging && mass_read::Evaluate(input.model,
      force_inputs::ForceView(input),input.prepared_transfers,tile,numerical_mass);
  if(threadIdx.x)return;
  if(ready){summary.numerical_mass=numerical_mass;summary.report={};summary.mode=Mode::Publish;}
  else {
    // Staging has changed no original destinations. The existing serial apply
    // still owns fallback, first error and every partly published failing row.
    summary.report=force_transfers::Apply(input);summary.mode=Mode::SerialCompleted;
  }
}
__global__ void PublishDestinations(Input input) {
  if (input.control->status != NodalStatus::Ok || input.force_gather.summary->mode != Mode::Publish) return;
  const auto view = input.force_gather;
  const auto force = force_inputs::ForceView(input);
  const auto stride = blockDim.x * gridDim.x;
  const auto first = blockIdx.x * blockDim.x + threadIdx.x;
  for (std::uint32_t index = first; index < view.master_count; index += stride)
    PublishMaster(force, input.model.node_count, view.nodes[index], view.values[index]);
  for (std::uint32_t row = first; row < input.model.row_count; row += stride)
    PublishSecondary(input.model, force, input.prepared_transfers, row);
  if (!first) *force.numerical_mass = view.summary->numerical_mass;
}
} // namespace
cudaError_t Launch(const Input& input, cudaStream_t stream) {
  constexpr unsigned threads = 128;
  const auto count = input.force_gather.master_count > input.model.row_count
      ? input.force_gather.master_count : input.model.row_count;
  const auto blocks = 1 + (count - 1) / threads;
  Gather<<<blocks, threads, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Complete<<<1, mass_read::Threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  PublishDestinations<<<blocks, threads, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::force_gather
