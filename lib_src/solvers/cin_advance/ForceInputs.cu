// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ForceInputs.h"

namespace tl::fea::cin_advance::force_inputs {
namespace {
__global__ void BeginInputs(Input input) {
  Begin(input);
}
__global__ void CheckNodes(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto force = ForceView(input);
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < input.model.node_count; node += gridDim.x*blockDim.x) {
    if (!cin::detail::CheckForceNode(input.model, force, node)) {
      atomicMin(input.input_failure, static_cast<FailureKey>(node));
    }
  }
}
__global__ void CompleteInputs(Input input) {
  Complete(input);
}
__global__ void CopyEntryInertia(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto force = ForceView(input);
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < input.model.node_count; node += gridDim.x*blockDim.x) {
    force.entry_inertia[node] = force.inertia[node];
  }
}
} // namespace

cudaError_t Launch(const Input& input, cudaStream_t stream) {
  BeginInputs<<<1, 1, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  constexpr unsigned threads = 128;
  const auto blocks = (input.model.node_count+threads-1)/threads;
  CheckNodes<<<blocks, threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CompleteInputs<<<1, 1, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CopyEntryInertia<<<blocks, threads, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::force_inputs
