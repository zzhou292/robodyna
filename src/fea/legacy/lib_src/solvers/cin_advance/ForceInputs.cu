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
__global__ void CompleteNodeInputs(Input input) { CompleteNodes(input); }
__global__ void CheckWitnesses(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto force = ForceView(input);
  for (std::uint32_t witness = blockIdx.x*blockDim.x+threadIdx.x;
       witness < input.model.witness_count; witness += gridDim.x*blockDim.x) {
    if (!cin::detail::CheckForceWitness(input.model, force, witness))
      atomicMin(input.input_failure, static_cast<FailureKey>(witness));
  }
}
__global__ void CompleteWitnessInputs(Input input) { CompleteWitnesses(input); }
__global__ void CheckRows(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  const auto force = ForceView(input);
  for (std::uint32_t row = blockIdx.x*blockDim.x+threadIdx.x;
       row < input.model.row_count; row += gridDim.x*blockDim.x) {
    if (!cin::detail::CheckForceRow(input.model, force, row))
      atomicMin(input.input_failure, static_cast<FailureKey>(row));
  }
}
__global__ void CompleteInputs(Input input) { CompleteRows(input); }
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
  CompleteNodeInputs<<<1, 1, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  if (input.model.witness_count) {
    const auto witness_blocks = 1+(input.model.witness_count-1)/threads;
    CheckWitnesses<<<witness_blocks, threads, 0, stream>>>(input);
    error = cudaGetLastError();
    if (error != cudaSuccess) return error;
  }
  CompleteWitnessInputs<<<1, 1, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  if (input.model.row_count) {
    const auto row_blocks = 1+(input.model.row_count-1)/threads;
    CheckRows<<<row_blocks, threads, 0, stream>>>(input);
    error = cudaGetLastError();
    if (error != cudaSuccess) return error;
  }
  CompleteInputs<<<1, 1, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CopyEntryInertia<<<blocks, threads, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::force_inputs
