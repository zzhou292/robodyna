// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Capture.h"

namespace tl::fea::cin_advance::capture {
namespace {
__global__ void Nodes(const Input input) {
  // Earlier failure may leave partial rigid capture. Do not touch any row until
  // rigid candidates, recovery and every secondary drift/orientation succeed.
  if (input.control->status != NodalStatus::Ok || !input.capture.node) return;
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < input.model.node_count; node += blockDim.x*gridDim.x) {
    CopyNode(input, node);
  }
}
} // namespace

cudaError_t Launch(const Input& input, cudaStream_t stream) {
  if (!input.capture.node) return cudaSuccess;
  const auto blocks = Blocks(input.model.node_count);
  if (!blocks) return cudaSuccess;
  Nodes<<<blocks,Threads,0,stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::capture
