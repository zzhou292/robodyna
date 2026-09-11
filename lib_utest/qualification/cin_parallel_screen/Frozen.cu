// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FrozenCaller.inc"
namespace cin_screen_test {
using namespace cin_advance;
cudaError_t LaunchFrozen(const cin_advance::Input& input, cudaStream_t stream) {
  const bool parallel_inputs = input.input_failure != nullptr;
  auto error = cudaSuccess;
  if (parallel_inputs) {
    error = force_inputs::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  PrepareCin<<<1,1,0,stream>>>(input, parallel_inputs);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  constexpr unsigned threads = 128;
  const auto blocks = (input.model.node_count+threads-1)/threads;
  AdvanceOrdinaryCin<<<blocks,threads,0,stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CompleteCin<<<1,1,0,stream>>>(input);
  return cudaGetLastError();
}

} // namespace cin_screen_test
} // namespace tl::fea (opened by the complete frozen caller)
