// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include <cuda_runtime.h>
#include <cstring>
namespace tl::fea::cin_recovery_test {
namespace {
// Test-only guarded tail; the actual owner test exercises startup allocation.
template<class Launch> cudaError_t WithRecovery(const cin_advance::Input& original, cudaStream_t stream, Launch launch) {
  recovery::Row* storage = nullptr;
  const auto count = original.model.row_count;
  auto error = cudaMalloc(&storage, (count+2)*sizeof(recovery::Row));
  if (error != cudaSuccess) return error;
  recovery::FailureRow* failure = nullptr;
  const auto finish = [&](cudaError_t status) {
    if (failure) cudaFree(failure);
    const auto released = cudaFree(storage);
    return status == cudaSuccess ? released : status;
  };
  error = cudaMemsetAsync(storage, 0xa5, (count+2)*sizeof(recovery::Row), stream);
  if (error != cudaSuccess) return finish(error);
  auto input = original;
  input.prepared_recovery = storage+1;
  error = cudaMalloc(&failure, sizeof(*failure));
  if (error != cudaSuccess) return finish(error);
  input.recovery_failure = failure;
  error = cudaMemsetAsync(failure, 0xa5, sizeof(*failure), stream);
  if (error != cudaSuccess) return finish(error);
  error = launch(input, stream);
  if (error != cudaSuccess) return finish(error);
  error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) return finish(error);
  std::vector<unsigned char> bytes((count+2)*sizeof(recovery::Row));
  error = cudaMemcpy(bytes.data(), storage, bytes.size(), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  for (std::size_t index = 0; index < sizeof(recovery::Row); ++index) {
    EXPECT_EQ(bytes[index], 0xa5);
    EXPECT_EQ(bytes[(count+1)*sizeof(recovery::Row)+index], 0xa5);
  }
  recovery::FailureRow observed = 0;
  error = cudaMemcpy(&observed, failure, sizeof(observed), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  if (observed == 0xa5a5a5a5u) {
    for (std::size_t index = sizeof(recovery::Row); index < (count+1)*sizeof(recovery::Row); ++index)
      EXPECT_EQ(bytes[index], 0xa5);
  }
  nodal_detail::Control control;
  error = cudaMemcpy(&control, input.control, sizeof(control), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  if (control.status == NodalStatus::Ok) {
    for (unsigned row = 0; row < count; ++row) {
      recovery::Row value;
      std::memcpy(&value, bytes.data()+(row+1)*sizeof(value), sizeof(value));
      EXPECT_TRUE(value.valid);
    }
  }
  return finish(cudaSuccess);
}
} // namespace
} // namespace tl::fea::cin_recovery_test
