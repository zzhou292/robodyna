// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../cin_parallel_recovery/DeviceFixture.cuh"
#include <cstring>

namespace tl::fea::cin_drift_test {
// Test-only guarded tail. Reuse the recovery key when that phase is present;
// the actual owner qualification verifies the single startup arena instead.
template<class Launch>
cudaError_t WithDrift(const cin_advance::Input& original, cudaStream_t stream,
    Launch launch, bool expect_untouched = false) {
  const auto count = original.model.row_count;
  drift::Row* storage = nullptr;
  auto error = cudaMalloc(&storage, (count+2)*sizeof(drift::Row));
  if (error != cudaSuccess) return error;
  recovery::FailureRow* owned_key = nullptr;
  const auto finish = [&](cudaError_t status) {
    if (owned_key) cudaFree(owned_key);
    const auto released = cudaFree(storage);
    return status == cudaSuccess ? released : status;
  };
  error = cudaMemsetAsync(storage, 0xa5, (count+2)*sizeof(drift::Row), stream);
  if (error != cudaSuccess) return finish(error);
  auto input = original;
  input.prepared_drift = storage+1;
  if (!input.recovery_failure) {
    error = cudaMalloc(&owned_key, sizeof(*owned_key));
    if (error != cudaSuccess) return finish(error);
    input.recovery_failure = owned_key;
    error = cudaMemsetAsync(owned_key, 0xa5, sizeof(*owned_key), stream);
    if (error != cudaSuccess) return finish(error);
  }
  error = launch(input, stream);
  if (error != cudaSuccess) return finish(error);
  error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) return finish(error);
  std::vector<unsigned char> bytes((count+2)*sizeof(drift::Row));
  error = cudaMemcpy(bytes.data(), storage, bytes.size(), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  for (std::size_t index = 0; index < sizeof(drift::Row); ++index) {
    EXPECT_EQ(bytes[index], 0xa5);
    EXPECT_EQ(bytes[(count+1)*sizeof(drift::Row)+index], 0xa5);
  }
  if (expect_untouched) {
    for (std::size_t index = sizeof(drift::Row); index < (count+1)*sizeof(drift::Row); ++index)
      EXPECT_EQ(bytes[index], 0xa5);
  }
  nodal_detail::Control control;
  error = cudaMemcpy(&control, input.control, sizeof(control), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  if (control.status == NodalStatus::Ok) {
    recovery::FailureRow failure = 0;
    error = cudaMemcpy(&failure, input.recovery_failure, sizeof(failure), cudaMemcpyDeviceToHost);
    if (error != cudaSuccess) return finish(error);
    EXPECT_EQ(failure, recovery::NoFailure);
    for (unsigned row = 0; row < count; ++row) {
      drift::Row value;
      std::memcpy(&value, bytes.data()+(row+1)*sizeof(value), sizeof(value));
      EXPECT_EQ(value.status, NodalStatus::Ok);
      EXPECT_EQ(value.stored_axes, 3u);
    }
  }
  return finish(cudaSuccess);
}
inline cudaError_t Parallel(const cin_advance::Input& input, cudaStream_t stream) {
  return cin_recovery_test::WithRecovery(input, stream,
      [](const cin_advance::Input& source, cudaStream_t s) {
        return WithDrift(source, s, cin_advance::Launch);
      });
}
} // namespace tl::fea::cin_drift_test
