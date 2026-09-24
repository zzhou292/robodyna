// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <cuda_runtime.h>
#include <stdexcept>
namespace native_gpu_test {
struct Streams {
  cudaStream_t first = nullptr, other = nullptr;
  Streams() {
    if (cudaStreamCreateWithFlags(&first, cudaStreamNonBlocking) != cudaSuccess)
      throw std::runtime_error("Native GPU qualification stream allocation failed");
    if (cudaStreamCreateWithFlags(&other, cudaStreamNonBlocking) != cudaSuccess) {
      cudaStreamDestroy(first); first = nullptr;
      throw std::runtime_error("Native GPU qualification second stream allocation failed");
    }
  }
  ~Streams() {
    if (first) { EXPECT_EQ(cudaStreamSynchronize(first), cudaSuccess); EXPECT_EQ(cudaStreamDestroy(first), cudaSuccess); }
    if (other) { EXPECT_EQ(cudaStreamSynchronize(other), cudaSuccess); EXPECT_EQ(cudaStreamDestroy(other), cudaSuccess); }
  }
};
inline c::RepresentedIntervalGpuReport Compare(c::RepresentedIntervalCrossing& cpu,
    c::RepresentedIntervalCrossingGpu& gpu, const Cases& cases, cudaStream_t stream) {
  const auto original = cpu.Certify(cases.paths.data(), cases.paths.size(), cases.pairs.data(), cases.pairs.size());
  const auto current = gpu.Certify(cases.paths.data(), cases.paths.size(), cases.pairs.data(), cases.pairs.size(), stream);
  fixture::SameNativeReport(current.native, original);
  fixture::SameView(gpu.results(), cpu.results());
  return current;
}
}  // namespace native_gpu_test
