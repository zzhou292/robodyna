// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PrecisionProbe.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <cmath>
namespace {
__global__ void Precision(const double* input, double* output) {
  output[0] = type25_precision_test::ProductSum(input[0], input[1], input[2]);
  output[1] = ::fma(input[0], input[1], input[2]);
}
struct Memory {
  double* input = nullptr; double* output = nullptr; cudaStream_t stream = nullptr;
  ~Memory() {
    if (stream) EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    if (output) EXPECT_EQ(cudaFree(output), cudaSuccess);
    if (input) EXPECT_EQ(cudaFree(input), cudaSuccess);
    if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  }
};
TEST(Type25FrictionConsumerCuda, NvccConsumerInheritsPrecisionWithoutPrivateFlags) {
  double input[]{1. + std::ldexp(1., -27), 1. - std::ldexp(1., -27), -1.}, output[2]{};
  Memory memory; // Declared after borrowed host buffers; drains before their lifetime ends.
  ASSERT_EQ(cudaStreamCreateWithFlags(&memory.stream, cudaStreamNonBlocking), cudaSuccess);
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&memory.input), sizeof(input)), cudaSuccess);
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&memory.output), sizeof(output)), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(memory.input, input, sizeof(input), cudaMemcpyHostToDevice, memory.stream), cudaSuccess);
  Precision<<<1, 1, 0, memory.stream>>>(memory.input, memory.output);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(output, memory.output, sizeof(output), cudaMemcpyDeviceToHost, memory.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(memory.stream), cudaSuccess);
  EXPECT_EQ(output[0], 0.); EXPECT_EQ(output[1], -std::ldexp(1., -54));
}
}
