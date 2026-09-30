// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <gtest/gtest.h>
#include <cstddef>
#include <cuda_runtime.h>
namespace type25_friction_test {
struct Drain { cudaStream_t stream; ~Drain() { EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess); } };
template<std::size_t BytesPerRow=1024>
class PacketCuda : public ::testing::Test {
 protected:
  static constexpr std::size_t Capacity = 512, RowBytes = BytesPerRow;
  cudaStream_t stream = nullptr;
  void* input = nullptr; void* output = nullptr;
  void SetUp() override {
    ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    ASSERT_EQ(cudaMalloc(&input, Capacity * RowBytes), cudaSuccess);
    ASSERT_EQ(cudaMalloc(&output, Capacity * RowBytes), cudaSuccess);
  }
  void TearDown() override {
    if (stream) EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    if (output) EXPECT_EQ(cudaFree(output), cudaSuccess);
    if (input) EXPECT_EQ(cudaFree(input), cudaSuccess);
    if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  }
};
using FrictionCuda=PacketCuda<>;
} // namespace type25_friction_test
