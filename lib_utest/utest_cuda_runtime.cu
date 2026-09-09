#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cstdio>
#include <vector>

namespace {
__global__ void AffineTransform(const double* input, double* output, int count) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) {
    output[i] = 2.0 * input[i] + 1.0;
  }
}

TEST(CudaRuntime, DeviceAllocationKernelAndReadback) {
  int count = 0;
  ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess)
      << "GPU unavailable: this is an execution failure, not a passing skip";
  ASSERT_GT(count, 0);
  cudaDeviceProp properties{};
  ASSERT_EQ(cudaGetDeviceProperties(&properties, 0), cudaSuccess);
  size_t free_bytes = 0;
  size_t total_bytes = 0;
  ASSERT_EQ(cudaMemGetInfo(&free_bytes, &total_bytes), cudaSuccess);
  std::printf("device=%s compute=%d.%d free_bytes=%zu total_bytes=%zu\n",
              properties.name, properties.major, properties.minor,
              free_bytes, total_bytes);
  ASSERT_GT(free_bytes, size_t{8} * 1024 * 1024 * 1024)
      << "Preserve workstation GPU memory reserve";

  constexpr int kCount = 1024;
  std::vector<double> input(kCount), result(kCount);
  for (int i = 0; i < kCount; ++i) {
    input[i] = i * 0.25;
  }
  double* d_input = nullptr;
  double* d_output = nullptr;
  ASSERT_EQ(cudaMalloc(&d_input, kCount * sizeof(double)), cudaSuccess);
  const auto second_allocation = cudaMalloc(&d_output, kCount * sizeof(double));
  if (second_allocation != cudaSuccess) {
    cudaFree(d_input);
    FAIL() << cudaGetErrorString(second_allocation);
  }
  EXPECT_EQ(cudaMemcpy(d_input, input.data(), kCount * sizeof(double),
                       cudaMemcpyHostToDevice), cudaSuccess);
  AffineTransform<<<4, 256>>>(d_input, d_output, kCount);
  EXPECT_EQ(cudaGetLastError(), cudaSuccess);
  EXPECT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  EXPECT_EQ(cudaMemcpy(result.data(), d_output, kCount * sizeof(double),
                       cudaMemcpyDeviceToHost), cudaSuccess);
  EXPECT_EQ(cudaFree(d_output), cudaSuccess);
  EXPECT_EQ(cudaFree(d_input), cudaSuccess);
  for (int i = 0; i < kCount; ++i) {
    EXPECT_DOUBLE_EQ(result[i], i * 0.5 + 1.0);
  }
}
}  // namespace
