#include "NativeOracle.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#ifdef REAR18_ORIGINAL
#include "OriginalFixture.h"
#endif

namespace rear18_test {
namespace {
struct Device {
  s::ReferenceInput* input = nullptr;
  law::Reference* output = nullptr;
  s::Status* status = nullptr;
  ~Device() { cudaFree(status); cudaFree(output); cudaFree(input); }
  bool Allocate(unsigned count) {
    return cudaMalloc(&input, count*sizeof(*input)) == cudaSuccess &&
           cudaMalloc(&output, count*sizeof(*output)) == cudaSuccess &&
           cudaMalloc(&status, count*sizeof(*status)) == cudaSuccess;
  }
};
__global__ void Prepare(const s::ReferenceInput* input, law::Reference* output,
                        s::Status* status, unsigned count) {
  const unsigned i = blockIdx.x*blockDim.x + threadIdx.x;
  if (i < count) status[i] = law::InitializeReference(input[i], output[i]);
}
__global__ void AliasedPrepare(law::Reference* output, s::Status* status) {
  const unsigned i = threadIdx.x;
  status[i] = law::InitializeReference(output[i].input(), output[i]);
}
void Download(Device& device, law::Reference* output, s::Status* status, unsigned count) {
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(output, device.output, count*sizeof(*output), cudaMemcpyDeviceToHost), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(status, device.status, count*sizeof(*status), cudaMemcpyDeviceToHost), cudaSuccess);
}
}
TEST(Rear18Cuda, NativeValuesRepeatedScatterAndAliasedInput) {
  const s::ReferenceInput input[]{Cube(), Collapsed()};
  Device device;
  ASSERT_TRUE(device.Allocate(2));
  ASSERT_EQ(cudaMemcpy(device.input, input, sizeof(input), cudaMemcpyHostToDevice), cudaSuccess);
  Prepare<<<1,2>>>(device.input, device.output, device.status, 2);
  law::Reference output[2]; s::Status status[2]{};
  ASSERT_NO_FATAL_FAILURE(Download(device, output, status, 2));
  for (unsigned i = 0; i < 2; ++i) {
    ASSERT_EQ(status[i], s::Status::Success);
    Compare(output[i], Native(input[i]));
  }
  const auto first = Values(output[0]), second = Values(output[1]);
  AliasedPrepare<<<1,2>>>(device.output, device.status);
  ASSERT_NO_FATAL_FAILURE(Download(device, output, status, 2));
  EXPECT_EQ(status[0], s::Status::Success);
  EXPECT_EQ(status[1], s::Status::Success);
  EXPECT_EQ(Values(output[0]), first);
  EXPECT_EQ(Values(output[1]), second);
}
TEST(Rear18Cuda, LateTopologyRejectionLeavesReferenceAndRetryUnchanged) {
  auto input = Collapsed();
  law::Reference accepted;
  ASSERT_EQ(law::InitializeReference(input, accepted), s::Status::Success);
  Device device;
  ASSERT_TRUE(device.Allocate(1));
  ASSERT_EQ(cudaMemcpy(device.output, &accepted, sizeof(accepted), cudaMemcpyHostToDevice), cudaSuccess);
  auto bad = input;
  bad.position_m[7].x = std::nextafter(bad.position_m[6].x, 2.0);
  ASSERT_EQ(cudaMemcpy(device.input, &bad, sizeof(bad), cudaMemcpyHostToDevice), cudaSuccess);
  Prepare<<<1,1>>>(device.input, device.output, device.status, 1);
  law::Reference output; s::Status status{};
  ASSERT_NO_FATAL_FAILURE(Download(device, &output, &status, 1));
  EXPECT_EQ(status, s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(output), solid18_test::Bytes(accepted));
  ASSERT_EQ(cudaMemcpy(device.input, &input, sizeof(input), cudaMemcpyHostToDevice), cudaSuccess);
  Prepare<<<1,1>>>(device.input, device.output, device.status, 1);
  ASSERT_NO_FATAL_FAILURE(Download(device, &output, &status, 1));
  ASSERT_EQ(status, s::Status::Success);
  Compare(output, Native(input));
}
#ifdef REAR18_ORIGINAL
TEST(Rear18CudaOriginal, EveryOriginalSourceCellKeepsNativeSlotsAndMass) {
  std::array<s::ReferenceInput,306> input;
  std::array<law::Reference,306> output;
  std::array<s::Status,306> status{};
  for (unsigned i = 0; i < 306; ++i) input[i] = original::Input(i);
  Device device;
  ASSERT_TRUE(device.Allocate(306));
  ASSERT_EQ(cudaMemcpy(device.input, input.data(), sizeof(input), cudaMemcpyHostToDevice), cudaSuccess);
  Prepare<<<3,128>>>(device.input, device.output, device.status, 306);
  ASSERT_NO_FATAL_FAILURE(Download(device, output.data(), status.data(), 306));
  unsigned repeated = 0;
  for (unsigned i = 0; i < 306; ++i) {
    SCOPED_TRACE(input[i].source_element_id);
    ASSERT_EQ(status[i], s::Status::Success);
    Compare(output[i], Native(input[i]));
    repeated += output[i].topology() == law::SourceTopology::RepeatedPairs56And78;
  }
  EXPECT_EQ(repeated, 109u);
}
#endif
}  // namespace rear18_test
