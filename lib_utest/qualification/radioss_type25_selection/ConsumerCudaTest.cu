// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
namespace type25_selection_test {
namespace {
struct Packet {s::NativePairInput input;n::NativeGeometryHistory prior;s::NativeRetainedResult result;s::Status status;};
__global__ void EvaluateRetainedHeader(Packet* p) {
  p->status=s::EvaluateNativeRetained({1,5,1,false,false,false},p->input,p->prior,&p->result);
}
struct Cleanup {Packet* value=nullptr;~Cleanup(){if(value)cudaFree(value);}};
TEST(Type25SelectionConsumerCuda, NativeHeaderExecutesWithoutOracleDependency) {
  const auto c=Basic();Packet actual;actual.input=c.input;actual.prior=c.prior;Cleanup device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.value),sizeof(Packet)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.value,&actual,sizeof(actual),cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateRetainedHeader<<<1,1>>>(device.value);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&actual,device.value,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(actual.status,s::Status::Ok);EXPECT_TRUE(actual.result.active);
  EXPECT_DOUBLE_EQ(actual.result.classification_product,800.);
}
} // namespace
} // namespace type25_selection_test
