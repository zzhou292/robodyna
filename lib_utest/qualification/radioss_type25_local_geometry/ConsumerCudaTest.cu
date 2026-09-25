// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
namespace type25_geometry_test {
namespace {
struct Packet { n::NativeGeometryInput input; n::NativeRawGeometryResult result; n::GeometryStatus status; };
__global__ void Run(Packet* p) {
  p->status=n::EvaluateNativeRawGeometry({1,1,5,1,false,false,false},p->input,&p->result);
}
struct Cleanup { Packet* data; ~Cleanup(){if(data)cudaFree(data);} };
TEST(Type25GeometryConsumerCuda, PublicHeaderExecutesWithoutFortranOrPrivatePrecisionFlags) {
  Packet actual;actual.input=Quad();Cleanup device{};
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.data),sizeof(Packet)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.data,&actual,sizeof(actual),cudaMemcpyHostToDevice),cudaSuccess);
  Run<<<1,1>>>(device.data);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&actual,device.data,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(actual.status,n::GeometryStatus::Ok);
  EXPECT_DOUBLE_EQ(actual.result.geometric_penetration,.4);
}
} // namespace
} // namespace type25_geometry_test
