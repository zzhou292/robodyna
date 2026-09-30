// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
#include <cuda_runtime.h>
namespace tied_search_test {
__global__ void Evaluate(const ts::SearchInput* in,ts::SearchChoice* out,ts::Status* status) {
  *status=ts::ConsiderCandidate(*in,7,*out);
}
TEST(TiedSearchCuda, ActualProjectionNativeParityAndRejectedAttemptRetry) {
  ts::SearchInput* input=nullptr;ts::SearchChoice* output=nullptr;ts::Status* status=nullptr;
  ASSERT_EQ(cudaMalloc(&input,sizeof(*input)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&output,sizeof(*output)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&status,sizeof(*status)),cudaSuccess);
  for(unsigned shape=0;shape<3;++shape) {
    const auto in=Shape(shape);ts::SearchChoice empty,actual;NativeChoice native;
    ASSERT_EQ(cudaMemcpy(input,&in,sizeof(in),cudaMemcpyHostToDevice),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(output,&empty,sizeof(empty),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(input,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ts::Status result;
    ASSERT_EQ(cudaMemcpy(&result,status,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(result,ts::Status::Success);
    ASSERT_EQ(cudaMemcpy(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    Compare(actual.projection,Native(in,7,native));EXPECT_EQ(actual.ordered_master,native.selected);
    std::array<unsigned char,sizeof(actual)> before{},after{};
    ASSERT_EQ(cudaMemcpy(before.data(),output,before.size(),cudaMemcpyDeviceToHost),cudaSuccess);
    auto bad=in;bad.geometry_m.master_position[3].z=std::numeric_limits<double>::quiet_NaN();
    ASSERT_EQ(cudaMemcpy(input,&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(input,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&result,status,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_NE(result,ts::Status::Success);
    ASSERT_EQ(cudaMemcpy(after.data(),output,after.size(),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(before,after);
    ASSERT_EQ(cudaMemcpy(input,&in,sizeof(in),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(input,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&result,status,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(result,ts::Status::Success);
    ASSERT_EQ(cudaMemcpy(after.data(),output,after.size(),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(before,after);
  }
  for(const auto& bad:InvalidFiniteInputs()) {
    ts::SearchChoice empty;
    ASSERT_EQ(cudaMemcpy(output,&empty,sizeof(empty),cudaMemcpyHostToDevice),cudaSuccess);
    std::array<unsigned char,sizeof(empty)> before{},after{};
    ASSERT_EQ(cudaMemcpy(before.data(),output,before.size(),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(input,&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(input,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ts::Status result;
    ASSERT_EQ(cudaMemcpy(&result,status,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_NE(result,ts::Status::Success);
    ASSERT_EQ(cudaMemcpy(after.data(),output,after.size(),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(before,after);
    auto retry=Shape();
    ASSERT_EQ(cudaMemcpy(input,&retry,sizeof(retry),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(input,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&result,status,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(result,ts::Status::Success);
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(output),cudaSuccess);EXPECT_EQ(cudaFree(input),cudaSuccess);
}
} // namespace tied_search_test
