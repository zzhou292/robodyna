// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <vector>
namespace solid24_test {
struct DeviceResult {s::Reference reference; s::Status status;};
__global__ void Prepare(s::ReferenceInput input,DeviceResult* out) {
  out->status=s::InitializeReference(input,out->reference);
}
__global__ void PrepareBatch(const s::ReferenceInput* inputs,DeviceResult* out,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i<count)out[i].status=s::InitializeReference(inputs[i],out[i].reference);
}
TEST(Solid24Cuda, ActualDeviceMatchesCompleteNativeStartup) {
  DeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(DeviceResult)),cudaSuccess);
  struct Release {DeviceResult* p;~Release(){cudaFree(p);}} release{device};
  for(const auto& input:{Brick(),Distorted()}) {
    const auto native=Native(input);ASSERT_EQ(native.status,0);
    Prepare<<<1,1>>>(input,device);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    DeviceResult actual;
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.status,s::Status::Success);
    EXPECT_TRUE(Agree(Values(actual.reference),native.values));
    for(unsigned n=0;n<8;++n)EXPECT_EQ(actual.reference.source_slot(n),unsigned(native.permutation[n]));
  }
}
TEST(Solid24Cuda, EveryOriginalBrickMatchesNativeWithSourceSlots) {
  std::vector<s::ReferenceInput> inputs;
  for(unsigned i=0;i<SourceCount;++i) {auto input=Source(i);if(IsBrick(input))inputs.push_back(input);}
  ASSERT_EQ(inputs.size(),1309u);
  s::ReferenceInput* source=nullptr;DeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&source,inputs.size()*sizeof(*source)),cudaSuccess);
  struct ReleaseInput {s::ReferenceInput* p;~ReleaseInput(){cudaFree(p);}} source_release{source};
  ASSERT_EQ(cudaMalloc(&device,inputs.size()*sizeof(*device)),cudaSuccess);
  struct ReleaseOutput {DeviceResult* p;~ReleaseOutput(){cudaFree(p);}} output_release{device};
  ASSERT_EQ(cudaMemset(device,0,inputs.size()*sizeof(*device)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(source,inputs.data(),inputs.size()*sizeof(*source),cudaMemcpyHostToDevice),cudaSuccess);
  PrepareBatch<<<(inputs.size()+63)/64,64>>>(source,device,inputs.size());
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  std::vector<DeviceResult> actual(inputs.size());
  ASSERT_EQ(cudaMemcpy(actual.data(),device,actual.size()*sizeof(*device),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<inputs.size();++i) {
    SCOPED_TRACE(inputs[i].source_element_id);
    const auto native=Native(inputs[i]);ASSERT_EQ(native.status,0);
    ASSERT_EQ(actual[i].status,s::Status::Success);
    ASSERT_TRUE(Agree(Values(actual[i].reference),native.values));
    for(unsigned n=0;n<8;++n)ASSERT_EQ(actual[i].reference.source_slot(n),unsigned(native.permutation[n]));
  }
}
}  // namespace solid24_test
