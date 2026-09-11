// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <vector>
namespace solid6z_test {
struct DeviceResult {s::Reference reference; s::Status status;};
__global__ void Prepare(s::ReferenceInput input,DeviceResult* out) {
  out->status=s::InitializeReference(input,out->reference);
}
__global__ void PrepareBatch(const s::ReferenceInput* inputs,DeviceResult* out,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i<count)out[i].status=s::InitializeReference(inputs[i],out[i].reference);
}
TEST(Solid6zCuda, ActualDeviceMatchesCompleteNativeStartup) {
  DeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(DeviceResult)),cudaSuccess);
  struct Release {DeviceResult* p;~Release(){cudaFree(p);}} release{device};
  ASSERT_EQ(cudaMemset(device,0,sizeof(DeviceResult)),cudaSuccess);
  for(const auto& input:{Wedge(),Distorted()}) {
    const auto native=Native(input);ASSERT_EQ(native.status,0);
    Prepare<<<1,1>>>(input,device);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    DeviceResult actual;
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.status,s::Status::Success);
    EXPECT_TRUE(Agree(Values(actual.reference),native.values));
    for(unsigned n=0;n<6;++n)EXPECT_EQ(actual.reference.source_slot(n),unsigned(native.permutation[n]));
  }
}
TEST(Solid6zCuda, EveryMappedOriginalWedgeMatchesNativeWithSourceSlots) {
  std::vector<s::ReferenceInput> inputs;
  for(unsigned i=0;i<SourceCount;++i) {s::ReferenceInput input;if(Source(i,input))inputs.push_back(input);}
  ASSERT_EQ(inputs.size(),195u);
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
    for(unsigned n=0;n<6;++n)ASSERT_EQ(actual[i].reference.source_slot(n),unsigned(native.permutation[n]));
  }
}
TEST(Solid6zCuda, LateMassFailurePreservesDeviceReferenceAndExactRetry) {
  DeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(DeviceResult)),cudaSuccess);
  struct Release {DeviceResult* p;~Release(){cudaFree(p);}} release{device};
  ASSERT_EQ(cudaMemset(device,0,sizeof(DeviceResult)),cudaSuccess);
  auto input=Distorted();
  Prepare<<<1,1>>>(input,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  DeviceResult before;
  ASSERT_EQ(cudaMemcpy(&before,device,sizeof(before),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(before.status,s::Status::Success);
  auto invalid=input;
  for(auto& x:invalid.position_m) {x.x*=1e6;x.y*=1e6;x.z*=1e6;}
  invalid.density_kg_m3=std::numeric_limits<double>::max();
  Prepare<<<1,1>>>(invalid,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  DeviceResult rejected;
  ASSERT_EQ(cudaMemcpy(&rejected,device,sizeof(rejected),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(rejected.status,s::Status::NonfiniteResult);
  EXPECT_EQ(Values(before.reference),Values(rejected.reference));
  EXPECT_EQ(rejected.reference.input().density_kg_m3,input.density_kg_m3);
  Prepare<<<1,1>>>(input,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  DeviceResult retry;
  ASSERT_EQ(cudaMemcpy(&retry,device,sizeof(retry),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(retry.status,s::Status::Success);
  EXPECT_EQ(Values(before.reference),Values(retry.reference));
  const auto native=Native(input); ASSERT_EQ(native.status,0);
  EXPECT_TRUE(Agree(Values(retry.reference),native.values));
}
}  // namespace solid6z_test
