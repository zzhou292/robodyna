// SPDX-License-Identifier: AGPL-3.0-or-later
#include "JacobianNative.h"
#include "SourceFixture.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <vector>
namespace solid24_test {
struct GlobalDeviceResult {s::Reference reference; s::Status status;};
__global__ void PrepareGlobal(s::ReferenceInput input,GlobalDeviceResult* output) {
  output->status=s::InitializeReference(input,output->reference);
}
__global__ void PrepareGlobalBatch(const s::ReferenceInput* input,GlobalDeviceResult* output,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if (i<count) output[i].status=s::InitializeReference(input[i],output[i].reference);
}
TEST(Solid24JacobianCuda, DeviceWorldJacobianUnitFloorsAndLateRejectionRetry) {
  GlobalDeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);
  struct Release {GlobalDeviceResult* p;~Release(){cudaFree(p);}} release{device};
  ASSERT_EQ(cudaMemset(device,0,sizeof(*device)),cudaSuccess);
  for (const auto unit:{s::WorkingLengthUnit::Metre,s::WorkingLengthUnit::Millimetre}) {
    for (double factor:{.5,1.,2.}) {
      auto input=FloorControl(unit,factor);
      PrepareGlobal<<<1,1>>>(input,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      GlobalDeviceResult actual;
      ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(actual.status,s::Status::Success);
      const double scale=unit==s::WorkingLengthUnit::Metre?1.0:.001;
      for (auto& x:input.position_m) {x.x/=scale;x.y/=scale;x.z/=scale;}
      const auto native=GlobalNativeRaw(input); ASSERT_EQ(native.status,0);
      EXPECT_TRUE(JacobianAgree(JacobianValues(actual.reference),WorkingJacobianToSI(native.jacobian(),unit)));
    }
  }
  const auto good=TotalReference(Distorted(),s::WorkingLengthUnit::Millimetre);
  PrepareGlobal<<<1,1>>>(good,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  GlobalDeviceResult before,after;
  ASSERT_EQ(cudaMemcpy(&before,device,sizeof(before),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(before.status,s::Status::Success);
  auto bad=good;
  for (auto& x:bad.position_m) {x.x*=1e6;x.y*=1e6;x.z*=1e6;}
  bad.density_kg_m3=std::numeric_limits<double>::max();
  PrepareGlobal<<<1,1>>>(bad,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&after,device,sizeof(after),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(after.status,s::Status::NonfiniteResult);
  EXPECT_EQ(JacobianValues(before.reference),JacobianValues(after.reference));
  EXPECT_EQ(Values(before.reference),Values(after.reference));
  EXPECT_EQ(after.reference.input().density_kg_m3,good.density_kg_m3);
  PrepareGlobal<<<1,1>>>(good,device); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&after,device,sizeof(after),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(after.status,s::Status::Success);
  EXPECT_EQ(JacobianValues(before.reference),JacobianValues(after.reference));
}
TEST(Solid24JacobianCuda, All1309GlobalReferencesMatchNativeAndRetainOriginalSource) {
  std::vector<s::ReferenceInput> input;
  for (unsigned i=0;i<SourceCount;++i) {
    auto source=Source(i);
    if (IsBrick(source)) input.push_back(TotalReference(source,s::WorkingLengthUnit::Millimetre));
  }
  ASSERT_EQ(input.size(),1309u);
  s::ReferenceInput* source=nullptr; GlobalDeviceResult* device=nullptr;
  ASSERT_EQ(cudaMalloc(&source,input.size()*sizeof(*source)),cudaSuccess);
  struct ReleaseSource {s::ReferenceInput* p;~ReleaseSource(){cudaFree(p);}} source_release{source};
  ASSERT_EQ(cudaMalloc(&device,input.size()*sizeof(*device)),cudaSuccess);
  struct ReleaseOutput {GlobalDeviceResult* p;~ReleaseOutput(){cudaFree(p);}} output_release{device};
  ASSERT_EQ(cudaMemset(device,0,input.size()*sizeof(*device)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(source,input.data(),input.size()*sizeof(*source),cudaMemcpyHostToDevice),cudaSuccess);
  PrepareGlobalBatch<<<(input.size()+63)/64,64>>>(source,device,input.size());
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  std::vector<GlobalDeviceResult> actual(input.size());
  ASSERT_EQ(cudaMemcpy(actual.data(),device,actual.size()*sizeof(*device),cudaMemcpyDeviceToHost),cudaSuccess);
  for (unsigned i=0;i<input.size();++i) {
    SCOPED_TRACE(input[i].source_element_id);
    const auto native=GlobalNativeRaw(input[i]); ASSERT_EQ(native.status,0);
    ASSERT_EQ(actual[i].status,s::Status::Success);
    EXPECT_TRUE(JacobianAgree(JacobianValues(actual[i].reference),native.jacobian()));
    EXPECT_TRUE(Agree(Values(actual[i].reference),native.legacy()));
    for (unsigned n=0;n<8;++n) {
      EXPECT_EQ(actual[i].reference.source_slot(n),unsigned(native.permutation[n]));
      EXPECT_EQ(actual[i].reference.input().source_node_id[n],input[i].source_node_id[n]);
    }
  }
}
}  // namespace solid24_test
