// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cuda_runtime.h>
#include <vector>
#ifdef BEAM18_ORIGINAL
#include "OriginalFixture.h"
#endif
namespace beam18_test {
__global__ void ReferenceKernel(const beam::Input* input,beam::Reference* output,int* status,unsigned count) {
  for(unsigned i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    status[i]=static_cast<int>(beam::InitializeReference(input[i],output[i]));
}
void DeviceValues(const std::vector<beam::Input>& inputs,bool reject) {
  const auto count=static_cast<unsigned>(inputs.size());
  beam::Input* in=nullptr;beam::Reference* output=nullptr;int* status=nullptr;
  ASSERT_EQ(cudaMalloc(&in,count*sizeof(*in)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&output,count*sizeof(*output)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&status,count*sizeof(*status)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(in,inputs.data(),count*sizeof(*in),cudaMemcpyHostToDevice),cudaSuccess);
  ReferenceKernel<<<2,32>>>(in,output,status,count);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  std::vector<beam::Reference> accepted(count),retried(count);std::vector<int> results(count);
  ASSERT_EQ(cudaMemcpy(accepted.data(),output,count*sizeof(*output),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(results.data(),status,count*sizeof(*status),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<count;++i) {
    ASSERT_EQ(results[i],static_cast<int>(beam::Status::Success));
    Compare(accepted[i],Native(inputs[i]));
  }
  if(reject) {
    auto bad=inputs;
    for(auto& row:bad)row.density=1.7976931348623157e308;
    ASSERT_EQ(cudaMemcpy(in,bad.data(),count*sizeof(*in),cudaMemcpyHostToDevice),cudaSuccess);
    ReferenceKernel<<<2,32>>>(in,output,status,count);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(retried.data(),output,count*sizeof(*output),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(results.data(),status,count*sizeof(*status),cudaMemcpyDeviceToHost),cudaSuccess);
    for(unsigned i=0;i<count;++i) {
      EXPECT_EQ(results[i],static_cast<int>(beam::Status::NonfiniteResult));
      EXPECT_EQ(Bytes(retried[i]),Bytes(accepted[i]));
    }
    ASSERT_EQ(cudaMemcpy(in,inputs.data(),count*sizeof(*in),cudaMemcpyHostToDevice),cudaSuccess);
    ReferenceKernel<<<2,32>>>(in,output,status,count);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(retried.data(),output,count*sizeof(*output),cudaMemcpyDeviceToHost),cudaSuccess);
    for(unsigned i=0;i<count;++i)EXPECT_EQ(Values(retried[i]),Values(accepted[i]));
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(output),cudaSuccess);EXPECT_EQ(cudaFree(in),cudaSuccess);
}
TEST(Beam18Cuda, NativeSectionMassBranchesAndAtomicRetry) {
  DeviceValues({Input(.01),Input(4),Input(16),Input(20,5.9)},true);
}
#ifdef BEAM18_ORIGINAL
TEST(Beam18CudaSource, All142OriginalReferencesMatchIndependentNative) {
  std::vector<beam::Input> input;
  for(unsigned i=0;i<std::size(original::Cells);++i)input.push_back(original::Input(i));
  DeviceValues(input,false);
}
#endif
} // namespace beam18_test
