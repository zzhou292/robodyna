#include "PackingFixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
using namespace candidate_test;
namespace {
__global__ void Pack(const c::LocalRow* in,std::size_t count,c::PackedRow* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    status[i]=c::PackLocal(in[i],out+i);
}
}
TEST(NativeCandidateCuda,ActualCor3tPackingAgainstWholeNative) {
  const auto rows=PackingCases();const auto count=rows.size();
  c::LocalRow* input=nullptr;c::PackedRow* output=nullptr;c::Status* status=nullptr;
  ASSERT_EQ(cudaMallocManaged(&input,count*sizeof(*input)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&output,count*sizeof(*output)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&status,count*sizeof(*status)),cudaSuccess);
  std::copy(rows.begin(),rows.end(),input);
  Pack<<<32,128>>>(input,count,output,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  for(std::size_t i=0;i<count;++i) {
    SCOPED_TRACE(i);ASSERT_EQ(status[i],c::Status::Ok);const auto native=NativePack(rows[i]);
    ASSERT_EQ(Bits(output[i].gap),Bits(native.gap));ASSERT_EQ(output[i].symmetry,native.symmetry);
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(output),cudaSuccess);EXPECT_EQ(cudaFree(input),cudaSuccess);
}
