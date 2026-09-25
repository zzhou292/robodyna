#include "PrimaryContextFixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
using namespace candidate_test;
namespace {
__global__ void Evaluate(const PrimaryCase* in,std::size_t n,c::FilterResult* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<n;i+=gridDim.x*blockDim.x)status[i]=c::EvaluatePacked(in[i].row,out+i);
}
struct Buffer {void* data=nullptr;~Buffer(){cudaDeviceSynchronize();if(data)cudaFree(data);}};
}
TEST(NativeCandidateCuda,CanonicalPrimaryContextEqualsEveryNativeWorkerThreshold) {
  const auto cases=PrimaryCases();Buffer a,b,cbuf;
  ASSERT_EQ(cudaMallocManaged(&a.data,cases.size()*sizeof(PrimaryCase)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&b.data,cases.size()*sizeof(c::FilterResult)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&cbuf.data,cases.size()*sizeof(c::Status)),cudaSuccess);
  auto* in=static_cast<PrimaryCase*>(a.data);auto* out=static_cast<c::FilterResult*>(b.data);auto* status=static_cast<c::Status*>(cbuf.data);
  std::copy(cases.begin(),cases.end(),in);Evaluate<<<32,128>>>(in,cases.size(),out,status);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  for(std::size_t i=0;i<cases.size();++i){SCOPED_TRACE(i);ASSERT_EQ(status[i],c::Status::Ok);
    for(int worker=1;worker<=cases[i].primary;++worker)for(int role:{0,cases[i].primary+1,3*cases[i].primary+1})
      ASSERT_EQ(Bits(out[i].squared_clearance),Bits(NativeThreshold(cases[i].row,worker,role)));}
}
