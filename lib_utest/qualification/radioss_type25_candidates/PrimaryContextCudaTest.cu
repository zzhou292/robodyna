#include "PrimaryContextFixture.h"
#include "ObservedPrimaryContext.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
using namespace candidate_test;
namespace {
__global__ void Evaluate(const PrimaryCase* in,std::size_t n,c::FilterResult* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<n;i+=gridDim.x*blockDim.x)status[i]=c::EvaluatePacked(in[i].row,out+i);
}
__global__ void Observe(const ObservedPrimaryRow* in,std::size_t n,c::FilterResult* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<n;i+=gridDim.x*blockDim.x)status[i]=c::EvaluateLocal(in[i].row,out+i);
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

TEST(NativeCandidateCuda,ActualObservedPrimaryChunksUseDefinedNativeOperands) {
  const auto rows=ObservedPrimaryRows();Buffer a,b,status_buffer;
  ASSERT_EQ(cudaMallocManaged(&a.data,rows.size()*sizeof(ObservedPrimaryRow)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&b.data,rows.size()*sizeof(c::FilterResult)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&status_buffer.data,rows.size()*sizeof(c::Status)),cudaSuccess);
  auto* input=static_cast<ObservedPrimaryRow*>(a.data);auto* out=static_cast<c::FilterResult*>(b.data);
  auto* status=static_cast<c::Status*>(status_buffer.data);std::copy(rows.begin(),rows.end(),input);
  Observe<<<4,64>>>(input,rows.size(),out,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  for(std::size_t i=0;i<rows.size();++i) {
    ASSERT_EQ(status[i],c::Status::Ok);const auto& row=rows[i].row;
    if(!NativeScreen(row.screen)){EXPECT_FALSE(out[i].included);continue;}
    c::PackedRow native;for(unsigned j=0;j<4;++j){native.nodes[j]=row.nodes[j];native.vertices[j]=row.screen.vertices[j];}
    native.secondary=row.screen.secondary;native.margin=row.screen.margin;
    const auto packed=NativePack(row);native.gap=packed.gap;native.symmetry=packed.symmetry;
    const double expected=NativeThreshold(native,rows[i].worker_count,rows[i].unshifted_role);
    EXPECT_EQ(Bits(out[i].squared_clearance),Bits(expected));EXPECT_EQ(out[i].included,expected!=0.);
  }
}
