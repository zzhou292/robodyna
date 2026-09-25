#include "Fixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
using namespace candidate_test;
namespace {
__global__ void Evaluate(const c::PackedRow* rows,std::size_t count,c::FilterResult* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    status[i]=c::EvaluatePacked(rows[i],out+i);
}
__global__ void Screens(const c::ScreenRow* rows,std::size_t count,bool* out,c::Status* status) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    status[i]=c::EvaluateScreen(rows[i],out+i);
}
}
TEST(NativeCandidateCuda,ActualDevicePackedNativeRows) {
  const auto cases=Cases();const auto count=cases.size();
  c::PackedRow* rows=nullptr;c::FilterResult* out=nullptr;c::Status* status=nullptr;
  ASSERT_EQ(cudaMallocManaged(&rows,count*sizeof(*rows)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&out,count*sizeof(*out)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&status,count*sizeof(*status)),cudaSuccess);
  for(std::size_t i=0;i<count;++i)rows[i]=Pack(cases[i]);
  Evaluate<<<32,128>>>(rows,count,out,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  for(std::size_t i=0;i<count;++i) {
    SCOPED_TRACE(i);ASSERT_EQ(status[i],c::Status::Ok);
    const auto native=pen3_test::Evaluate(cases[i],0)[0];
    ASSERT_EQ(Bits(out[i].squared_clearance),Bits(native));ASSERT_EQ(out[i].included,native!=0.);
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(out),cudaSuccess);EXPECT_EQ(cudaFree(rows),cudaSuccess);
}
TEST(NativeCandidateCuda,ActualDeviceStrictNativeScreens) {
  const auto cases=Cases();const auto count=cases.size();
  c::ScreenRow* rows=nullptr;bool* out=nullptr;c::Status* status=nullptr;
  ASSERT_EQ(cudaMallocManaged(&rows,count*sizeof(*rows)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&out,count*sizeof(*out)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(&status,count*sizeof(*status)),cudaSuccess);
  for(std::size_t i=0;i<count;++i) {
    const auto packet=Pack(cases[i]);rows[i]={};
    for(unsigned j=0;j<4;++j)rows[i].vertices[j]=packet.vertices[j];
    rows[i].secondary=packet.secondary;rows[i].secondary_gap=packet.gap;rows[i].margin=packet.margin;
    rows[i].gap_load=-.125;rows[i].drad=.125;rows[i].stored_motion=.03125;
  }
  Screens<<<32,128>>>(rows,count,out,status);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  for(std::size_t i=0;i<count;++i) {
    SCOPED_TRACE(i);ASSERT_EQ(status[i],c::Status::Ok);ASSERT_EQ(out[i],NativeScreen(rows[i]));
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(out),cudaSuccess);EXPECT_EQ(cudaFree(rows),cudaSuccess);
}
