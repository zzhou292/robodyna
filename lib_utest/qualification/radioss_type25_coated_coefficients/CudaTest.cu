// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace coated_coefficient_test {
using Drain=type25_friction_test::Drain;
class CoatedCuda:public type25_friction_test::FrictionCuda {};
__global__ void Evaluate(const Packet* packets,Result* results,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=gridDim.x*blockDim.x) {
    Result r;r.values={11,13,17};r.status=n::EvaluateNativeCoatedMainCoefficient(packets[i].input,&r.values);results[i]=r;
  }
}
TEST_F(CoatedCuda, NativeAsymmetricCoefficientsPreserveAllSchedulingOrders) {
  auto cases=Cases();std::vector<Packet> packets;for(const auto& p:cases)packets.push_back({p});
  static_assert(sizeof(Packet)<=RowBytes&&sizeof(Result)<=RowBytes);ASSERT_LE(packets.size(),Capacity);
  std::vector<Result> actual(packets.size());Drain drain{stream};
  for(unsigned threads:{32u,128u})for(unsigned repeat=0;repeat<4;++repeat) {
    std::reverse(packets.begin(),packets.end());
    ASSERT_EQ(cudaMemcpyAsync(input,packets.data(),packets.size()*sizeof(Packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    Evaluate<<<2,threads,0,stream>>>(static_cast<Packet*>(input),static_cast<Result*>(output),packets.size());
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(Result),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(std::size_t i=0;i<packets.size();++i) {SCOPED_TRACE(i);ASSERT_EQ(actual[i].status,n::CoefficientStatus::Ok);Same(actual[i].values,Oracle(packets[i].input));}
  }
}
TEST_F(CoatedCuda, OverflowPreservesEntireResultThenValidRetry) {
  Packet p{Base()};p.input.solid.area=std::numeric_limits<double>::max();Result actual;Drain drain{stream};
  for(unsigned attempt=0;attempt<2;++attempt) {
    if(attempt)p.input=Base();
    ASSERT_EQ(cudaMemcpyAsync(input,&p,sizeof(p),cudaMemcpyHostToDevice,stream),cudaSuccess);
    Evaluate<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Result*>(output),1);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    if(!attempt){EXPECT_EQ(actual.status,n::CoefficientStatus::NonfiniteResult);Same(actual.values,{11,13,17},true);}
    else{ASSERT_EQ(actual.status,n::CoefficientStatus::Ok);Same(actual.values,Oracle(p.input));}
  }
}
}
