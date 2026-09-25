// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ContinuationAssertions.h"
#include "NativeOracle.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <algorithm>
namespace type25_selection_test {
namespace {
using Drain=type25_friction_test::Drain;
class ContinuationCuda:public type25_friction_test::FrictionCuda {};
struct Packet {s::NativeContinuationInput input;n::NativeGeometryHistory prior;};
struct Response {s::NativeContinuationResult value;s::Status status;};
__global__ void ContinuationPackets(const Packet* in,Response* out,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    out[i].status=s::EvaluateNativeContinuation({1,5,1,false,false,false},in[i].input,in[i].prior,&out[i].value);
}
TEST_F(ContinuationCuda, CompleteDefinedFieldsMatchCpuAndNativeUnderPermutations) {
  static_assert(sizeof(Packet)<=RowBytes&&sizeof(Response)<=RowBytes);
  auto cases=ContinuationCases();ASSERT_LE(cases.size(),Capacity);
  for(unsigned threads:{32u,128u}) {
    std::reverse(cases.begin(),cases.end());
    std::vector<Packet> packets;for(const auto& c:cases)packets.push_back({c.input,c.prior});
    std::vector<Response> actual(cases.size());Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input,packets.data(),packets.size()*sizeof(Packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ContinuationPackets<<<2,threads,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output),packets.size());
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(Response),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(std::size_t i=0;i<cases.size();++i) {
      SCOPED_TRACE(cases[i].name);ASSERT_EQ(actual[i].status,s::Status::Ok);
      s::NativeContinuationResult cpu;
      ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),cases[i].input,cases[i].prior,&cpu),s::Status::Ok);
      Same(actual[i].value,cpu,true);
      Same(actual[i].value,OracleContinuation(Profile(),cases[i].input,cases[i].prior));
    }
  }
}
TEST_F(ContinuationCuda, BadSourcePreservesOutputAndValidRetryUsesSameAllocation) {
  auto c=BasicContinuation();Packet packet{c.input,c.prior};
  Response actual,before{ContinuationSentinel(),s::Status::Ok};Drain drain{stream};
  for(bool invalid:{true,false}) {
    packet.input.segment_count=invalid?0:c.input.segment_count;
    ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(output,&before,sizeof(before),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ContinuationPackets<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output),1);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    EXPECT_EQ(actual.status,invalid?s::Status::InvalidInput:s::Status::Ok);
    if(invalid)Same(actual.value,before.value,true);
    else Same(actual.value,OracleContinuation(Profile(),c.input,c.prior));
  }
}
} // namespace
} // namespace type25_selection_test
