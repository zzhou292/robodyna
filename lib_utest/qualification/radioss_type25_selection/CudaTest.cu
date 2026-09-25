// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "NativeOracle.h"
#include <algorithm>
#include "../radioss_type25_friction/CudaFixture.h"
namespace type25_selection_test {
namespace {
using Drain=type25_friction_test::Drain;
class RetainedCuda:public type25_friction_test::FrictionCuda {};
struct Packet {s::Profile profile;s::NativePairInput input;n::NativeGeometryHistory prior;};
struct Response {s::NativeRetainedResult value;s::Status status;};
__global__ void RetainedPackets(const Packet* in,Response* out,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    out[i].status=s::EvaluateNativeRetained(in[i].profile,in[i].input,in[i].prior,&out[i].value);
}
TEST_F(RetainedCuda, FullCorpusPermutationsMatchCpuNativeAndDefinedMasks) {
  static_assert(sizeof(Packet)<=RowBytes&&sizeof(Response)<=RowBytes);
  auto cases=Cases();ASSERT_LE(cases.size(),Capacity);
  for(unsigned threads:{32u,128u}) {
    std::reverse(cases.begin(),cases.end());
    std::vector<Packet> packets;for(const auto& c:cases)packets.push_back({Profile(),c.input,c.prior});
    std::vector<Response> actual(cases.size());Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input,packets.data(),packets.size()*sizeof(Packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    RetainedPackets<<<2,threads,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output),packets.size());
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(Response),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(std::size_t i=0;i<cases.size();++i) {
      SCOPED_TRACE(cases[i].name);ASSERT_EQ(actual[i].status,s::Status::Ok);
      s::NativeRetainedResult cpu;
      ASSERT_EQ(s::EvaluateNativeRetained(Profile(),cases[i].input,cases[i].prior,&cpu),s::Status::Ok);
      Same(actual[i].value,cpu,true);Same(actual[i].value,OracleRetained(Profile(),cases[i].input,cases[i].prior));
    }
  }
}
TEST_F(RetainedCuda, InactiveUndefinedAndLateInvalidPacketsPreserveThenRetry) {
  auto c=Basic();Packet packet{Profile(),c.input,c.prior};Response actual,before{Sentinel(),s::Status::Ok};
  Drain drain{stream};
  for(unsigned fault=0;fault<3;++fault) {
    packet={Profile(),c.input,c.prior};
    if(fault==0)packet.prior.row.irtlm[1]=0;
    if(fault==1)packet.input.key.generation++;
    if(fault==2)packet.input.main_coefficient=0;
    ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(output,&before,sizeof(before),cudaMemcpyHostToDevice,stream),cudaSuccess);
    RetainedPackets<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output),1);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    EXPECT_EQ(actual.status,fault==0?s::Status::UndefinedNativeInput: fault==1?s::Status::InvalidInput:s::Status::Ok);
    if(fault<2)Same(actual.value,before.value,true);
    else Same(actual.value,OracleRetained(Profile(),packet.input,packet.prior));
  }
  packet={Profile(),c.input,c.prior};
  ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
  RetainedPackets<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output),1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  EXPECT_EQ(actual.status,s::Status::Ok);Same(actual.value,OracleRetained(Profile(),c.input,c.prior));
}
} // namespace
} // namespace type25_selection_test
