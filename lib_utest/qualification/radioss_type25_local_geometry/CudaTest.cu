// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include "NativeOracle.h"
#include "UnitFixture.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace type25_geometry_test {
namespace {
using Drain=type25_friction_test::Drain;
class GeometryCuda:public type25_friction_test::FrictionCuda {};
struct Response { n::NativeRawGeometryResult value; n::GeometryStatus status; };
__global__ void RawKernel(n::GeometryProfile p,const n::NativeGeometryInput* in,Response* out,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    out[i].status=n::EvaluateNativeRawGeometry(p,in[i],&out[i].value);
}
TEST_F(GeometryCuda, CompleteNativeCorpusAndSchedulePermutationsMatchAllDefinedFields) {
  auto cases=Cases();
  static_assert(sizeof(n::NativeGeometryInput)<=RowBytes&&sizeof(Response)<=RowBytes);
  ASSERT_LE(cases.size(),Capacity);
  for(unsigned threads:{32u,128u}) {
    for(unsigned repeat=0;repeat<4;++repeat) {
      std::reverse(cases.begin(),cases.end());
      std::vector<n::NativeGeometryInput> packets;for(const auto& c:cases)packets.push_back(c.input);
      std::vector<Response> actual(cases.size());
      Drain drain{stream};
      ASSERT_EQ(cudaMemcpyAsync(input,packets.data(),packets.size()*sizeof(packets[0]),cudaMemcpyHostToDevice,stream),cudaSuccess);
      RawKernel<<<2,threads,0,stream>>>(Profile(),static_cast<n::NativeGeometryInput*>(input),static_cast<Response*>(output),packets.size());
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(actual[0]),cudaMemcpyDeviceToHost,stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
      for(std::size_t i=0;i<cases.size();++i) {
        SCOPED_TRACE(cases[i].name);ASSERT_EQ(actual[i].status,n::GeometryStatus::Ok);
        n::NativeRawGeometryResult cpu;ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),packets[i],&cpu),n::GeometryStatus::Ok);
        Same(actual[i].value,cpu,true);Same(actual[i].value,OracleRaw(Profile(),packets[i]));
      }
    }
  }
}
TEST_F(GeometryCuda, MalformedPacketKeepsEveryPriorFieldAndRetrySucceeds) {
  auto packet=Quad();packet.corner_normal[2].x=std::numeric_limits<float>::quiet_NaN();
  Response before{Sentinel<n::NativeUnitsTag>(),n::GeometryStatus::Ok},actual=before;
  Drain drain{stream};
  ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(output,&before,sizeof(before),cudaMemcpyHostToDevice,stream),cudaSuccess);
  RawKernel<<<1,1,0,stream>>>(Profile(),static_cast<n::NativeGeometryInput*>(input),static_cast<Response*>(output),1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  EXPECT_EQ(actual.status,n::GeometryStatus::InvalidInput);Same(actual.value,before.value,true);
  packet=Quad();
  ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
  RawKernel<<<1,1,0,stream>>>(Profile(),static_cast<n::NativeGeometryInput*>(input),static_cast<Response*>(output),1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  ASSERT_EQ(actual.status,n::GeometryStatus::Ok);Same(actual.value,OracleRaw(Profile(),packet));
}
struct SiPacket { n::UnitScale units; n::SiGeometryInput input; };
struct SiResponse { n::SiRawGeometryResult value; n::GeometryStatus status; };
__global__ void SiKernel(const SiPacket* in,SiResponse* out) {
  out->status=n::EvaluateSiRawGeometry({1,1,5,1,false,false,false},in->units,in->input,&out->value);
}
TEST_F(GeometryCuda, ActualSiGeometryConversionRunsOnDevice) {
  static_assert(sizeof(SiPacket)<=RowBytes&&sizeof(SiResponse)<=RowBytes);
  const auto native=Cases().at(136).input;
  SiPacket packet{{.001,1000,1},Si(native,{.001,1000,1})};SiResponse actual;
  Drain drain{stream};
  ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
  SiKernel<<<1,1,0,stream>>>(static_cast<SiPacket*>(input),static_cast<SiResponse*>(output));
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  ASSERT_EQ(actual.status,n::GeometryStatus::Ok);
  Same(actual.value,Si(OracleRaw(Profile(),native),packet.units));
}
} // namespace
} // namespace type25_geometry_test
