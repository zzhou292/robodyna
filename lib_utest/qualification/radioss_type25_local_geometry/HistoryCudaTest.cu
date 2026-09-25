// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include "NativeOracle.h"
#include "HistoryFixture.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace type25_geometry_test {
namespace {
using Drain=type25_friction_test::Drain;
class GeometryHistoryCuda:public type25_friction_test::FrictionCuda {};
struct Packet {
  n::NativeRawGeometryResult geometry[4];
  n::NativeGeometryHistory rows[2];
  double time=0;
  int fault=0;
};
struct Response {
  n::NativeGeometryHistory staged[2],scratch_rows[2];
  n::NativeGeometryFinalResult results[4],scratch_results[4];
  n::GeometryBatchReport report;
};
__global__ void Serial(const Packet* in,Response* out) {
  n::GeometryHistoryBatch b{in->geometry,4,in->rows,2,out->staged,out->results,
      out->scratch_rows,2,out->scratch_results,4};
  n::GeometryBatchLimits limits;
  if(in->fault==1)limits.scratch_bytes=0;
  if(in->fault==2)b.scratch_results=b.results;
  out->report=n::FinalizeNativeGeometryHistory({1,1,5,1,false,false,false},in->time,b,limits);
}
// Qualification of the shared row-local phases with independent row writers.
// The fixed explicit incidence is test data, not a runtime search or scheduler.
__global__ void RowWriters(const Packet* in,Response* out) {
  const unsigned row=threadIdx.x;
  if(blockIdx.x||row>=2)return;
  auto value=in->rows[row];
  const unsigned incidence[2][2]{{0,2},{1,3}};
  for(unsigned j=0;j<2;++j)
    n::geometry_detail::InitialOffset(in->time,in->geometry[incidence[row][j]],value.row);
  for(unsigned j=0;j<2;++j) {
    const unsigned i=incidence[row][j];
    out->results[i].geometry=in->geometry[i];
    out->results[i].penetration=n::geometry_detail::ShiftOffset(in->geometry[i],value.row);
  }
  for(unsigned j=0;j<2;++j)
    n::geometry_detail::StageStiffness(out->results[incidence[row][j]],value.row);
  out->staged[row]=value;
}
Packet Input(double time,bool new_impact) {
  Packet p;p.time=time;p.rows[0]=History(Quad());p.rows[1]=p.rows[0];
  p.rows[1].secondary_source_id=2202;
  if(new_impact)p.rows[0].row.irtlm[0]=p.rows[1].row.irtlm[0]=-3;
  for(unsigned i=0;i<4;++i) {
    p.geometry[i]=Raw(i==0?-0.0:.1*i,400+100*i);
    p.geometry[i].key.history_index=i%2;p.geometry[i].key.secondary_source_id=p.rows[i%2].secondary_source_id;
  }
  return p;
}
TEST_F(GeometryHistoryCuda, RepeatedTargetsMatchNativeAndIndependentGpuRowWriters) {
  static_assert(sizeof(Packet)<=Capacity*RowBytes&&sizeof(Response)<=Capacity*RowBytes);
  for(double time:{0.,1.})for(bool new_impact:{false,true}) {
    auto packet=Input(time,new_impact);Response actual,parallel;
    const auto expected=OracleHistory(Profile(),time,
        std::vector<n::NativeRawGeometryResult>(packet.geometry,packet.geometry+4),
        std::vector<n::NativeGeometryHistory>(packet.rows,packet.rows+2));
    Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    Serial<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output));
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    ASSERT_EQ(actual.report.status,n::GeometryStatus::Ok);
    RowWriters<<<1,32,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output));
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&parallel,output,sizeof(parallel),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(unsigned i=0;i<2;++i){Same(actual.staged[i],expected.rows[i]);Same(parallel.staged[i],actual.staged[i]);}
    for(unsigned i=0;i<4;++i) {
      Same(actual.results[i].geometry,expected.results[i].geometry,true);
      Number(actual.results[i].penetration,expected.results[i].penetration,true);
      Same(parallel.results[i].geometry,actual.results[i].geometry,true);
      Number(parallel.results[i].penetration,actual.results[i].penetration,true);
    }
  }
}
TEST_F(GeometryHistoryCuda, CapacityAliasesAndLateSourceMismatchKeepBothOutputsThenRetry) {
  for(int fault:{1,2,3}) {
    auto packet=Input(1,false);packet.fault=fault;
    if(fault==3)packet.geometry[3].key.generation=999;
    Response prior;
    for(unsigned i=0;i<2;++i){prior.staged[i]=packet.rows[i];prior.staged[i].row.penetration_auxiliary=901;}
    for(auto& r:prior.results){r.geometry=Sentinel<n::NativeUnitsTag>();r.penetration=902;}
    Response actual=prior;Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(output,&prior,sizeof(prior),cudaMemcpyHostToDevice,stream),cudaSuccess);
    Serial<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output));
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    EXPECT_EQ(actual.report.status,fault==1?n::GeometryStatus::CapacityExceeded:n::GeometryStatus::InvalidInput);
    for(unsigned i=0;i<2;++i)Same(actual.staged[i],prior.staged[i]);
    for(unsigned i=0;i<4;++i){Same(actual.results[i].geometry,prior.results[i].geometry,true);Number(actual.results[i].penetration,prior.results[i].penetration,true);}
    packet=Input(1,false);
    ASSERT_EQ(cudaMemcpyAsync(input,&packet,sizeof(packet),cudaMemcpyHostToDevice,stream),cudaSuccess);
    Serial<<<1,1,0,stream>>>(static_cast<Packet*>(input),static_cast<Response*>(output));
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);EXPECT_EQ(actual.report.status,n::GeometryStatus::Ok);
  }
}
} // namespace
} // namespace type25_geometry_test
