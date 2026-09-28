// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FrozenTail.h"
#include "lib_src/collision/radioss_type25/runtime/AssemblyTail.h"
#include "lib_src/collision/radioss_type25/runtime/Launch.h"
#include "../radioss_type25_assembly/Packet.h"
#include <cuda_runtime.h>
#include <array>
#include <cstring>
#include <cmath>
#include <stdexcept>
namespace native_tail_test {
namespace rd=n::runtime_detail;namespace a=n::assembly;namespace fe=tl::fea;
constexpr std::size_t Nodes=93,Rows=257;
enum class Fault {None,FirstLaunch,SecondLaunch,Copy,Drain};
struct Data {
  rd::Control control;
  a::Connectivity rows[Rows];a::SiEndpoints packets[Rows];
  std::uint32_t ends[Rows]{},offsets[Nodes+1]{},occurrences[5*Rows]{};
  double x[Nodes]{},y[Nodes]{},z[Nodes]{},stiffness[Nodes]{};
  a::SiNodalValue output[Nodes];
};
void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
__global__ void NeverRuns() {}
std::uint64_t Bits(double v){std::uint64_t b;std::memcpy(&b,&v,sizeof b);return b;}
struct Result {
  n::TransactionReport report;rd::Control control;bool usable=true,published=false;
  std::size_t fences=0,copies=0,applies=0,clears=0;cudaError_t pending=cudaSuccess;
  std::array<a::SiNodalValue,Nodes> values;
};
struct Rig {
  Data* data=nullptr;cudaStream_t stream=nullptr;
  type25_assembly_test::Case source=type25_assembly_test::Corpus();
  explicit Rig(bool empty=false){
    if(empty){source.rows.clear();source.responses.clear();source.cohorts.clear();}
    Check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));Check(cudaMallocManaged(&data,sizeof(Data)));
    *data={};Reset();
  }
  ~Rig(){if(stream)cudaStreamSynchronize(stream);cudaFree(data);if(stream)cudaStreamDestroy(stream);}
  void Reset(int first_bad=-1,int second_bad=-1,bool overflow=false){
    Check(cudaStreamSynchronize(stream));*data={};
    data->control.required_sliding=17;data->control.required_candidates=source.rows.size();
    data->control.kept=source.rows.size();data->control.active=13;
    data->control.elastic_energy=-0.;data->control.damping_work=7.25;data->control.friction_work=19.5;
    std::copy(source.rows.begin(),source.rows.end(),data->rows);
    std::copy(source.cohorts.begin(),source.cohorts.end(),data->ends);
    const a::Schedule schedule{source.cohorts.empty()?nullptr:data->ends,source.cohorts.size(),source.rows.size()};
    if(!a::BuildIncidence(source.rows.empty()?nullptr:data->rows,schedule,Nodes,data->offsets,Nodes+1,
        source.rows.empty()?nullptr:data->occurrences,5*source.rows.size()))throw std::runtime_error("incidence setup");
    for(std::size_t row=0;row<source.rows.size();++row){a::NativeEndpoints native;
      if(a::PrepareNativeEndpoints(type25_assembly_test::Controls(),source.responses[row],&native)!=a::Status::Ok||
         a::EndpointsToSi({1,1,1},native,&data->packets[row])!=a::Status::Ok)throw std::runtime_error("endpoint setup");}
    for(std::size_t i=0;i<Nodes;++i){const auto& v=source.incoming[i];
      data->x[i]=v.force.x;data->y[i]=v.force.y;data->z[i]=v.force.z;data->stiffness[i]=v.stiffness;
      data->output[i]={{std::nan("17"),std::nan("18"),std::nan("19")},std::nan("20")};}
    for(int node:{first_bad,second_bad})if(node>=0)data->z[node]=std::nan("21");
    if(overflow){
      bool found=false;const std::size_t node=5;data->x[node]=std::numeric_limits<double>::max();
      for(auto at=data->offsets[node];at<data->offsets[node+1];++at){a::Occurrence o;
        if(!a::DecodeOccurrence(schedule,data->occurrences[at],&o))throw std::runtime_error("decode setup");
        auto& p=data->packets[o.row];if(!p.active)continue;
        if(o.slot<4)p.main_force[o.slot].x=std::numeric_limits<double>::max();
        else p.secondary_resultant.x=-std::numeric_limits<double>::max();found=true;break;}
      if(!found)throw std::runtime_error("overflow witness missing");
    }
  }
  std::array<a::SiNodalValue,Nodes> Values(){std::array<a::SiNodalValue,Nodes> v;
    for(std::size_t i=0;i<Nodes;++i)v[i]={{data->x[i],data->y[i],data->z[i]},data->stiffness[i]};return v;}
  Result Run(bool batched,Fault fault=Fault::None){
    Result result;result.control=data->control;rd::Device device;device.source.node_count=Nodes;device.control=&data->control;
    device.force_connectivity=data->rows;device.force_packets=data->packets;device.nodal_output=data->output;
    const a::Schedule schedule{source.cohorts.empty()?nullptr:data->ends,source.cohorts.size(),source.rows.size()};
    const a::Incidence incidence{data->offsets,source.rows.empty()?nullptr:data->occurrences,Nodes,5*source.rows.size()};
    fe::NodalAssemblyView view;view.forces.force_x=data->x;view.forces.force_y=data->y;view.forces.force_z=data->z;
    fe::NodalCinAssemblyView cin;cin.translational_stiffness=data->stiffness;
    auto bad_launch=[&]{NeverRuns<<<0,1,0,stream>>>();return cudaPeekAtLastError();};
    auto gather=[&]{return fault==Fault::FirstLaunch?bad_launch():rd::Gather(device,schedule,incidence,view,cin,stream);};
    auto apply=[&]{++result.applies;return fault==Fault::SecondLaunch?bad_launch():rd::Apply(device,view,cin,stream);};
    // Frozen production Fence semantics. Transport faults occur after real queued
    // work drains, without killing the CUDA context used by unrelated tests.
    auto fence=[&](cudaError_t error)->n::TransactionReport {
      ++result.fences;
      if(error==cudaSuccess){++result.copies;
        error=fault==Fault::Copy?cudaErrorInvalidValue:cudaMemcpyAsync(&result.control,device.control,
            sizeof(result.control),cudaMemcpyDeviceToHost,stream);}
      auto drained=cudaStreamSynchronize(stream);if(fault==Fault::Drain)drained=cudaErrorUnknown;
      if(error!=cudaSuccess||drained!=cudaSuccess){result.usable=false;
        return {n::TransactionStatus::DeviceFailure,"Native GPU stage failed"};}
      if(result.control.failure!=~0ull)return {static_cast<n::TransactionStatus>(result.control.failure&255),
          "Native GPU stage rejected",std::size_t(result.control.failure>>16),SIZE_MAX,
          static_cast<n::selection::Status>((result.control.failure>>8)&255)};
      return {n::TransactionStatus::Ok,"OK"};
    };
    auto clear=[&]{++result.clears;return cudaGetLastError();};
    result.report=batched?rd::AssembleTail(gather,apply,fence,clear):FrozenTail(gather,apply,fence);
    result.published=result.report.status==n::TransactionStatus::Ok;
    result.pending=cudaGetLastError();Check(cudaStreamSynchronize(stream));result.values=Values();return result;
  }
};
void SameValues(const std::array<a::SiNodalValue,Nodes>& x,const std::array<a::SiNodalValue,Nodes>& y){
  for(std::size_t i=0;i<Nodes;++i){SCOPED_TRACE(i);EXPECT_EQ(Bits(x[i].force.x),Bits(y[i].force.x));
    EXPECT_EQ(Bits(x[i].force.y),Bits(y[i].force.y));EXPECT_EQ(Bits(x[i].force.z),Bits(y[i].force.z));
    EXPECT_EQ(Bits(x[i].stiffness),Bits(y[i].stiffness));}
}
void SameReport(const Result& a,const Result& b){
 EXPECT_EQ(a.report.status,b.report.status);EXPECT_STREQ(a.report.message,b.report.message);
 EXPECT_EQ(a.report.row,b.report.row);EXPECT_EQ(a.report.occurrence,b.report.occurrence);EXPECT_EQ(a.report.selection_status,b.report.selection_status);
 EXPECT_EQ(a.usable,b.usable);EXPECT_EQ(a.published,b.published);
}
void SameControl(const rd::Control& a,const rd::Control& b){
 EXPECT_EQ(a.failure,b.failure);EXPECT_EQ(a.required_sliding,b.required_sliding);
 EXPECT_EQ(a.required_candidates,b.required_candidates);EXPECT_EQ(a.kept,b.kept);EXPECT_EQ(a.active,b.active);
 EXPECT_EQ(Bits(a.elastic_energy),Bits(b.elastic_energy));EXPECT_EQ(Bits(a.damping_work),Bits(b.damping_work));
 EXPECT_EQ(Bits(a.friction_work),Bits(b.friction_work));
}
TEST(NativeContactTailCuda,FrozenTwoFenceSuccessMatchesExactCohortsRepeatedSlotsAndEmptyCase){
 for(bool empty:{false,true}){Rig r(empty);const auto serial=r.Run(false);r.Reset();const auto batched=r.Run(true);
  ASSERT_EQ(serial.report.status,n::TransactionStatus::Ok);SameReport(batched,serial);SameControl(batched.control,serial.control);SameValues(batched.values,serial.values);
  EXPECT_EQ(serial.fences,2u);EXPECT_EQ(serial.copies,2u);EXPECT_EQ(batched.fences,1u);EXPECT_EQ(batched.copies,1u);
  EXPECT_EQ(batched.clears,0u);EXPECT_EQ(batched.pending,cudaSuccess);
 }
}
TEST(NativeContactTailCuda,AnyGatherRejectionPreservesAllDestinationsAndEarliestNode){
 for(auto bad: {std::pair<int,int>{0,-1},{92,-1},{7,91}}){Rig r;r.Reset(bad.first,bad.second);const auto original=r.Values();
  const auto serial=r.Run(false);r.Reset(bad.first,bad.second);const auto batched=r.Run(true);
  ASSERT_EQ(serial.report.status,n::TransactionStatus::NumericalFailure);EXPECT_EQ(serial.report.row,std::size_t(bad.first));
  SameReport(batched,serial);SameControl(batched.control,serial.control);SameValues(batched.values,serial.values);SameValues(batched.values,original);
  EXPECT_EQ(serial.applies,0u);EXPECT_EQ(batched.applies,1u);EXPECT_TRUE(batched.usable);EXPECT_FALSE(batched.published);
 }
 Rig r;r.Reset(-1,-1,true);const auto original=r.Values();const auto serial=r.Run(false);
 r.Reset(-1,-1,true);const auto batched=r.Run(true);ASSERT_EQ(serial.report.status,n::TransactionStatus::NumericalFailure);
 SameReport(batched,serial);SameValues(batched.values,original);
}
TEST(NativeContactTailCuda,LaterRealInvalidLaunchCannotMaskEarlierGatherFailureAndRetryIsFresh){
 Rig r;r.Reset(92);const auto original=r.Values();const auto serial=r.Run(false,Fault::SecondLaunch);
 r.Reset(92);const auto batched=r.Run(true,Fault::SecondLaunch);SameReport(batched,serial);SameValues(batched.values,original);
 EXPECT_EQ(batched.report.status,n::TransactionStatus::NumericalFailure);EXPECT_EQ(batched.report.row,92u);
 EXPECT_TRUE(batched.usable);EXPECT_EQ(batched.clears,1u);EXPECT_EQ(batched.pending,cudaSuccess);
 r.Reset();const auto repaired=r.Run(true);ASSERT_EQ(repaired.report.status,n::TransactionStatus::Ok);
 r.Reset();const auto expected=r.Run(false);SameValues(repaired.values,expected.values);
}
TEST(NativeContactTailCuda,ExistingFirstAndSecondLaunchErrorsStillPoisonAndNeverPublish){
 for(auto fault:{Fault::FirstLaunch,Fault::SecondLaunch}){Rig r;const auto original=r.Values();const auto serial=r.Run(false,fault);
  r.Reset();const auto batched=r.Run(true,fault);SameReport(batched,serial);SameValues(batched.values,original);
  EXPECT_EQ(batched.report.status,n::TransactionStatus::DeviceFailure);EXPECT_FALSE(batched.usable);EXPECT_FALSE(batched.published);
  if(fault==Fault::FirstLaunch){EXPECT_EQ(batched.applies,0u);EXPECT_EQ(batched.clears,0u);EXPECT_EQ(batched.pending,serial.pending);}
  else EXPECT_EQ(batched.clears,1u);
 }
}
TEST(NativeContactTailCuda,ReadbackAndDrainFaultsRetainDevicePoisonAndSuppressPublication){
 for(auto fault:{Fault::Copy,Fault::Drain})for(bool bad:{false,true}){Rig r;if(bad)r.Reset(7);const auto serial=r.Run(false,fault);
  r.Reset(bad?7:-1);const auto batched=r.Run(true,fault);SameReport(batched,serial);
  EXPECT_EQ(batched.report.status,n::TransactionStatus::DeviceFailure);EXPECT_FALSE(batched.usable);EXPECT_FALSE(batched.published);
  // Device failure may leave different private attempt arrays because Apply
  // was queued earlier. Neither path can publish that attempt; owning tests
  // separately verify common-owner discard and accepted selector immutability.
 }
}
} // namespace native_tail_test
