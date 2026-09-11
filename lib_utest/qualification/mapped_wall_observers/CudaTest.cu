#include "../mapped_wall_evaluation/Fixture.h"
#include "../mapped_wall_evaluation/FrozenEvaluation.cuh"
#include "SerialObservers.h"
#include "Checks.h"
#include "Fixture.h"
#include "ExactChecks.h"
namespace wall_observer_cuda_test {
namespace old=wall_evaluation_test;
namespace c=tlfea::contact;
namespace d=c::nodal_wall_device_detail;
namespace m=c::nodal_wall_mapped;
namespace check=wall_observer_test;
using Code=c::NodalWallDeviceStatus;
constexpr auto Nodes=old::Nodes,Parents=old::Parents;
struct Extra {
  m::ObserverSummary summaries[m::ObserverMaxBlocks];
  double base_x[3*Nodes],base_v[3*Nodes];
};
struct Fixture:old::Fixture {
  Extra* extra=nullptr;
  Fixture(){EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&extra),sizeof(Extra)),cudaSuccess);*extra={};}
  ~Fixture(){cudaFree(extra);}
  m::ObserverScratch Scratch(){return {extra->summaries,m::ObserverMaxBlocks};}
};
__global__ void Serial(d::Storage* s,tl::fea::DeviceNodalKinematicsView k,c::NodalWallDiagnostics id,const std::uint8_t* a) {
  wall_evaluation_frozen::Evaluate<true>(*s,k,id,a);
}
__global__ void SerialGlobal(d::Storage* s,tl::fea::DeviceNodalKinematicsView k) {c::wall_observer_frozen::ReduceNodes<true>(*s,k);}
__global__ void CopyBase(d::Storage* s) {d::CopyBase(*s,blockIdx.x*blockDim.x+threadIdx.x,gridDim.x*blockDim.x);}
__global__ void Interval(d::Storage* s,tl::fea::NodalPreparedView view) {d::MeasureInterval(*s,view);}
void Sync(){ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
void Evaluate(Fixture& f,bool serial,bool reset=false,bool scratch=true) {
  if(serial) Serial<<<1,d::Workers>>>(f.storage,f.Kinematics(),f.Identity(),f.input->activity);
  else m::parallel::Evaluate(f.storage,f.Side(),f.Kinematics(),f.Identity(),Nodes,Parents,nullptr,reset,
      scratch?f.Scratch():m::ObserverScratch{});
  Sync();
}
void SameControl(const Fixture& a,const Fixture& b) {
  EXPECT_EQ(a.storage->control.status,b.storage->control.status);EXPECT_EQ(a.storage->control.node,b.storage->control.node);
  EXPECT_EQ(a.storage->control.parent,b.storage->control.parent);EXPECT_EQ(a.storage->control.point.status,b.storage->control.point.status);
  EXPECT_EQ(a.storage->control.point.cause,b.storage->control.point.cause);
}
void SamePhysical(const d::ActiveResults& a,const d::ActiveResults& b) {
  for(unsigned n=0;n<Nodes;++n) {check::Exact(a.nodes[n],b.nodes[n]);EXPECT_EQ(a.wall_face[n],b.wall_face[n]);}
  for(unsigned p=0;p<Parents;++p)check::Exact(a.parents[p],b.parents[p]);
}
TEST(MappedWallObserversCuda, ActualMixedParentsMasksAndDefaultPathPreserveEveryPhysicalPacket) {
  Fixture parallel,serial,legacy;
  for(unsigned mask=0;mask<4;++mask) {
    SCOPED_TRACE(mask);
    for(auto* f:{&parallel,&serial,&legacy}) {
      f->Restore();
      for(unsigned p=0;p<Parents;++p)f->input->activity[p]=mask==0?1:mask==1?0:((p+mask)%3!=0);
      for(unsigned n=0;n<Nodes;++n) {
        if(n%7==0)f->input->x[3*n]=-.001;
        if(n%11==0)f->input->x[3*n]=-0.;
        f->input->velocity[3*n+1]=double(int(n%3)-1)*.003;
      }
    }
    Evaluate(parallel,false);Evaluate(serial,true);Evaluate(legacy,false,false,false);
    ASSERT_EQ(parallel.storage->control.status,Code::Ok);SameControl(parallel,serial);SameControl(legacy,serial);
    SamePhysical(parallel.storage->result,serial.storage->result);SamePhysical(legacy.storage->result,serial.storage->result);
    check::ExactNonObserverDiagnostics(parallel.storage->result.diagnostics,serial.storage->result.diagnostics);
    check::ExactDiagnostics(legacy.storage->result.diagnostics,serial.storage->result.diagnostics);
    check::CheckGlobalTruth(parallel.storage->result.nodes,Nodes,parallel.storage->result.diagnostics);
  }
}
TEST(MappedWallObserversCuda, FailedPointParentAndExtremeGlobalPrefixesKeepOriginalPriorityAndRetry) {
  Fixture parallel,serial;
  for(unsigned fault=0;fault<4;++fault) {
    for(auto* f:{&parallel,&serial}) {
      f->Restore();
      if(fault==0){f->input->x[3*70+1]=3;f->input->velocity[3*130]=NAN;}
      if(fault==1)f->storage->model.config.law.parent_force_error=DBL_MIN;
      if(fault==2)f->storage->model.config.law.maximum_penetration=1e-5;
      if(fault==3){f->storage->control.status=Code::InvalidMass;f->input->summary.points_admitted=false;}
    }
    Evaluate(parallel,false,true);Evaluate(serial,true);
    EXPECT_NE(parallel.storage->control.status,Code::Ok);SameControl(parallel,serial);
    check::ExactDiagnostics(parallel.storage->result.diagnostics,serial.storage->result.diagnostics);
    EXPECT_FALSE(parallel.storage->base.diagnostics.valid);EXPECT_EQ(parallel.storage->base.diagnostics.node_count,0u);
    for(unsigned n=0;n<Nodes;++n)EXPECT_EQ(parallel.storage->base.nodes[n].force.value,0);
    parallel.Restore();serial.Restore();Evaluate(parallel,false);Evaluate(serial,true);
    ASSERT_EQ(parallel.storage->control.status,Code::Ok);SamePhysical(parallel.storage->result,serial.storage->result);
  }
  // Inject only diagnostic leaves after the independently qualified point and
  // parent stages. These controls exercise no invented contact force law.
  for(unsigned fault=0;fault<5;++fault) {
    for(auto* f:{&parallel,&serial}) {
      f->Restore();Evaluate(*f,false);
      f->storage->control={};f->storage->result.diagnostics=f->Identity();
      auto* n=f->storage->result.nodes;
      if(fault==0){n[1].wall_moment.x=.375*DBL_MAX;n[2].wall_moment.x=.375*DBL_MAX;n[3].wall_moment.x=-.375*DBL_MAX;}
      if(fault==1){n[1].wall_moment.x=DBL_MAX;n[2].wall_moment.x=DBL_MAX;n[3].wall_moment.x=-DBL_MAX;}
      if(fault==2){n[1].force={0,0,DBL_MAX,0};n[130].surface_power=INFINITY;}
      if(fault==3){n[70].potential.upper=NAN;n[130].surface_power=INFINITY;}
      if(fault==4){n[0].potential={-1,-1,0,0};}
    }
    SerialGlobal<<<1,1>>>(serial.storage,serial.Kinematics());
    m::parallel::ReduceGlobalObservers(parallel.storage,parallel.Side(),parallel.Kinematics(),parallel.Scratch(),nullptr);Sync();
    SameControl(parallel,serial);check::ExactDiagnostics(parallel.storage->result.diagnostics,serial.storage->result.diagnostics);
    SamePhysical(parallel.storage->result,serial.storage->result);
    parallel.Restore();Evaluate(parallel,false);ASSERT_EQ(parallel.storage->control.status,Code::Ok);
    check::CheckGlobalTruth(parallel.storage->result.nodes,Nodes,parallel.storage->result.diagnostics);
  }
}
TEST(MappedWallObserversCuda, AcceptedBaseCopyAndDerivedIntervalCertificatesSurviveCandidateRetry) {
  Fixture parallel,serial;
  for(auto* f:{&parallel,&serial}) {
    f->Restore();Evaluate(*f,f==&serial,false);
    std::copy_n(f->input->x,3*Nodes,f->extra->base_x);std::copy_n(f->input->velocity,3*Nodes,f->extra->base_v);
    f->storage->result.diagnostics.valid=true;CopyBase<<<m::parallel::Blocks(Parents),d::Workers>>>(f->storage);Sync();
    std::fill_n(f->storage->addition_error,Nodes,0.);
  }
  const auto accepted=parallel.Read(parallel.storage->base);
  for(unsigned attempt=0;attempt<2;++attempt) {
    for(auto* f:{&parallel,&serial}) {
      f->storage->control={};f->input->summary.parent_failure=~0ull;
      for(unsigned n=0;n<Nodes;++n) {f->input->x[3*n]=f->extra->base_x[3*n]+.00001;
        f->input->velocity[3*n]=f->extra->base_v[3*n]+.0003;}
      if(!attempt)f->input->velocity[3*(Nodes-1)]=NAN;
      Evaluate(*f,f==&serial,false);
    }
    SameControl(parallel,serial);
    if(!attempt){EXPECT_NE(parallel.storage->control.status,Code::Ok);continue;}
    ASSERT_EQ(parallel.storage->control.status,Code::Ok);SamePhysical(parallel.storage->result,serial.storage->result);
    for(auto* f:{&parallel,&serial}) {
      tl::fea::NodalPreparedView view;view.kick_dt=.001;view.kinematics=f->Kinematics();view.base_kinematics=f->Kinematics();
      view.base_kinematics.position_xyz=f->extra->base_x;view.base_kinematics.velocity_xyz=f->extra->base_v;
      Interval<<<1,1>>>(f->storage,view);Sync();ASSERT_EQ(f->storage->control.status,Code::Ok);
      check::CheckDerivedTruth(*f->storage,view);
    }
    const auto& a=parallel.storage->result.diagnostics;const auto& b=serial.storage->result.diagnostics;
    check::Exact(a.kick_work,b.kick_work);check::Exact(a.drift_work,b.drift_work);
    check::Exact(a.kick_work_roundoff,b.kick_work_roundoff);check::Exact(a.drift_work_roundoff,b.drift_work_roundoff);
    check::Exact(a.quadratic_work_upper,b.quadratic_work_upper);
  }
  const auto after=parallel.Read(parallel.storage->base);check::ExactDiagnostics(accepted.diagnostics,after.diagnostics);
  for(unsigned n=0;n<Nodes;++n)check::Exact(accepted.nodes[n],after.nodes[n]);
  for(unsigned p=0;p<Parents;++p)check::Exact(accepted.parents[p],after.parents[p]);EXPECT_EQ(accepted.faces,after.faces);
}
TEST(MappedWallObserversCuda, MaximumSummaryCountGridStrideCancellationAndSubnormalControl) {
  constexpr unsigned Count=33001;
  check::Fixture source(Count);
  struct Packet {d::Storage storage;m::Summary summary;m::ObserverSummary observer[256];};
  Packet* packet=nullptr;c::NodalWallPointResult* nodes=nullptr;double* positions=nullptr;
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&packet),sizeof(Packet)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&nodes),Count*sizeof(*nodes)),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&positions),3*Count*sizeof(double)),cudaSuccess);
  for(unsigned pattern=0;pattern<3;++pattern) {
    if(pattern==1)for(unsigned i=0;i<Count;++i)source.nodes[i].wall_moment.x=i%3==0?0x1p54:i%3==1?1:-0x1p54;
    if(pattern==2)for(auto& n:source.nodes) {
      n.force=n.potential={0x0.0000000000001p-1022,0x0.0000000000001p-1022,0x0.0000000000001p-1022,0};
      n.wall_reaction={0x0.0000000000001p-1022,-0.,0.};n.wall_moment={-0.,0.,-0.};n.surface_power=-0.;
    }
    const auto expected=source.Staged();ASSERT_EQ(source.storage.control.status,Code::Ok);
    *packet={};packet->storage=source.storage;packet->storage.control={};packet->storage.result.diagnostics=source.Identity();
    packet->storage.result.nodes=nodes;packet->summary.points_admitted=true;
    std::copy(source.nodes.begin(),source.nodes.end(),nodes);std::copy(source.positions.begin(),source.positions.end(),positions);
    auto k=source.Kinematics();k.position_xyz=positions;
    m::Sidecar side;side.summary=&packet->summary;
    m::parallel::ReduceGlobalObservers(&packet->storage,side,k,{packet->observer,256},nullptr);Sync();
    ASSERT_EQ(packet->storage.control.status,Code::Ok);check::ExactDiagnostics(packet->storage.result.diagnostics,expected);
    check::CheckGlobalTruth(nodes,Count,packet->storage.result.diagnostics);
  }
  EXPECT_EQ(cudaFree(positions),cudaSuccess);EXPECT_EQ(cudaFree(nodes),cudaSuccess);EXPECT_EQ(cudaFree(packet),cudaSuccess);
}
} // namespace wall_observer_cuda_test
