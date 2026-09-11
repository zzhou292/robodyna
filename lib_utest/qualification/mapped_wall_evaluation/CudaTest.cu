#include "Fixture.h"
#include "FrozenEvaluation.cuh"
namespace wall_evaluation_test {
__global__ void Serial(d::Storage* s,tl::fea::DeviceNodalKinematicsView k,c::NodalWallDiagnostics id,
    const std::uint8_t* activity) {wall_evaluation_frozen::Evaluate<true>(*s,k,id,activity);}
__global__ void ResetSerialBase(d::Storage* s) {
  wall_evaluation_frozen::ResetResult(s->base,s->model.parent_count,s->model.node_count);
}
__global__ void CopyAccepted(d::Storage* s,bool serial) {
  if(s->control.status!=c::NodalWallDeviceStatus::Ok) return;
  if(serial) wall_evaluation_frozen::CopyBase(*s);
  else d::CopyBase(*s,blockIdx.x*blockDim.x+threadIdx.x,gridDim.x*blockDim.x);
}
void EvaluateFixture(Fixture& f,bool serial) {
  if(serial) Serial<<<1,d::Workers>>>(f.storage,f.Kinematics(),f.Identity(),f.input->activity);
  else m::parallel::Evaluate(f.storage,f.Side(),f.Kinematics(),f.Identity(),Nodes,Parents,nullptr,false);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
}
void SameControl(const Fixture& a,const Fixture& b) {
  EXPECT_EQ(a.storage->control.status,b.storage->control.status);
  EXPECT_EQ(a.storage->control.node,b.storage->control.node);
  EXPECT_EQ(a.storage->control.parent,b.storage->control.parent);
  EXPECT_EQ(a.storage->control.point.status,b.storage->control.point.status);
  EXPECT_EQ(a.storage->control.point.cause,b.storage->control.point.cause);
}
TEST(MappedWallEvaluationCuda, MultipleBlocksMixedParentsMasksAndSignedMotionMatchFrozenBits) {
  Fixture parallel,serial;
  for(unsigned mask=0;mask<4;++mask) {
    SCOPED_TRACE(mask);
    for(auto* f:{&parallel,&serial}) {
      f->Restore();
      for(unsigned p=0;p<Parents;++p)
        f->input->activity[p]=mask==0?1:mask==1?0:((p+mask)%3!=0);
      for(unsigned n=0;n<Nodes;++n) {
        if(n%7==0) f->input->x[3*n]=-.001;
        if(n%11==0) f->input->x[3*n]=-0.;
        f->input->velocity[3*n+1]=(int(n%3)-1)*.003;
      }
    }
    EvaluateFixture(parallel,false);EvaluateFixture(serial,true);
    ASSERT_EQ(parallel.storage->control.status,c::NodalWallDeviceStatus::Ok);
    SameControl(parallel,serial);
    v::SameResults(parallel.Read(),serial.Read());
  }
}
TEST(MappedWallEvaluationCuda, NodeAndParentFailuresPreservePriorityAndCleanRetry) {
  Fixture parallel,serial;
  for(unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);
    for(auto* f:{&parallel,&serial}) {
      f->Restore();
      if(fault==0) f->input->velocity[3*(Nodes-1)]=std::numeric_limits<double>::quiet_NaN();
      if(fault==1) {
        f->input->x[3*70+1]=3;
        f->input->velocity[3*130]=std::numeric_limits<double>::quiet_NaN();
      }
      if(fault==2) f->storage->model.config.law.parent_force_error=std::numeric_limits<double>::min();
      if(fault==3) f->storage->model.config.law.maximum_penetration=1e-5;
      if(fault==4) {
        f->storage->control.status=c::NodalWallDeviceStatus::InvalidMass;
        f->storage->control.node=Nodes-1;
        f->input->summary.points_admitted=false;
      }
    }
    EvaluateFixture(parallel,false);EvaluateFixture(serial,true);
    EXPECT_NE(parallel.storage->control.status,c::NodalWallDeviceStatus::Ok);
    SameControl(parallel,serial);
    // Failed internal parent scratch may finish in parallel; it is never
    // published. Public failure identity and diagnostics retain the old order.
    EXPECT_EQ(nodal_wall_owner_test::Bytes(parallel.storage->result.diagnostics),
        nodal_wall_owner_test::Bytes(serial.storage->result.diagnostics));
    parallel.Restore();serial.Restore();
    EvaluateFixture(parallel,false);EvaluateFixture(serial,true);
    ASSERT_EQ(parallel.storage->control.status,c::NodalWallDeviceStatus::Ok);
    SameControl(parallel,serial);v::SameResults(parallel.Read(),serial.Read());
  }
}
TEST(MappedWallEvaluationCuda, RejectedAssemblyClearsBaseAndAcceptedCopySurvivesCandidate) {
  Fixture parallel,serial;
  for(bool reject:{true,false}) {
    for(auto* f:{&parallel,&serial}) {
      f->Restore();
      auto& base=f->storage->base;
      base.diagnostics.valid=true; base.diagnostics.node_count=Nodes;
      for(unsigned p=0;p<Parents;++p) {base.parents[p].valid=true;base.parents[p].potential.value=77;}
      for(unsigned n=0;n<Nodes;++n) {base.nodes[n].valid=true;base.nodes[n].force.value=42;base.wall_face[n]=55;}
      if(reject) {
        f->storage->control.status=c::NodalWallDeviceStatus::InvalidMass;
        f->input->summary.points_admitted=false;
      }
    }
    ResetSerialBase<<<1,d::Workers>>>(serial.storage);
    m::parallel::Evaluate(parallel.storage,parallel.Side(),parallel.Kinematics(),parallel.Identity(),
        Nodes,Parents,nullptr,true);
    EvaluateFixture(serial,true);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    EXPECT_FALSE(parallel.storage->base.diagnostics.valid);
    EXPECT_EQ(parallel.storage->base.diagnostics.node_count,0u);
    for(unsigned p=0;p<Parents;++p) EXPECT_EQ(parallel.storage->base.parents[p].potential.value,0);
    for(unsigned n=0;n<Nodes;++n) EXPECT_EQ(parallel.storage->base.nodes[n].force.value,0);
    v::SameResults(parallel.Read(parallel.storage->base),serial.Read(serial.storage->base));
    if(!reject) for(auto* f:{&parallel,&serial}) f->storage->result.diagnostics.valid=true;
    CopyAccepted<<<m::parallel::Blocks(Parents),d::Workers>>>(parallel.storage,false);
    CopyAccepted<<<1,d::Workers>>>(serial.storage,true);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    v::SameResults(parallel.Read(parallel.storage->base),serial.Read(serial.storage->base));
    if(reject) {EXPECT_FALSE(parallel.storage->base.diagnostics.valid);continue;}
    const auto accepted=parallel.Read(parallel.storage->base);
    for(auto* f:{&parallel,&serial}) for(unsigned n=0;n<Nodes;++n) f->input->x[3*n]+=.0001;
    EvaluateFixture(parallel,false);EvaluateFixture(serial,true);
    ASSERT_EQ(parallel.storage->control.status,c::NodalWallDeviceStatus::Ok);
    v::SameResults(parallel.Read(),serial.Read());
    v::SameResults(accepted,parallel.Read(parallel.storage->base));
  }
}
} // namespace wall_evaluation_test
