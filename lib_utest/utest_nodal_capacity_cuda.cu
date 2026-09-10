// Owner capacity only: constant prescribed loads do not qualify larger shell
// batches, mesh-wall contributors or nodal-rigid connection integration.
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>

namespace {
namespace fe=tl::fea;
using Status=fe::NodalStatus;
constexpr double H=1./1024;
struct Fields {
  std::size_t n;
  std::vector<double> x,v,w,q,rf,rc;
  fe::NodalStamp stamp{};
  explicit Fields(std::size_t count):n(count),x(3*n),v(3*n),w(3*n),q(4*n),rf(3*n),rc(3*n) {}
  fe::NodalSnapshotBuffer buffer() { return {x.data(),v.data(),n,q.data(),w.data(),rf.data(),rc.data()}; }
};
struct Initial:Fields {
  std::vector<double> inverse,inertia;
  std::vector<std::uint8_t> fixed,rotation_fixed;
  explicit Initial(std::size_t count):Fields(count),inverse(n),inertia(n),fixed(n),rotation_fixed(n) {
    for(std::size_t i=0;i<n;++i) {
      inverse[i]=.5+.125*(i%4); inertia[i]=.25+.0625*(i%4);
      x[3*i]=double(i)/2048; x[3*i+1]=-.5; x[3*i+2]=.25;
      v[3*i]=.125; v[3*i+1]=-.25; v[3*i+2]=.0625;
      w[3*i+2]=.2; q[4*i]=1;
    }
  }
  fe::NodalStateConfig config(fe::NodalTemporalScheme scheme) const {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=H; c.temporal_scheme=scheme; return c;
  }
  fe::NodalReport initialize(fe::FENodalState& owner,fe::NodalStateConfig c) const {
    return owner.Initialize(c,{x.data(),v.data(),w.data(),n,q.data()},inverse.data(),
                            {fixed.data(),rotation_fixed.data(),inertia.data()});
  }
};
__global__ void Loads(fe::NodalAssemblyView view,bool corrupt_last) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<view.accepted.node_count;i+=gridDim.x*blockDim.x) {
    view.forces.force_x[i]=1.+double(i)/2048;
    view.forces.force_y[i]=-.5; view.forces.force_z[i]=.25;
    view.forces.couple_z[i]=corrupt_last&&i+1==view.accepted.node_count?1.7976931348623157e308:.75;
  }
}
bool Read(fe::FENodalState& owner,Fields& out) {
  const auto r=owner.CopyAccepted(out.buffer(),&out.stamp);
  EXPECT_EQ(r.status,Status::Ok)<<r.message; return r.status==Status::Ok;
}
void Same(const Fields& a,const Fields& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.q,b.q); EXPECT_EQ(a.w,b.w);
  EXPECT_EQ(a.rf,b.rf); EXPECT_EQ(a.rc,b.rc);
  EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);
  EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.velocity_time,b.stamp.velocity_time);
  EXPECT_EQ(a.stamp.velocity_phase,b.stamp.velocity_phase);
}
fe::NodalReport Advance(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view) {
  if(owner.accepted().temporal_scheme==fe::NodalTemporalScheme::StaggeredHalfKickStart)
    return fe::AdvanceStaggeredPrescribed(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,H,.1});
  return fe::AdvanceNodal(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,H,.1,
                                    fe::NodalStepAdmissionKind::PrescribedConstantLoads});
}
bool Prepare(fe::FENodalState& owner,fe::NodalTrialToken& token,bool corrupt_last=false) {
  fe::NodalAssemblyView view;
  auto r=owner.BeginTrial(&token,&view); EXPECT_EQ(r.status,Status::Ok); if(r.status!=Status::Ok) return false;
  Loads<<<8,128,0,view.stream>>>(view,corrupt_last);
  r=owner.SealAssembly(token); EXPECT_EQ(r.status,Status::Ok); if(r.status!=Status::Ok) return false;
  r=Advance(owner,token,view); EXPECT_EQ(r.status,Status::Ok)<<r.message; return r.status==Status::Ok;
}
// Download each declared field through its borrowed public view. No assumption
// about the owner's private packing or pointer adjacency is made here.
bool PreparedValues(fe::FENodalState& owner,const fe::NodalTrialToken& token,std::vector<double>& output) {
  fe::NodalPreparedView view;
  auto r=owner.BorrowPrepared(token,&view); EXPECT_EQ(r.status,Status::Ok); if(r.status!=Status::Ok) return false;
  const auto n=view.kinematics.node_count;
  output.resize(13*n);
  const double* fields[]{view.kinematics.position_xyz,view.kinematics.velocity_xyz,
    view.kinematics.angular_velocity_xyz,view.kinematics.orientation_wxyz};
  for(unsigned field=0;field<4;++field) {
    const auto e=cudaMemcpyAsync(output.data()+3*n*field,fields[field],(field==3?4:3)*n*sizeof(double),
                                cudaMemcpyDeviceToHost,view.stream);
    EXPECT_EQ(e,cudaSuccess); if(e!=cudaSuccess) return false;
  }
  const auto done=cudaStreamSynchronize(view.stream); EXPECT_EQ(done,cudaSuccess); return done==cudaSuccess;
}
class NodalCapacityCuda:public ::testing::Test {
  void SetUp() override { int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0); }
};

TEST_F(NodalCapacityCuda, AssemblyAndMaximumOwnerCountsMatchAnalyticConstantLoadsAndSpin) {
  for(auto n:{std::size_t(1030),fe::MaxNodalStateNodes})
    for(auto scheme:{fe::NodalTemporalScheme::VelocityFirst,fe::NodalTemporalScheme::StaggeredHalfKickStart}) {
      SCOPED_TRACE(n);
      SCOPED_TRACE(static_cast<int>(scheme));
      Initial in(n); fe::FENodalState owner;
      ASSERT_EQ(in.initialize(owner,in.config(scheme)).status,Status::Ok);
      const auto allocation=owner.allocations(); EXPECT_EQ(allocation.device_allocations,6u);
      EXPECT_LE(allocation.device_bytes,fe::MaxTranslationDeviceBytes);
      constexpr unsigned steps=8;
      for(unsigned step=0;step<steps;++step) {
        fe::NodalTrialToken token; ASSERT_TRUE(Prepare(owner,token)); ASSERT_EQ(owner.Commit(token).status,Status::Ok);
      }
      Fields out(n); ASSERT_TRUE(Read(owner,out));
      const bool staggered=scheme==fe::NodalTemporalScheme::StaggeredHalfKickStart;
      const double kicks=(steps-(staggered?.5:0))*H;
      const double drifts=.5*steps*(steps+(staggered?0:1))*H*H;
      for(std::size_t i=0;i<n;++i) {
        const double forces[]{1.+double(i)/2048,-.5,.25};
        for(unsigned axis=0;axis<3;++axis) {
          const auto j=3*i+axis; const auto acceleration=in.inverse[i]*forces[axis];
          EXPECT_NEAR(out.v[j],in.v[j]+kicks*acceleration,2e-15);
          EXPECT_NEAR(out.x[j],in.x[j]+steps*H*in.v[j]+drifts*acceleration,2e-15);
          EXPECT_EQ(out.rf[j],0); EXPECT_EQ(out.rc[j],0);
        }
        const double alpha=.75*in.inertia[i],angle=steps*H*.2+drifts*alpha;
        EXPECT_NEAR(out.w[3*i+2],.2+kicks*alpha,2e-15);
        EXPECT_EQ(out.w[3*i],0); EXPECT_EQ(out.w[3*i+1],0);
        EXPECT_NEAR(out.q[4*i],std::cos(.5*angle),2e-15);
        EXPECT_NEAR(out.q[4*i+3],std::sin(.5*angle),2e-15);
        EXPECT_EQ(out.q[4*i+1],0); EXPECT_EQ(out.q[4*i+2],0);
      }
      EXPECT_EQ(out.stamp.epoch,steps); EXPECT_EQ(out.stamp.time,steps*H);
      EXPECT_EQ(out.stamp.velocity_time,kicks);
      EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
      EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
    }
}

TEST_F(NodalCapacityCuda, LastNodeAdvanceFailurePreservesAcceptedStateAndRetriesExactly) {
  Initial in(1030); in.inertia.back()=2;
  fe::FENodalState owner;
  ASSERT_EQ(in.initialize(owner,in.config(fe::NodalTemporalScheme::StaggeredHalfKickStart)).status,Status::Ok);
  Fields before(in.n),after(in.n); ASSERT_TRUE(Read(owner,before));
  fe::NodalTrialToken clean; ASSERT_TRUE(Prepare(owner,clean));
  std::vector<double> expected; ASSERT_TRUE(PreparedValues(owner,clean,expected)); owner.Discard();
  fe::NodalTrialToken failed; fe::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&failed,&view).status,Status::Ok);
  Loads<<<8,128,0,view.stream>>>(view,true);
  ASSERT_EQ(owner.SealAssembly(failed).status,Status::Ok);
  const auto rejected=Advance(owner,failed,view);
  EXPECT_EQ(rejected.status,Status::InvalidOutput); EXPECT_EQ(rejected.node,in.n-1);
  EXPECT_EQ(owner.Commit(failed).status,Status::WrongPhase);
  ASSERT_TRUE(Read(owner,after)); Same(after,before);
  fe::NodalTrialToken retry; ASSERT_TRUE(Prepare(owner,retry));
  std::vector<double> actual; ASSERT_TRUE(PreparedValues(owner,retry,actual)); EXPECT_EQ(actual,expected);
  ASSERT_EQ(owner.Commit(retry).status,Status::Ok);
  ASSERT_TRUE(Read(owner,after)); EXPECT_EQ(after.stamp.epoch,1u); EXPECT_EQ(after.stamp.velocity_time,H/2);
}

TEST_F(NodalCapacityCuda, ActiveCountAllocationAndLateStartupFailurePreserveBudgetAndOwner) {
  Initial large(fe::MaxNodalStateNodes),small(1030);
  auto scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  fe::FENodalState large_owner,small_owner;
  ASSERT_EQ(large.initialize(large_owner,large.config(scheme)).status,Status::Ok);
  ASSERT_EQ(small.initialize(small_owner,small.config(scheme)).status,Status::Ok);
  EXPECT_EQ(large_owner.allocations().device_bytes-small_owner.allocations().device_bytes,411*(large.n-small.n));
  auto c=large.config(scheme); c.max_device_bytes=large_owner.allocations().device_bytes-1;
  fe::FENodalState rejected;
  EXPECT_EQ(large.initialize(rejected,c).status,Status::ResourceLimit);
  EXPECT_EQ(rejected.allocations().device_bytes,0u); EXPECT_EQ(rejected.accepted().owner_id,0u);
  large.q.back()=std::numeric_limits<double>::quiet_NaN();
  const auto invalid=large.initialize(rejected,large.config(scheme));
  EXPECT_EQ(invalid.status,Status::InvalidInput); EXPECT_EQ(invalid.node,large.n-1);
  EXPECT_EQ(rejected.allocations().device_bytes,0u); EXPECT_EQ(rejected.accepted().owner_id,0u);
}
} // namespace
