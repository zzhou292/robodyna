#include "ForceStageCaptureFixture.h"
#include "ForceStageCaptureCudaProbe.h"
#include <vector>

namespace force_stage_capture_test {
namespace {
__global__ void EveryNodeLoad(fe::NodalAssemblyView v) {
  for(std::size_t n=0;n<v.accepted.node_count;++n) {
    v.forces.force_x[n]=(1.+n)/1024.;v.forces.force_y[n]=-.5;v.forces.force_z[n]=.25;
    v.forces.couple_x[n]=.001;v.forces.couple_y[n]=-.002;v.forces.couple_z[n]=.003;
  }
}
}
TEST_F(Cuda,HighCountCapturePreflightsWholeOwnerBytesWithoutAllocationAndReadsEveryNode) {
  constexpr std::size_t n=fe::MaxNodalStateNodes;
  static_assert(n==2048,"This gate explicitly reaches the admitted owner tail");
  std::vector<double> x(3*n),v(3*n),w(3*n),q(4*n),inverse(n,.5),inverse_j(n,1000),a(3*n,-99),ar(3*n,-99);
  std::vector<std::uint8_t> fixed(n);
  for(std::size_t i=0;i<n;++i){v[3*i]=.125;q[4*i]=1;}
  fe::NodalRigidGroupMember members[]{
    {101,2047,{-.1,0,0},2,.001,.0004,.0006},
    {102,17,{.1,0,0},2,.001,.0004,.0006},
    {103,1025,{0,.2,0},2,.001,.0004,.0006}};
  for(const auto& m:members){x[3*m.global_node]=m.position.x;x[3*m.global_node+1]=m.position.y;}
  fe::NodalRigidGroupInput group{300,400,members,3};fe::NodalRigidGroupModel model;
  ASSERT_TRUE(model.Initialize({Source,n,&group,1,{1000,.001}}));
  fe::NodalStateConfig c;c.node_count=n;c.fixed_dt=1./1024;c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto initialize=[&](fe::FENodalState& owner) {
    return owner.Initialize(c,{x.data(),v.data(),w.data(),n,q.data()},inverse.data(),{fixed.data(),fixed.data(),inverse_j.data()},model);
  };
  fe::FENodalState legacy;ASSERT_EQ(initialize(legacy).status,Code::Ok);const auto base=legacy.allocations();
  const auto required=base.device_bytes+6*(n+1)*sizeof(double);
  ASSERT_LE(required,fe::MaxTranslationDeviceBytes);
  c.capture_force_stage_accelerations=true;c.max_device_bytes=required-1;
  const auto calls=force_stage_capture_probe::AllocationCalls();fe::FENodalState rejected;
  EXPECT_EQ(initialize(rejected).status,Code::ResourceLimit);EXPECT_EQ(force_stage_capture_probe::AllocationCalls(),calls);
  EXPECT_EQ(rejected.allocations().device_bytes,0u);EXPECT_EQ(rejected.accepted().owner_id,0u);
  c.max_device_bytes=required;fe::FENodalState owner;ASSERT_EQ(initialize(owner).status,Code::Ok);
  EXPECT_EQ(owner.allocations().device_bytes,required);EXPECT_EQ(owner.allocations().device_allocations,base.device_allocations);
  fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_EQ(owner.BeginTrial(&token,&view).status,Code::Ok);
  EveryNodeLoad<<<1,1,0,view.stream>>>(view);ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  ASSERT_EQ(fe::AdvanceStaggeredRigidGroups(owner,token,Admission(owner,view)).status,Code::Ok);
  fe::NodalRigidGroupAccelerationSnapshot g;fe::NodalPreparedView prepared;
  const auto before_read=force_stage_capture_probe::AllocationCalls();
  ASSERT_EQ(owner.CopyPreparedForceStage(token,{a.data(),ar.data(),n,&g,1},&prepared).status,Code::Ok);
  EXPECT_EQ(force_stage_capture_probe::AllocationCalls(),before_read);
  EXPECT_EQ(g.source_group_id,300u);EXPECT_EQ(g.source_node_set_id,400u);EXPECT_EQ(g.member_count,3u);
  for(std::size_t i=0;i<n;++i) {
    const bool member=i==2047||i==17||i==1025;
    for(unsigned j=0;j<3;++j){EXPECT_TRUE(std::isfinite(a[3*i+j]));EXPECT_TRUE(std::isfinite(ar[3*i+j]));}
    if(!member) {
      EXPECT_EQ(a[3*i],.5*(1.+i)/1024);EXPECT_EQ(a[3*i+1],-.25);EXPECT_EQ(a[3*i+2],.125);
      EXPECT_EQ(ar[3*i],1);EXPECT_EQ(ar[3*i+1],-2);EXPECT_EQ(ar[3*i+2],3);
    }
  }
  ASSERT_TRUE(Accept(owner,token,view));
}
TEST_F(Cuda,CaptureStartupRejectsMissingGroupsLegacyDofsAndWrongSchemeBeforeDeviceAllocation) {
  Fixture fixture;auto c=Config(fixture);const auto calls=force_stage_capture_probe::AllocationCalls();
  const auto& in=fixture.input;const fe::HostNodalKinematicsView input{in.x.data(),in.v.data(),in.omega.data(),in.n,in.q.data()};
  const fe::NodalDofConfig dofs{in.fixed.data(),in.rotation_fixed.data(),in.inverse_inertia.data()};
  fe::FENodalState no_groups,legacy,wrong_scheme;
  EXPECT_EQ(no_groups.Initialize(c,input,in.inverse.data(),dofs).status,Code::UnsupportedTemporalScheme);
  EXPECT_EQ(legacy.Initialize(c,input,in.inverse.data(),in.fixed.data()).status,Code::UnsupportedTemporalScheme);
  c.temporal_scheme=fe::NodalTemporalScheme::VelocityFirst;
  EXPECT_EQ(Initialize(fixture,wrong_scheme,c).status,Code::UnsupportedTemporalScheme);
  EXPECT_EQ(force_stage_capture_probe::AllocationCalls(),calls);
}
} // namespace force_stage_capture_test
