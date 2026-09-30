#include "GroupOwnerFixture.h"
#include <vector>
namespace rigid_owner_test {
namespace {
__global__ void SparseLoads(fe::NodalAssemblyView view) {
  view.forces.force_x[2047]=2; view.forces.force_x[17]=2; view.forces.force_x[1025]=2;
  view.forces.force_x[2046]=4;
}
}
TEST_F(Cuda,SparseSourceMembersReachLastAdmittedNodeWithinWholeOwnerBudget) {
  constexpr std::size_t n=fe::MaxNodalStateNodes;
  static_assert(n==2048,"This gate explicitly reaches the current admitted tail");
  std::vector<double> x(3*n,0),v(3*n,0),omega(3*n,0),q(4*n,0),inverse(n,.5),inverse_j(n,1000);
  std::vector<std::uint8_t> fixed(n,0);
  for(std::size_t i=0;i<n;++i) { v[3*i]=.125; q[4*i]=1; }
  fe::NodalRigidGroupMember members[]{
    {101,2047,{-.1,0,0},2,.001,.0004,.0006},
    {102,17,{.1,0,0},2,.001,.0004,.0006},
    {103,1025,{0,.2,0},2,.001,.0004,.0006}};
  for(const auto& m:members) { x[3*m.global_node]=m.position.x; x[3*m.global_node+1]=m.position.y; }
  fe::NodalRigidGroupInput group{300,400,members,3}; fe::NodalRigidGroupModel model;
  ASSERT_TRUE(model.Initialize({Source,n,&group,1,{1000,.001}}));
  fe::NodalStateConfig config; config.node_count=n; config.fixed_dt=1./1024;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  fe::FENodalState owner;
  ASSERT_EQ(owner.Initialize(config,{x.data(),v.data(),omega.data(),n,q.data()},inverse.data(),
    {fixed.data(),fixed.data(),inverse_j.data()},model).status,Code::Ok);
  const auto allocations=owner.allocations(); EXPECT_EQ(allocations.device_allocations,7u);
  EXPECT_LE(allocations.device_bytes,fe::MaxTranslationDeviceBytes);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,Code::Ok);
  SparseLoads<<<1,1,0,view.stream>>>(view); ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  ASSERT_EQ(fe::AdvanceStaggeredRigidGroups(owner,token,Admission(owner,view)).status,Code::Ok);
  ASSERT_TRUE(Accept(owner,token,view));
  fe::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),n,q.data(),omega.data()},&stamp).status,Code::Ok);
  for(const auto& m:members) {
    rigid_step_test::Agreement(x[3*m.global_node],m.position.x+.125*config.fixed_dt+.5*config.fixed_dt*config.fixed_dt);
    rigid_step_test::Agreement(v[3*m.global_node],.125+.5*config.fixed_dt);
    for(unsigned a=0;a<3;++a) rigid_step_test::Agreement(omega[3*m.global_node+a],0);
  }
  rigid_step_test::Agreement(x[3*2046],.125*config.fixed_dt+config.fixed_dt*config.fixed_dt);
  rigid_step_test::Agreement(v[3*2046],.125+config.fixed_dt);
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  Groups output; ASSERT_TRUE(Read(owner,output));
  EXPECT_EQ(output.values[0].source_group_id,300u);
  EXPECT_TRUE(fe::SameRigidGroupInfo(stamp.rigid_groups,{Source,1,3}));
}
} // namespace rigid_owner_test
