#include "VehicleRigidOwnerFixture.h"
#include "lib_src/solvers/FENodalStateStorage.h"
#include "lib_src/solvers/NodalRigidGroupStorage.h"
namespace vehicle_rigid_owner_test {
TEST_F(Cuda,All759Groups7539MembersAndFinalVehicleNodeShareClockCaptureAndPublication) {
  Fixture f;fe::FENodalState owner,control;
  ASSERT_EQ(f.Initialize(owner,f.Config(true)).status,Code::Ok);ASSERT_EQ(f.Initialize(control,f.Config()).status,Code::Ok);
  const auto g=f.source.model.group_count(),m=f.source.model.member_count();
  const auto allocation=owner.allocations();EXPECT_EQ(allocation.device_allocations,7u);
  // Independent tally of the existing separate allocations and the compact
  // immutable ranges/metrics/mask; no second group-state owner is counted.
  const auto base=(51*f.n+36*g)*sizeof(double)+3*f.n+sizeof(fe::nodal_detail::Control)+
    g*sizeof(rigid::GroupRange)+m*sizeof(rigid::MemberMetric)+f.n;
  EXPECT_EQ(control.allocations().device_bytes,base);
  EXPECT_EQ(allocation.device_bytes,base+6*(f.n+g)*sizeof(double));
  std::vector<fe::NodalRigidGroupSnapshot> groups(g);std::vector<fe::NodalRigidGroupAccelerationSnapshot> capture(g);
  std::vector<double> a(3*f.n),ar(3*f.n);Snapshot actual(f.n),reference(f.n);
  for(unsigned step=0;step<3;++step) {
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(Begin(owner,m,token,view));ASSERT_TRUE(Prepare(owner,token,view));
    fe::NodalPreparedView prepared,captured;
    ASSERT_EQ(owner.CopyPreparedRigidGroups(token,{groups.data(),g},&prepared).status,Code::Ok);
    ASSERT_EQ(owner.CopyPreparedForceStage(token,{a.data(),ar.data(),f.n,capture.data(),g},&captured).status,Code::Ok);
    EXPECT_TRUE(fe::trial_identity::SamePrepared(prepared,captured));
    EXPECT_TRUE(fe::SameRigidGroupInfo(prepared.rigid_groups,{Source,g,m}));
    for(std::size_t i=0;i<g;++i) {
      EXPECT_EQ(groups[i].source_group_id,f.source.groups[i].source_group_id);
      EXPECT_EQ(capture[i].source_group_id,groups[i].source_group_id);
      EXPECT_EQ(capture[i].member_count,f.source.groups[i].member_count);
      EXPECT_NEAR(capture[i].acceleration.x,1,2e-13);
    }
    for(const auto& member:f.source.members)EXPECT_NEAR(a[3*member.global_node],1,2e-10);
    EXPECT_EQ(a[3*(f.n-2)],2);
    ASSERT_TRUE(Commit(owner,token,view));
    ASSERT_TRUE(Begin(control,m,token,view));ASSERT_TRUE(Prepare(control,token,view));ASSERT_TRUE(Commit(control,token,view));
    ASSERT_TRUE(Read(owner,actual));ASSERT_TRUE(Read(control,reference));Same(actual,reference,false);
    EXPECT_EQ(actual.stamp.epoch,step+1);
    EXPECT_NEAR(actual.v[3*(f.n-1)],.125+(step+.5)*f.h,2e-12);
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
  }
}
TEST_F(Cuda,Maximum1024GroupTailReadbackAndObservationRetainDeclaredDescriptor) {
  Fixture f(1024,8193);fe::FENodalState owner;ASSERT_EQ(f.Initialize(owner,f.Config(true)).status,Code::Ok);
  const auto g=f.source.model.group_count();fe::NodalTrialToken token;fe::NodalAssemblyView view;
  ASSERT_TRUE(Begin(owner,8192,token,view));ASSERT_TRUE(Prepare(owner,token,view));
  std::vector<fe::NodalRigidGroupSnapshot> groups(g);fe::NodalPreparedView prepared;
  auto before=groups;EXPECT_EQ(owner.CopyPreparedRigidGroups(token,{groups.data(),g-1},&prepared).status,Code::ResourceLimit);
  SameGroups(groups,before);ASSERT_EQ(owner.CopyPreparedRigidGroups(token,{groups.data(),g},&prepared).status,Code::Ok);
  auto* bad_values=reinterpret_cast<double*>(1);auto* bad_groups=reinterpret_cast<fe::NodalRigidGroupAccelerationSnapshot*>(1);
  fe::NodalPreparedView capture_identity;
  EXPECT_EQ(owner.CopyPreparedForceStage(token,{bad_values,bad_values,f.n,bad_groups,g-1},&capture_identity).status,Code::ResourceLimit);
  std::vector<double> a(3*f.n),ar(3*f.n);std::vector<fe::NodalRigidGroupAccelerationSnapshot> capture(g);
  ASSERT_EQ(owner.CopyPreparedForceStage(token,{a.data(),ar.data(),f.n,capture.data(),g},&capture_identity).status,Code::Ok);
  EXPECT_TRUE(fe::trial_identity::SamePrepared(prepared,capture_identity));EXPECT_EQ(capture.back().source_group_id,8001023u);
  EXPECT_EQ(groups.back().source_group_id,8001023u);ASSERT_TRUE(Commit(owner,token,view));
  Snapshot nodes(f.n);ASSERT_TRUE(Read(owner,nodes));fe::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAcceptedRigidGroups({groups.data(),g},&stamp).status,Code::Ok);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,nodes.stamp));EXPECT_EQ(stamp.rigid_groups.member_count,8192u);
  const auto& property=f.source.model.groups()[g-1];std::array<rigid::MemberMotion,8> motion{};
  const auto* members=f.source.model.members()+property.member_offset;
  for(unsigned i=0;i<8;++i){const auto n=members[i].global_node;
    motion[i]={{nodes.v[3*n],nodes.v[3*n+1],nodes.v[3*n+2]},{nodes.w[3*n],nodes.w[3*n+1],nodes.w[3*n+2]}};}
  rigid::GroupKineticObservation result;ASSERT_TRUE(rigid::ObserveGroupKinetic({{&property,members,8},motion.data(),groups.back().state,
    {rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,f.h,.5*f.h,0}},result));
  EXPECT_GT(result.members.translation,0);EXPECT_GT(result.aggregate.translation,0);
}
} // namespace vehicle_rigid_owner_test
