#include "TwoMemberOwnerFixture.h"
#include "GroupPhaseNativeFixture.h"
#include "lib_src/solvers/NodalRotation.h"
namespace rigid_two_owner_test {
TEST_F(Cuda,FourTwoMemberSourceShapesMatchNativeOwnerMotionReactionsAndForceCapture) {
  Fixture f;fe::FENodalState owner,disabled;
  ASSERT_EQ(f.Initialize(owner,true).status,Code::Ok);ASSERT_EQ(f.Initialize(disabled).status,Code::Ok);
  const auto allocation=owner.allocations();
  EXPECT_EQ(allocation.device_allocations,disabled.allocations().device_allocations);
  EXPECT_EQ(allocation.device_bytes-disabled.allocations().device_bytes,6*(9+4)*sizeof(double));
  std::array<rigid_step_test::Input,4> reference;
  for(unsigned g=0;g<4;++g)reference[g]=f.source.Packet(g);
  std::array<tl::math::Quaternion,8> q;for(auto& value:q)value={1,0,0,0};
  rt::NativeSchedule schedule;
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);const auto loads=f.Loads(step);const auto duration=schedule.Next(f.input.h);
    std::array<rt::Trial,4> expected;
    for(unsigned g=0;g<4;++g) {reference[g].body.durations=duration;
      for(unsigned i=0;i<2;++i) {reference[g].member[i].force=Node(loads.force,2*g+i);
        reference[g].member[i].couple=Node(loads.couple,2*g+i);}
      expected[g]=rt::NativeTwoPacket(reference[g],.001);
    }
    Groups before,still,candidate;ASSERT_TRUE(Read(owner,before));
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(ro::Prepare(owner,loads,token,view));
    fe::NodalPreparedView prepared,capture_identity;
    ASSERT_EQ(owner.CopyPreparedRigidGroups(token,candidate.buffer(),&prepared).status,Code::Ok);
    std::array<double,27> a{},ar{};std::array<fe::NodalRigidGroupAccelerationSnapshot,4> capture{};
    ASSERT_EQ(owner.CopyPreparedForceStage(token,{a.data(),ar.data(),9,capture.data(),4},&capture_identity).status,Code::Ok);
    EXPECT_TRUE(fe::trial_identity::SamePrepared(prepared,capture_identity));
    EXPECT_TRUE(fe::SameRigidGroupInfo(prepared.rigid_groups,{781,4,8}));
    ASSERT_TRUE(Read(owner,still));SameGroups(before,still);
    ASSERT_TRUE(ro::Accept(owner,token,view));ASSERT_TRUE(ro::Step(disabled,loads));SameOwners(owner,disabled);
    Groups accepted;nt::Snapshot nodes;ASSERT_TRUE(Read(owner,accepted));ASSERT_TRUE(nt::Read(owner,nodes));
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted.stamp,nodes.stamp));SameGroups(candidate,accepted);
    EXPECT_EQ(nodes.stamp.epoch,step+1);EXPECT_EQ(nodes.stamp.reaction_time,step*f.input.h);
    for(unsigned g=0;g<4;++g) {const auto& target=expected[g];const auto& group=accepted.values[g];
      EXPECT_EQ(group.source_group_id,rt::GroupIds[g]);EXPECT_EQ(group.source_node_set_id,rt::GroupIds[g]);
      EXPECT_EQ(capture[g].source_group_id,rt::GroupIds[g]);EXPECT_EQ(capture[g].member_count,2u);
      rt::Agreement(group.state.center,target.primary.center);rt::Agreement(group.state.velocity,target.primary.velocity);
      rt::Agreement(group.state.omega,target.primary.omega);rt::Agreement(capture[g].acceleration,target.primary.acceleration);
      rt::Agreement(capture[g].angular_acceleration,target.primary.angular_acceleration);
      for(unsigned j=0;j<9;++j)rt::Agreement(group.state.principal_axes.v[j],target.primary.force_frame.axes.v[j]);
      for(unsigned i=0;i<2;++i) {const unsigned n=2*g+i;const auto& member=target.member[i];
        rt::Agreement(Node(nodes.x.data(),n),member.position);rt::Agreement(Node(nodes.v.data(),n),member.velocity);
        rt::Agreement(Node(nodes.omega.data(),n),member.omega);rt::Agreement(Node(nodes.reaction.data(),n),member.reaction_force);
        rt::Agreement(Node(nodes.couple.data(),n),member.reaction_couple);rt::Agreement(Node(a.data(),n),member.acceleration);
        rt::Agreement(Node(ar.data(),n),member.angular_acceleration);
        const double increment[]{f.input.h*member.omega.x,f.input.h*member.omega.y,f.input.h*member.omega.z};
        tl::math::Quaternion next;ASSERT_TRUE(fe::nodal_detail::IncrementWorldRotation(q[n],increment,next));q[n]=next;
        const double values[]{next.w,next.x,next.y,next.z};for(unsigned j=0;j<4;++j)rt::Agreement(nodes.q[4*n+j],values[j]);
      }
      rt::Carry(target,reference[g]);
    }
    const double time=(step+1)*f.input.h;
    rt::Agreement(nodes.x[24],2-.125*time+time*time);EXPECT_EQ(a[24],2);EXPECT_EQ(ar[26],.25);
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
  }
}
} // namespace rigid_two_owner_test
