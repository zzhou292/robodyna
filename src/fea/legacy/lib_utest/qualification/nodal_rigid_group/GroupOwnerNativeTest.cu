#include "GroupOwnerFixture.h"
#include "GroupPhaseNativeFixture.h"
#include "lib_src/solvers/NodalRotation.h"
namespace rigid_owner_test {
namespace native=rigid_step_test;
namespace {
Vec3 Node(const double* values,std::size_t node) { return {values[3*node],values[3*node+1],values[3*node+2]}; }
native::Input NativeInitial(const Fixture& fixture,unsigned g) {
  native::Input input; const auto& p=fixture.model.groups()[g];
  input.body.previous_frame=p.principal; input.body.mass=p.total_mass_kg;
  input.body.center=p.center; input.body.velocity={.3,-.2,.1};
  for(unsigned m=0;m<4;++m) {
    const auto node=4*g+m; const auto& source=fixture.members[node];
    input.member[m]={source.position,{.3,-.2,.1},{},{},{},source.mass_kg,source.total_inertia_kg_m2};
  }
  return input;
}
void CompareGroup(const fe::NodalRigidGroupSnapshot& value,const native::Trial& reference) {
  native::Agreement(value.state.center,reference.primary.center);
  native::Agreement(value.state.velocity,reference.primary.velocity);
  native::Agreement(value.state.omega,reference.primary.omega);
  for(unsigned i=0;i<9;++i) native::Agreement(value.state.principal_axes.v[i],reference.primary.force_frame.axes.v[i]);
}
}

TEST_F(Cuda,SixtyFourFreshStepsMatchNativeGroupsMembersReactionsAndOrdinaryNode) {
  Fixture fixture; fe::FENodalState owner; ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  const auto allocation=owner.allocations();
  native::Input reference[]{NativeInitial(fixture,0),NativeInitial(fixture,1)};
  native::NativeSchedule schedule;
  std::array<tl::math::Quaternion,8> orientations;
  for(auto& q:orientations) q={1,0,0,0};
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step); const auto loads=fixture.Load(step); const auto duration=schedule.Next(fixture.input.h);
    native::Trial expected[2];
    for(unsigned g=0;g<2;++g) {
      reference[g].body.durations=duration;
      for(unsigned i=0;i<4;++i) {
        reference[g].member[i].force=Node(loads.force,4*g+i);
        reference[g].member[i].couple=Node(loads.couple,4*g+i);
      }
      expected[g]=native::NativePacket(reference[g]);
    }
    Groups before,still_accepted,candidate; ASSERT_TRUE(Read(owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(Prepare(owner,loads,token,view));
    fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedRigidGroups(token,candidate.buffer(),&prepared).status,Code::Ok);
    EXPECT_EQ(prepared.kick_dt,duration.kick_dt); EXPECT_EQ(prepared.base_time,step*fixture.input.h);
    EXPECT_EQ(prepared.velocity_time,(step+.5)*fixture.input.h);
    EXPECT_TRUE(fe::SameRigidGroupInfo(prepared.rigid_groups,{Source,2,8}));
    for(unsigned g=0;g<2;++g) CompareGroup(candidate.values[g],expected[g]);
    ASSERT_TRUE(Read(owner,still_accepted)); SameGroups(before,still_accepted);
    ASSERT_TRUE(Accept(owner,token,view));
    Groups accepted; ASSERT_TRUE(Read(owner,accepted)); SameGroups(candidate,accepted,false);
    nt::Snapshot nodes; ASSERT_TRUE(nt::Read(owner,nodes));
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted.stamp,nodes.stamp));
    EXPECT_EQ(nodes.stamp.epoch,step+1); EXPECT_EQ(nodes.stamp.time,(step+1)*fixture.input.h);
    EXPECT_EQ(nodes.stamp.velocity_time,(step+.5)*fixture.input.h);
    EXPECT_EQ(nodes.stamp.reaction_time,step*fixture.input.h);
    EXPECT_EQ(nodes.stamp.reaction_kick_dt,duration.kick_dt);
    for(unsigned g=0;g<2;++g) {
      CompareGroup(accepted.values[g],expected[g]);
      for(unsigned m=0;m<4;++m) {
        const auto node=4*g+m; const auto& value=expected[g].member[m];
        native::Agreement(Node(nodes.x.data(),node),value.position);
        native::Agreement(Node(nodes.v.data(),node),value.velocity);
        native::Agreement(Node(nodes.omega.data(),node),value.omega);
        native::Agreement(Node(nodes.reaction.data(),node),value.reaction_force);
        native::Agreement(Node(nodes.couple.data(),node),value.reaction_couple);
        // TL's shell orientation is intentionally separate from the native
        // lagged principal frame. Integrate qualified TL q using native omega.
        const double increment[]{fixture.input.h*value.omega.x,fixture.input.h*value.omega.y,fixture.input.h*value.omega.z};
        tl::math::Quaternion next;
        ASSERT_TRUE(fe::nodal_detail::IncrementWorldRotation(orientations[node],increment,next));
        orientations[node]=next;
        const double q[]{next.w,next.x,next.y,next.z};
        for(unsigned a=0;a<4;++a) native::Agreement(nodes.q[4*node+a],q[a]);
      }
      native::CarryNative(expected[g],reference[g]);
    }
    const auto free=fixture.input.n-1; const double t=(step+1)*fixture.input.h,vt=(step+.5)*fixture.input.h;
    // Independent closed form for the ordinary node's prescribed acceleration.
    native::Agreement(nodes.x[3*free],2-.125*t+t*t);
    native::Agreement(nodes.v[3*free],-.125+2*vt);
    native::Agreement(nodes.omega[3*free+2],.25*vt);
    for(unsigned a=0;a<3;++a) { EXPECT_EQ(nodes.reaction[3*free+a],0); EXPECT_EQ(nodes.couple[3*free+a],0); }
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
  }
  EXPECT_GT(std::abs(reference[0].body.omega.x),1e-3);
}
} // namespace rigid_owner_test
