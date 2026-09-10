#include "GroupForceStageFixture.h"
#include <cstring>
#include <limits>
namespace rigid_observation_test {
TEST(NodalRigidForceStage,FirstAndSixtyFourChangingForceStagesMatchLongDoubleTensor) {
  ForceStageFixture f;
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);ASSERT_TRUE(f.kick.Solve());const auto in=f.Input();rigid::GroupForceStageKineticObservation out;
    ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,out));CheckForceStageOracle(in,out);
    if(!step) {EXPECT_EQ(out.collocated_primary.velocity.x,in.before_primary.velocity.x);EXPECT_EQ(out.collocated_primary.omega.x,0);}
    f.kick.Carry(step+1);
  }
}
TEST(NodalRigidForceStage,PrimaryAndCorrectedInertiaPartitionsUseTheSameNativeMetricOnce) {
  for(auto kind:{Fixture::Kind::Dense,Fixture::Kind::MeasurablePrimary,Fixture::Kind::CorrectedInertia}) {
    Fixture f(kind);std::array<rigid::ForceStageAcceleration,Count> a{};
    for(unsigned i=0;i<Count;++i)a[i]={{.3*(i+1),-.2,.5},{-.1,.2*(i+1),.3}};
    rigid::GroupForceStageKineticInput in{f.Metric(),f.motion.data(),a.data(),{f.state.velocity,f.state.omega},
      {{.1,-.3,.4},{-.2,.6,.1}},f.state.principal_axes,{.125,.0625,0,{.125,.1875,.25}}};
    rigid::GroupForceStageKineticObservation out;ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,out));CheckForceStageOracle(in,out);
  }
}
TEST(NodalRigidForceStage,WrongPhysicalInputsAreDetectableAndWrongScheduleIsRejected) {
  ForceStageFixture f;for(unsigned i=0;i<12;++i){ASSERT_TRUE(f.kick.Solve());f.kick.Carry(i+1);}ASSERT_TRUE(f.kick.Solve());
  const auto in=f.Input();rigid::GroupForceStageKineticObservation good,bad;
  ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,good));
  auto wrong=in;wrong.force_frame=f.kick.packet.body.previous_frame.axes;
  ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(wrong,bad));EXPECT_GT(std::abs(good.aggregate.rotation-bad.aggregate.rotation),1e-14);
  wrong=in;wrong.before_primary={f.kick.trial.primary.velocity,f.kick.trial.primary.omega};wrong.before_members=f.kick.after.data();
  ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(wrong,bad));EXPECT_GT(std::abs(good.aggregate.total-bad.aggregate.total),1e-8);
  wrong=in;wrong.phase.durations.kick_dt*=2;
  EXPECT_EQ(rigid::ObserveGroupForceStageKinetic(wrong,bad).status,Status::UnsupportedPhase);
  auto doubled=f.acceleration;for(auto& a:doubled){a.translation=rigid::detail::Add(a.translation,a.translation);a.rotation=rigid::detail::Add(a.rotation,a.rotation);}
  wrong=in;wrong.member_acceleration=doubled.data();ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(wrong,bad));
  EXPECT_GT(std::abs(good.members.total-bad.members.total),1e-8); // DT12 instead of DT1/2 on later fixed steps.
}
TEST(NodalRigidForceStage,RoundedReactionAndKickInversionCannotReplaceActualAcceleration) {
  Fixture f;std::array<Motion,Count> before{};std::array<rigid::ForceStageAcceleration,Count> a{};
  for(auto& motion:before)motion.velocity={1,0,0};a.back().translation={1,0,0};
  rigid::GroupForceStageKineticInput in{f.Metric(),before.data(),a.data(),{{1,0,0},{}},{},f.state.principal_axes,
    {1,.5,0,{1,1,1}}};
  rigid::GroupForceStageKineticObservation good,bad;ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,good));
  const double force=1e20,mass=f.source.back().mass_kg,reaction=mass*a.back().translation.x-force;
  const double recovered=(force+reaction)/mass;EXPECT_EQ(recovered,0);
  a.back().translation.x=recovered;ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,bad));
  EXPECT_GT(good.members.translation-bad.members.translation,.1);
  const double old=1e20,acceleration=1,kick=1,new_value=old+kick*acceleration;
  EXPECT_EQ((new_value-old)/kick,0); // Lost kick information cannot authenticate A.
  EXPECT_EQ(good.source_group_id,in.metric.group->source_group_id);EXPECT_EQ(good.member_count,Count);
}
TEST(NodalRigidForceStage,LateNonfiniteScopeAndAliasesPreserveAllOutputBytes) {
  ForceStageFixture f;ASSERT_TRUE(f.kick.Solve());f.kick.Carry(1);ASSERT_TRUE(f.kick.Solve());const auto good=f.Input();
  rigid::GroupForceStageKineticObservation out;ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(good,out));const auto before=out;
  auto reject=[&](auto in){EXPECT_FALSE(rigid::ObserveGroupForceStageKinetic(in,out));EXPECT_EQ(std::memcmp(&before,&out,sizeof(out)),0);};
  auto in=good;in.phase.force_time=std::numeric_limits<double>::infinity();reject(in);
  in=good;in.phase={1e308,9e307,8e307,{1e308,1e308,1e308}};reject(in);
  in=good;in.phase.durations.previous_drift_dt=0;reject(in);
  in=good;in.metric.member_count=257;in.before_members=reinterpret_cast<const Motion*>(1);reject(in);
  in=good;in.before_members=reinterpret_cast<const Motion*>(UINTPTR_MAX-4);reject(in);
  auto members=f.kick.fixture.source;members.back().source_node_id=members.front().source_node_id;in=good;in.metric.members=members.data();reject(in);
  auto a=f.acceleration;a.back().rotation.z=std::numeric_limits<double>::quiet_NaN();in=good;in.member_acceleration=a.data();reject(in);
  a=f.acceleration;a.back().rotation.z=std::numeric_limits<double>::max();in=good;in.member_acceleration=a.data();reject(in);
  for(unsigned which=0;which<4;++which){in=good;
    if(which==0)in.metric.group=reinterpret_cast<const fe::NodalRigidGroupProperties*>(&out);
    if(which==1)in.metric.members=reinterpret_cast<const fe::NodalRigidGroupMember*>(&out);
    if(which==2)in.before_members=reinterpret_cast<const Motion*>(&out);
    if(which==3)in.member_acceleration=reinterpret_cast<const rigid::ForceStageAcceleration*>(&out);
    reject(in);
  }
  alignas(rigid::GroupForceStageKineticObservation) unsigned char overlap[sizeof(in)+sizeof(out)]{};
  auto* aliased=new(overlap) rigid::GroupForceStageKineticInput(good);unsigned char saved[sizeof(overlap)];std::memcpy(saved,overlap,sizeof(saved));
  auto* target=reinterpret_cast<rigid::GroupForceStageKineticObservation*>(overlap);
  EXPECT_FALSE(rigid::ObserveGroupForceStageKinetic(*aliased,*target));EXPECT_EQ(std::memcmp(saved,overlap,sizeof(saved)),0);
}
} // namespace rigid_observation_test
