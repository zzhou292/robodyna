#include "GroupStepTestSupport.h"
#include <limits>

namespace rigid_step_test {
TEST(NodalRigidGroupStepMath,SphericalPrimaryMatchesIndependentKickDriftForDistinctDurations) {
  for(const auto d:{rigid::StepDurations{0,1./256,1./128},
      rigid::StepDurations{1./128,1./128,1./128},rigid::StepDurations{1./256,3./512,1./128}}) {
    auto b=Fixture(d).body; b.previous_frame.inertia={4,4,4}; b.omega={};
    b.applied={{4,-8,12},{2,4,-6}}; rigid::PrimaryStepTrial trial;
    ASSERT_EQ(rigid::EvaluatePrimaryStep(b,trial),rigid::StepStatus::Success);
    Agreement(trial.acceleration,{.5,-1,1.5}); Agreement(trial.angular_acceleration,{.5,1,-1.5});
    for(unsigned i=0;i<3;++i) {
      const long double a=Get(b.applied.force,i)/static_cast<long double>(b.mass);
      const long double velocity=Get(b.velocity,i)+static_cast<long double>(d.kick_dt)*a;
      Agreement(Get(trial.velocity,i),static_cast<double>(velocity));
      Agreement(Get(trial.center,i),static_cast<double>(Get(b.center,i)+d.drift_dt*velocity));
      Agreement(Get(trial.omega,i),d.kick_dt*Get(b.applied.couple,i)/4);
    }
  }
  RecordProperty("zero_previous_duration_scope","Proposed TL physical-rest half-kick mapping; not native engine startup proof");
}
TEST(NodalRigidGroupStepMath,NativeFrameTracksIndependentRodriguesAcrossTinyAngleFloor) {
  const auto initial=DenseFrame().axes;
  for(double dt:{0.,1e-7,std::nextafter(1e-5,0.),1e-5,std::nextafter(1e-5,1.),.2}) {
    const Vec3 omega{.36,.48,.8}; tl::math::Matrix3 actual;
    ASSERT_EQ(rigid::RotatePrincipalFrame(initial,omega,dt,actual),rigid::MathStatus::Success);
    const long double theta=dt,c=std::cos(theta),s=std::sin(theta),u[]{.36L,.48L,.8L};
    const long double skew[3][3]{{0,-u[2],u[1]},{u[2],0,-u[0]},{-u[1],u[0],0}};
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
      long double expected=0;
      for(unsigned k=0;k<3;++k) expected+=initial.v[3*i+k]*((k==j?c:0)+(1-c)*u[k]*u[j]+s*skew[k][j]);
      Agreement(actual.v[3*i+j],static_cast<double>(expected));
    }
  }
}
TEST(NodalRigidGroupStepMath,SecondOrderMemberDriftHasAnalyticDistanceGrowthAndImpulseReactions) {
  auto b=Fixture({.1,.1,.1}).body;
  b.previous_frame={{{1,0,0,0,1,0,0,0,1}},{1,1,1}};
  b.center={}; b.velocity={}; b.omega={0,0,2}; b.applied={};
  rigid::PrimaryStepTrial primary; ASSERT_EQ(rigid::EvaluatePrimaryStep(b,primary),rigid::StepStatus::Success);
  rigid::MemberStepInput in{{1,0,0},{0,2,0},{0,0,2},{3,-4,2},{.1,.2,.3},2,.001};
  rigid::MemberStepTrial trial; ASSERT_EQ(rigid::EvaluateMemberStep(b,primary,in,trial),rigid::StepStatus::Success);
  Agreement(trial.position,{.98,.2,0});
  const double length2=trial.position.x*trial.position.x+trial.position.y*trial.position.y;
  Agreement(length2,1+std::pow(.2,4)/4); EXPECT_GT(length2,1);
  for(unsigned a=0;a<3;++a) {
    Agreement(in.mass*(Get(trial.velocity,a)-Get(in.velocity,a))-.1*Get(in.force,a),.1*Get(trial.reaction_force,a));
    Agreement(in.inertia*(Get(trial.omega,a)-Get(in.omega,a))-.1*Get(in.couple,a),.1*Get(trial.reaction_couple,a));
  }
}
TEST(NodalRigidGroupStepMath,CompletePacketRetainsEveryMemberForceAndCouple) {
  auto with=Fixture(); Trial a,b; ASSERT_EQ(EvaluatePacket(with,a),rigid::StepStatus::Success);
  auto without=with; for(auto& member:without.member) member.couple={};
  ASSERT_EQ(EvaluatePacket(without,b),rigid::StepStatus::Success);
  Agreement(a.primary.acceleration,b.primary.acceleration,0);
  const auto change=rigid::detail::Subtract(a.primary.angular_acceleration,b.primary.angular_acceleration);
  EXPECT_GT(rigid::detail::Dot(change,change),.01);
  for(unsigned i=0;i<Count;++i) Agreement(a.member[i].omega,a.primary.omega);
  // A native frame update retains saved pre-update VI, independent of caller output reuse.
  Agreement(a.primary.saved_body_omega,rigid::detail::ToLocal(with.body.previous_frame.axes,with.body.omega),0);
}
TEST(NodalRigidGroupStepMath,LateInvalidMembersAndOverflowPreserveWholePacketAndRetry) {
  const auto clean=Fixture(); Trial expected; ASSERT_EQ(EvaluatePacket(clean,expected),rigid::StepStatus::Success);
  for(unsigned failure=0;failure<4;++failure) {
    auto in=clean; auto held=expected; const auto bytes=Bytes(held);
    if(failure==0) in.member[Count-1].mass=0;
    if(failure==1) in.member[Count-1].position.x=std::numeric_limits<double>::max();
    if(failure==2) in.body.durations.kick_dt=0;
    if(failure==3) in.body.previous_frame.axes.v[0]=2;
    EXPECT_NE(EvaluatePacket(in,held),rigid::StepStatus::Success); EXPECT_EQ(Bytes(held),bytes);
    ASSERT_EQ(EvaluatePacket(clean,held),rigid::StepStatus::Success); Agreement(held,expected,0);
  }
}
TEST(NodalRigidGroupStepMath,SourceOldSpinLimitIsStrictAndDoesNotAlterPriorValueOutput) {
  auto b=Fixture({.125,.125,.125}).body; b.omega={0,0,8};
  rigid::PrimaryStepTrial out; ASSERT_EQ(rigid::EvaluatePrimaryStep(b,out),rigid::StepStatus::Success);
  const auto bytes=Bytes(out); b.omega.z=std::nextafter(8.,9.);
  EXPECT_EQ(rigid::EvaluatePrimaryStep(b,out),rigid::StepStatus::RotationLimit); EXPECT_EQ(Bytes(out),bytes);
  b.omega.z=std::numeric_limits<double>::max();
  EXPECT_EQ(rigid::EvaluatePrimaryStep(b,out),rigid::StepStatus::NonfiniteResult); EXPECT_EQ(Bytes(out),bytes);
}
} // namespace rigid_step_test
