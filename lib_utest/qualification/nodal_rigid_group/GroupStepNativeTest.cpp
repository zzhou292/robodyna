#include "GroupStepNativeFixture.h"

namespace rigid_step_test {
TEST(NodalRigidGroupStepNative,FrameMatchesOriginalZeroTinyBoundaryAndFiniteRotationBranches) {
  const auto frame=DenseFrame().axes;
  for(const auto omega:{Vec3{},Vec3{.36,.48,.8},Vec3{-.7,1.2,.4}})
    for(double dt:{0.,1e-7,std::nextafter(1e-5,0.),1e-5,std::nextafter(1e-5,1.),.2}) {
      tl::math::Matrix3 actual;
      ASSERT_EQ(rigid::RotatePrincipalFrame(frame,omega,dt,actual),rigid::MathStatus::Success);
      const auto reference=NativeFrame(frame,omega,dt);
      for(unsigned a=0;a<9;++a) Agreement(actual.v[a],reference.v[a]);
    }
}
TEST(NodalRigidGroupStepNative,CompleteWrenchSavedSpinGyroMemberReactionAndDriftMatchPinnedPackets) {
  for(const auto d:{rigid::StepDurations{1./128,1./128,1./128},
      rigid::StepDurations{1./256,3./512,1./128},rigid::StepDurations{.01,.0075,.005}}) {
    const auto input=Fixture(d); Trial actual;
    ASSERT_EQ(EvaluatePacket(input,actual),rigid::StepStatus::Success);
    Agreement(actual,NativePacket(input));
  }
}
TEST(NodalRigidGroupStepNative,ProposedTLHalfKickPacketHasExplicitScopeAndNoHiddenDurationSubstitution) {
  auto input=Fixture({0,1./256,1./128}); input.body.omega={};
  for(auto& member:input.member) { member.omega={}; member.velocity=input.body.velocity; }
  Trial actual; ASSERT_EQ(EvaluatePacket(input,actual),rigid::StepStatus::Success);
  Agreement(actual,NativePacket(input));
  auto full=input; full.body.durations.kick_dt=full.body.durations.drift_dt;
  Trial full_trial; ASSERT_EQ(EvaluatePacket(full,full_trial),rigid::StepStatus::Success);
  EXPECT_GT(std::abs(full_trial.primary.velocity.x-actual.primary.velocity.x),1e-4);
  RecordProperty("startup_scope","Authored TL durations (0,h/2,h); native packet arithmetic only, not engine startup parity");
}
TEST(NodalRigidGroupStepNative,UpdatedFrameAndSavedSpinOutputsRemainDistinctOnSuccessivePackets) {
  // Short explicitly carried packet sequence exercises the changing dense
  // frame. It neither calls a nodal owner nor establishes native engine timing.
  auto input=Fixture(); Input reference_input=input;
  for(unsigned step=0;step<8;++step) {
    Trial actual; ASSERT_EQ(EvaluatePacket(input,actual),rigid::StepStatus::Success);
    const auto reference=NativePacket(reference_input); Agreement(actual,reference);
    input.body.previous_frame=actual.primary.force_frame; input.body.center=actual.primary.center;
    input.body.velocity=actual.primary.velocity; input.body.omega=actual.primary.omega;
    reference_input.body.previous_frame=reference.primary.force_frame; reference_input.body.center=reference.primary.center;
    reference_input.body.velocity=reference.primary.velocity; reference_input.body.omega=reference.primary.omega;
    for(unsigned n=0;n<Count;++n) {
      input.member[n].position=actual.member[n].position; input.member[n].velocity=actual.member[n].velocity;
      input.member[n].omega=actual.member[n].omega;
      reference_input.member[n].position=reference.member[n].position; reference_input.member[n].velocity=reference.member[n].velocity;
      reference_input.member[n].omega=reference.member[n].omega;
    }
  }
}
} // namespace rigid_step_test
