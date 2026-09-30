#include "GroupPhaseNativeFixture.h"
namespace rigid_step_test {
TEST(NodalRigidPhaseNative,FreshScheduleHasHalfKickAndPreservesUnequalDurations) {
  NativeSchedule schedule;
  auto dt=schedule.Next(.125); EXPECT_EQ(dt.previous_drift_dt,0); EXPECT_EQ(dt.kick_dt,.0625); EXPECT_EQ(dt.drift_dt,.125);
  dt=schedule.Next(.0625); EXPECT_EQ(dt.previous_drift_dt,.125); EXPECT_EQ(dt.kick_dt,.09375); EXPECT_EQ(dt.drift_dt,.0625);
  dt=schedule.Next(.03125); EXPECT_EQ(dt.previous_drift_dt,.0625); EXPECT_EQ(dt.kick_dt,.046875); EXPECT_EQ(dt.drift_dt,.03125);
}

TEST(NodalRigidPhaseNative,ThirtyTwoStepFreshOracleDetectsFullKickAndPrematureFrame) {
  auto input=Fixture(); input.body.omega={};
  for(auto& m:input.member) { m.velocity=input.body.velocity; m.omega={}; }
  NativeSchedule schedule; bool distinguished_frame=false,distinguished_new_spin=false;
  for(unsigned step=0;step<32;++step) {
    input.body.durations=schedule.Next(1./1024);
    const auto native=NativePacket(input);
    Trial candidate; ASSERT_EQ(EvaluatePacket(input,candidate),rigid::StepStatus::Success);
    Agreement(candidate,native);
    if(step==0) {
      auto wrong=input; wrong.body.durations.kick_dt=wrong.body.durations.drift_dt;
      const auto full=NativePacket(wrong);
      EXPECT_GT(std::abs(full.primary.omega.x-native.primary.omega.x),1e-5);
      EXPECT_GT(std::abs(full.member[0].position.x-native.member[0].position.x),1e-8);
      // ROTBMR still normalizes/crosses axes at zero duration; this is not a
      // bitwise copy even with zero spin and an already orthonormal input.
      const auto zero_frame=NativeFrame(input.body.previous_frame.axes,{},0);
      for(unsigned i=0;i<9;++i) EXPECT_EQ(native.primary.force_frame.axes.v[i],zero_frame.v[i]);
    } else {
      auto premature=input;
      // Wrongly retain an endpoint frame: rotate the stored force-stage frame
      // once more before the next native force operation rotates it again.
      const auto body_spin=rigid::detail::ToLocal(input.body.previous_frame.axes,input.body.omega);
      premature.body.previous_frame.axes=NativeFrame(input.body.previous_frame.axes,body_spin,input.body.durations.previous_drift_dt);
      const auto wrong=NativePacket(premature);
      distinguished_frame|=std::abs(wrong.primary.force_frame.axes.v[0]-native.primary.force_frame.axes.v[0])>1e-7;
      const auto new_spin=rigid::detail::ToLocal(input.body.previous_frame.axes,native.primary.omega);
      const auto wrong_spin_frame=NativeFrame(input.body.previous_frame.axes,new_spin,input.body.durations.previous_drift_dt);
      distinguished_new_spin|=std::abs(wrong_spin_frame.v[0]-native.primary.force_frame.axes.v[0])>1e-10;
    }
    CarryNative(native,input);
    if(step==15) for(auto& m:input.member) { m.force={}; m.couple={}; }
  }
  EXPECT_TRUE(distinguished_frame); EXPECT_TRUE(distinguished_new_spin);
}
} // namespace rigid_step_test
