#include "ForceStageCaptureFixture.h"
#include "GroupPhaseNativeFixture.h"

namespace force_stage_capture_test {
namespace native=rigid_step_test;
TEST_F(Cuda,ThirtyTwoOwnerForceStagesMatchNativePrimaryMemberAndOrdinaryAccelerations) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  native::Input reference[2];
  for(unsigned g=0;g<2;++g) {
    auto& in=reference[g];const auto& p=fixture.model.groups()[g];
    in.body.previous_frame=p.principal;in.body.mass=p.total_mass_kg;in.body.center=p.center;in.body.velocity={.3,-.2,.1};
    for(unsigned m=0;m<4;++m) {
      const auto& member=fixture.members[4*g+m];
      in.member[m]={member.position,in.body.velocity,{},{},{},member.mass_kg,member.total_inertia_kg_m2};
    }
  }
  native::NativeSchedule schedule;
  for(unsigned step=0;step<32;++step) {
    SCOPED_TRACE(step);const auto loads=fixture.Load(step);const auto duration=schedule.Next(fixture.input.h);
    native::Trial expected[2];
    for(unsigned g=0;g<2;++g) {
      reference[g].body.durations=duration;
      for(unsigned m=0;m<4;++m) {
        reference[g].member[m].force=CaptureNode(loads.force,4*g+m);
        reference[g].member[m].couple=CaptureNode(loads.couple,4*g+m);
      }
      expected[g]=native::NativePacket(reference[g]);
    }
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(Prepare(owner,loads,token,view));Capture sample;
    ASSERT_EQ(owner.CopyPreparedForceStage(token,sample.buffer(),&sample.prepared).status,Code::Ok);
    for(unsigned g=0;g<2;++g) {
      native::Agreement(sample.groups[g].acceleration,expected[g].primary.acceleration);
      native::Agreement(sample.groups[g].angular_acceleration,expected[g].primary.angular_acceleration);
      for(unsigned m=0;m<4;++m) {
        native::Agreement(CaptureNode(sample.a.data(),4*g+m),expected[g].member[m].acceleration);
        native::Agreement(CaptureNode(sample.ar.data(),4*g+m),expected[g].member[m].angular_acceleration);
      }
      native::CarryNative(expected[g],reference[g]);
    }
    EXPECT_EQ(sample.a[3*(fixture.input.n-1)],2);EXPECT_EQ(sample.ar[3*(fixture.input.n-1)+2],.25);
    ASSERT_TRUE(Accept(owner,token,view));
  }
}
} // namespace force_stage_capture_test
