#include "ForceStageCapturePacketFixture.h"
#include "GroupPhaseNativeFixture.h"

namespace force_stage_capture_test {
TEST(ForceStageCaptureNative,SixtyFourActualPacketStagesMatchPinnedNativePrimaryAndMemberAccelerations) {
  auto input=native::Fixture();input.body.omega={};
  for(auto& member:input.member){member.velocity=input.body.velocity;member.omega={};}
  native::NativeSchedule schedule;
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);input.body.durations=schedule.Next(1./4096);
    for(unsigned n=0;n<4;++n) {
      const auto original=native::Fixture().member[n];const double sign=step<16?1:step<32?-.625:0;
      input.member[n].force={sign*original.force.x,sign*original.force.y,sign*original.force.z};
      input.member[n].couple={sign*original.couple.x,sign*original.couple.y,sign*original.couple.z};
    }
    const auto reference=native::NativePacket(input);PacketCapture packet(input);
    ASSERT_EQ(rigid::PrepareGroupCandidate<true>(packet.view(),0,packet.accepted.data(),packet.captured.data(),
      packet.loads.data(),PacketNodes,input.body.durations,packet.sink()).status,rigid::StepStatus::Success);
    CheckAcceleration(packet,reference);native::CarryNative(reference,input);
  }
  EXPECT_GT(std::abs(input.body.omega.x),1e-4);
}
} // namespace force_stage_capture_test
