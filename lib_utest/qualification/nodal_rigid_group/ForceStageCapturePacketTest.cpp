#include "ForceStageCapturePacketFixture.h"

namespace force_stage_capture_test {
TEST(ForceStageCapturePacket,EnabledSinkPreservesEveryTrialBitAndCopiesActualPacketAcceleration) {
  auto input=native::Fixture();input.body.durations={.001,.0015,.002};
  PacketCapture p(input);
  ASSERT_EQ(rigid::PrepareGroupCandidate(p.view(),0,p.accepted.data(),p.plain.data(),p.loads.data(),PacketNodes,input.body.durations).status,
    rigid::StepStatus::Success);
  ASSERT_EQ(rigid::PrepareGroupCandidate<true>(p.view(),0,p.accepted.data(),p.captured.data(),p.loads.data(),PacketNodes,input.body.durations,p.sink()).status,
    rigid::StepStatus::Success);
  EXPECT_EQ(0,std::memcmp(p.plain.data(),p.captured.data(),sizeof(p.plain)));
  native::Trial expected;ASSERT_EQ(native::EvaluatePacket(input,expected),rigid::StepStatus::Success);
  CheckAcceleration(p,expected);
}
TEST(ForceStageCapturePacket,DisabledSinkLeavesCapturePayloadUntouchedAndEnabledRequiresEveryRange) {
  auto input=native::Fixture();PacketCapture p(input);const auto old=p.accelerations;const auto group=p.primary;
  ASSERT_EQ(rigid::PrepareGroupCandidate(p.view(),0,p.accepted.data(),p.plain.data(),p.loads.data(),PacketNodes,input.body.durations,p.sink()).status,
    rigid::StepStatus::Success);
  EXPECT_EQ(p.accelerations,old);EXPECT_EQ(p.primary,group);
  for(unsigned missing=0;missing<4;++missing) {
    auto sink=p.sink();if(missing==0)sink.node=nullptr;if(missing==1)sink.node_rotation=nullptr;
    if(missing==2)sink.group=nullptr;
    if(missing==3)sink.group_rotation=nullptr;
    EXPECT_EQ(rigid::PrepareGroupCandidate<true>(p.view(),0,p.accepted.data(),p.captured.data(),p.loads.data(),PacketNodes,input.body.durations,sink).status,
      rigid::StepStatus::InvalidInput);
    EXPECT_EQ(p.captured,p.accepted);EXPECT_EQ(p.accelerations,old);EXPECT_EQ(p.primary,group);
  }
}
} // namespace force_stage_capture_test
